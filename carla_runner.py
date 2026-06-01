"""
CARLA migration entry point.

Preserves:  grid.py · algorithms.py  (zero modifications)
Replaces:   hard-coded grid  → CARLA HD map occupancy grid (WorldGrid)
            matplotlib plot  → live CARLA sensor feeds
            single-shot run  → synchronous 20 Hz control loop

Prerequisites
─────────────
  pip install carla numpy
  CARLA server:  ./CarlaUE4.sh  (Linux)  or  CarlaUE4.exe  (Windows)
  CARLA Python egg on PYTHONPATH (see CARLA/PythonAPI/carla/dist/)

Usage
─────
  python carla_runner.py
  python carla_runner.py --map Town01 --spawn 0 --goal 15 --speed 40
  python carla_runner.py --lidar --cell-size 1.5
"""
from __future__ import annotations

import argparse
import math
import queue
import sys
import time
import glob
import os
from typing import List, Optional

import carla
import numpy as np
import sys
print("--- SCRIPT STARTED ---")
print("Python executable:", sys.executable)
# ── Existing project modules (UNCHANGED) ─────────────────────────────────────
from grid import Config
from algorithms import astar

# ── New CARLA-specific modules ────────────────────────────────────────────────
from carla_grid import WorldGrid
from carla_controller import WaypointFollower

try:
    carla_egg = glob.glob('C:/Carla/WindowsNoEditor/PythonAPI/carla/dist/carla-*.egg')[0]
    sys.path.append(carla_egg)
    print(f"Using CARLA library from: {carla_egg}")
except IndexError:
    print("Error: Could not find carla .egg file in the expected path!")
    sys.exit()

import carla
# ─────────────────────────────────────────────────────────────────────────────
# Sensor callback factories
# ─────────────────────────────────────────────────────────────────────────────

def _rgb_callback(buf: queue.Queue):
    def _cb(image: carla.Image) -> None:
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        arr = arr.reshape((image.height, image.width, 4))[:, :, :3]  # BGRA → BGR
        try:
            buf.put_nowait(arr)
        except queue.Full:
            pass  # drop frame rather than stalling the CARLA callback thread
    return _cb


def _lidar_callback(buf: queue.Queue):
    def _cb(pc: carla.LidarMeasurement) -> None:
        pts = np.frombuffer(pc.raw_data, dtype=np.float32).reshape((-1, 4))
        try:
            buf.put_nowait(pts)   # shape: (N, 4) → [x, y, z, intensity]
        except queue.Full:
            pass
    return _cb


# ─────────────────────────────────────────────────────────────────────────────
# Actor helpers
# ─────────────────────────────────────────────────────────────────────────────

def _spawn_ego(world: carla.World, spawn_idx: int) -> carla.Vehicle:
    bp = world.get_blueprint_library().filter("vehicle.lincoln.mkz_2017")[0]
    spawn_pts = world.get_map().get_spawn_points()
    if not spawn_pts:
        raise RuntimeError("Map has no spawn points.")
    transform = spawn_pts[spawn_idx % len(spawn_pts)]
    actor = world.spawn_actor(bp, transform)
    actor.set_autopilot(False)
    return actor


def _attach_camera(
    world: carla.World,
    parent: carla.Actor,
    buf: queue.Queue,
    w: int = 800,
    h: int = 600,
) -> carla.Sensor:
    bp = world.get_blueprint_library().find("sensor.camera.rgb")
    bp.set_attribute("image_size_x", str(w))
    bp.set_attribute("image_size_y", str(h))
    bp.set_attribute("fov", "110")
    tf = carla.Transform(carla.Location(x=2.0, z=1.6))   # front-facing, hood mount
    s = world.spawn_actor(bp, tf, attach_to=parent)
    s.listen(_rgb_callback(buf))
    return s


def _attach_lidar(
    world: carla.World,
    parent: carla.Actor,
    buf: queue.Queue,
) -> carla.Sensor:
    bp = world.get_blueprint_library().find("sensor.lidar.ray_cast")
    bp.set_attribute("channels", "32")
    bp.set_attribute("range", "50")
    bp.set_attribute("points_per_second", "56000")
    bp.set_attribute("rotation_frequency", "20")
    bp.set_attribute("upper_fov", "2.0")
    bp.set_attribute("lower_fov", "-24.8")
    tf = carla.Transform(carla.Location(x=0.0, z=2.4))   # roof-mounted
    s = world.spawn_actor(bp, tf, attach_to=parent)
    s.listen(_lidar_callback(buf))
    return s


# ─────────────────────────────────────────────────────────────────────────────
# Main runner
# ─────────────────────────────────────────────────────────────────────────────

def run(
    host: str = "127.0.0.1",
    port: int = 2000,
    map_name: str = "Town04",
    spawn_idx: int = 0,
    goal_idx: int = 10,
    target_speed_kmh: float = 30.0,
    cell_size: float = 2.0,
    enable_lidar: bool = False,
    max_ticks: int = 4000,
) -> None:
    # ── Connect to CARLA ──────────────────────────────────────────────────────
    client = carla.Client(host, port)
    client.set_timeout(20.0)

    # Reload map only if needed (avoids a 30-second wait when already correct)
    world = client.get_world()
    if not world.get_map().name.endswith(map_name):
        print(f"[CARLA] Loading map {map_name} …")
        world = client.load_world(map_name)
    print(f"[CARLA] Connected  map={world.get_map().name}")

    actors: List[carla.Actor] = []
    original_settings: Optional[carla.WorldSettings] = None

    try:
        # ── Enable synchronous mode ───────────────────────────────────────────
        delta = 0.05   # 20 Hz fixed timestep
        original_settings = world.get_settings()
        s = world.get_settings()
        s.synchronous_mode = True
        s.fixed_delta_seconds = delta
        world.apply_settings(s)
        world.tick()   # settle

        # ── Spawn ego vehicle ─────────────────────────────────────────────────
        ego = _spawn_ego(world, spawn_idx)
        actors.append(ego)
        print(f"[CARLA] Ego vehicle id={ego.id}")

        # ── Resolve start / goal world locations ──────────────────────────────
        spawn_pts = world.get_map().get_spawn_points()
        start_loc = spawn_pts[spawn_idx  % len(spawn_pts)].location
        goal_loc  = spawn_pts[goal_idx   % len(spawn_pts)].location
        print(
            f"[Path]  start=({start_loc.x:.1f}, {start_loc.y:.1f})  "
            f"goal=({goal_loc.x:.1f}, {goal_loc.y:.1f})"
        )

        # ── Build occupancy grid from CARLA HD map ────────────────────────────
        print("[Path]  Building occupancy grid …")
        wg = WorldGrid(world, start_loc, goal_loc, cell_size=cell_size)
        print(f"[Path]  Grid: {wg.rows} rows × {wg.cols} cols  "
              f"({cell_size}m/cell)")

        # ── Plan path with A* — algorithms.py is UNCHANGED ───────────────────
        cfg = Config(allow_diagonal=True, avoid_corner_cutting=True)
        start_coord = wg.world_to_grid(start_loc)
        goal_coord  = wg.world_to_grid(goal_loc)
        print(f"[Path]  A* {start_coord} → {goal_coord} …")

        path_grid = astar(wg.grid, start_coord, goal_coord, cfg)
        if not path_grid:
            sys.exit(
                "[Path]  No path found. Try increasing --cell-size, "
                "choosing different spawn/goal indices, or using Town01."
            )
        print(f"[Path]  Found {len(path_grid)} grid waypoints.")

        # ── Translate grid path → CARLA world locations ───────────────────────
        path_world: List[carla.Location] = [
            wg.grid_to_world(c) for c in path_grid
        ]

        # ── Attach sensors ────────────────────────────────────────────────────
        img_q: queue.Queue = queue.Queue(maxsize=5)
        cam = _attach_camera(world, ego, img_q)
        actors.append(cam)

        lid_q: Optional[queue.Queue] = None
        if enable_lidar:
            lid_q = queue.Queue(maxsize=5)
            lid = _attach_lidar(world, ego, lid_q)
            actors.append(lid)

        world.tick()   # give sensors one tick to initialise

        # ── Instantiate follower (feeds on A* output) ─────────────────────────
        follower = WaypointFollower(
            path_world,
            target_speed_kmh=target_speed_kmh,
            dt=delta,
        )

        # ─────────────────────────────────────────────────────────────────────
        # Synchronous control loop
        # ─────────────────────────────────────────────────────────────────────
        print("[Loop]  Running — Ctrl-C to stop.")
        t0 = time.monotonic()

        for tick in range(1, max_ticks + 1):
            # 1. Advance the simulation one fixed timestep
            world.tick()

            # 2. Retrieve latest sensor data (non-blocking; None if not ready)
            latest_rgb: Optional[np.ndarray] = None
            try:
                latest_rgb = img_q.get_nowait()
            except queue.Empty:
                pass

            latest_lidar: Optional[np.ndarray] = None
            if lid_q is not None:
                try:
                    latest_lidar = lid_q.get_nowait()
                except queue.Empty:
                    pass

            # ── INSERT YOUR CV / AI MODEL INFERENCE HERE ──────────────────────
            #
            #   The following variables are available each tick:
            #     latest_rgb   : np.ndarray (H, W, 3) BGR, or None
            #     latest_lidar : np.ndarray (N, 4) [x,y,z,intensity], or None
            #
            #   Example:
            #     if latest_rgb is not None:
            #         detections = my_model.infer(latest_rgb)
            #         obstacle_cells = detections_to_grid(detections, wg)
            #         path_grid = astar(wg.grid, current_coord, goal_coord, cfg)
            #         follower = WaypointFollower(
            #             [wg.grid_to_world(c) for c in path_grid], ...)
            # ─────────────────────────────────────────────────────────────────

            # 3. Read ego state
            transform   = ego.get_transform()
            vel         = ego.get_velocity()
            speed_ms    = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)

            # 4. Compute and apply vehicle control
            control = follower.run_step(transform, speed_ms)
            ego.apply_control(control)

            # 5. Console log at ~1 Hz
            if tick % 20 == 0:
                elapsed = time.monotonic() - t0
                print(
                    f"  t={elapsed:6.1f}s  tick={tick:5d}  "
                    f"speed={speed_ms * 3.6:5.1f} km/h  "
                    f"thr={control.throttle:.2f}  "
                    f"steer={control.steer:+.3f}  "
                    f"brake={control.brake:.2f}  "
                    f"rgb={'✓' if latest_rgb is not None else '–'}"
                )

            # 6. Terminate when goal is reached
            if follower.done:
                elapsed = time.monotonic() - t0
                print(f"[Loop]  Goal reached  ticks={tick}  t={elapsed:.1f}s")
                break

        else:
            print(f"[Loop]  Max ticks ({max_ticks}) reached without reaching goal.")

    except KeyboardInterrupt:
        print("\n[CARLA] Interrupted.")

    finally:
        # ── Strict cleanup: destroy all actors, restore world settings ─────────
        print("[CARLA] Destroying actors …")
        for actor in reversed(actors):   # sensors before vehicle
            try:
                if actor.is_alive:
                    actor.destroy()
            except Exception as exc:
                print(f"  warn: could not destroy {actor.id}: {exc}")

        if original_settings is not None:
            world.apply_settings(original_settings)
            print("[CARLA] World settings restored.")

        print("[CARLA] Clean exit.")


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────

def _args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="CARLA AV migration runner")
    p.add_argument("--host",      default="127.0.0.1")
    p.add_argument("--port",      type=int,   default=2000)
    p.add_argument("--map",       default="Town04", dest="map_name",
                   help="CARLA map name, e.g. Town01 Town04 Town10HD")
    p.add_argument("--spawn",     type=int,   default=0,    dest="spawn_idx",
                   help="Ego vehicle spawn point index")
    p.add_argument("--goal",      type=int,   default=10,   dest="goal_idx",
                   help="Goal spawn point index")
    p.add_argument("--speed",     type=float, default=30.0, dest="target_speed_kmh",
                   help="Target cruise speed in km/h")
    p.add_argument("--cell-size", type=float, default=2.0,  dest="cell_size",
                   help="Grid cell edge length in metres (smaller = more detail, slower planning)")
    p.add_argument("--lidar",     action="store_true", dest="enable_lidar",
                   help="Attach a 32-channel LIDAR sensor")
    p.add_argument("--max-ticks", type=int,   default=4000, dest="max_ticks")
    return p.parse_args()


if __name__ == "__main__":
    a = _args()
    run(
        host=a.host,
        port=a.port,
        map_name=a.map_name,
        spawn_idx=a.spawn_idx,
        goal_idx=a.goal_idx,
        target_speed_kmh=a.target_speed_kmh,
        cell_size=a.cell_size,
        enable_lidar=a.enable_lidar,
        max_ticks=a.max_ticks,
    )