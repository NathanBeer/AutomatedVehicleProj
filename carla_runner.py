"""
CARLA migration entry point.
"""
from __future__ import annotations

import argparse
import math
import queue
import sys
import time
import glob
from typing import List, Optional
import numpy as np

print("--- SCRIPT STARTED ---")

# ── Existing project modules ─────────────────────────────────────────────────
from grid import Config
from algorithms import astar
from carla_grid import WorldGrid
from carla_controller import WaypointFollower

try:
    carla_egg = glob.glob('C:/Carla/WindowsNoEditor/PythonAPI/carla/dist/carla-*.egg')[0]
    sys.path.append(carla_egg)
except IndexError:
    print("Error: Could not find carla .egg file!")
    sys.exit()

import carla

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

# ─────────────────────────────────────────────────────────────────────────────
# Main runner
# ─────────────────────────────────────────────────────────────────────────────

def run(host="127.0.0.1", port=2000, map_name="Town04", spawn_idx=0, goal_idx=10, 
        target_speed_kmh=30.0, cell_size=2.0, enable_lidar=False, max_ticks=4000) -> None:
    
    client = carla.Client(host, port)
    client.set_timeout(20.0)
    world = client.get_world()
    
    if not world.get_map().name.endswith(map_name):
        world = client.load_world(map_name)

    # --- מנגנון ניקוי אוטומטי למניעת Collision ---
    print("[CARLA] Cleaning existing vehicles...")
    for actor in world.get_actors().filter('vehicle.*'):
        actor.destroy()
    # ---------------------------------------------

    actors: List[carla.Actor] = []
    original_settings = world.get_settings()

    try:
        delta = 0.05
        s = world.get_settings()
        s.synchronous_mode = True
        s.fixed_delta_seconds = delta
        world.apply_settings(s)
        world.tick()

        ego = _spawn_ego(world, spawn_idx)
        actors.append(ego)
        print(f"[CARLA] Ego vehicle spawned (id={ego.id})")

        spectator = world.get_spectator()
        spawn_pts = world.get_map().get_spawn_points()
        start_loc = spawn_pts[spawn_idx % len(spawn_pts)].location
        goal_loc = spawn_pts[goal_idx % len(spawn_pts)].location

        # הבנייה והתכנון
        wg = WorldGrid(world, start_loc, goal_loc, cell_size=cell_size)
        cfg = Config(allow_diagonal=True, avoid_corner_cutting=True)
        path_grid = astar(wg.grid, wg.world_to_grid(start_loc), wg.world_to_grid(goal_loc), cfg)
        
        if not path_grid:
            sys.exit("[Path] No path found. Try different spawn/goal indices.")
        
        path_world = [wg.grid_to_world(c) for c in path_grid]
        follower = WaypointFollower(path_world, target_speed_kmh=target_speed_kmh, dt=delta)

        # המתנה של 10 שניות לפני תחילת נסיעה
        print("[CARLA] Holding position for 10 seconds...")
        for _ in range(200):
            world.tick()
            transform = ego.get_transform()
            forward = transform.get_forward_vector()
            spectator.set_transform(carla.Transform(transform.location - carla.Location(x=forward.x*8, y=forward.y*8, z=-3), transform.rotation))
            ego.apply_control(carla.VehicleControl(brake=1.0))

        # לולאת נסיעה
        t0 = time.monotonic()
        for tick in range(1, max_ticks + 1):
            world.tick()
            transform = ego.get_transform()
            vel = ego.get_velocity()
            speed_ms = math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)

            # עדכון מצלמה
            forward = transform.get_forward_vector()
            spectator.set_transform(carla.Transform(transform.location - carla.Location(x=forward.x*8, y=forward.y*8, z=-3), transform.rotation))

            control = follower.run_step(transform, speed_ms)
            ego.apply_control(control)

            if follower.done:
                print("[CARLA] Goal reached. Holding for 5 seconds...")
                for _ in range(100):
                    world.tick()
                    time.sleep(0.05)
                break

    except KeyboardInterrupt:
        print("\n[CARLA] Interrupted.")
    finally:
        print("[CARLA] Destroying actors...")
        for actor in actors:
            if actor.is_alive: actor.destroy()
        world.apply_settings(original_settings)
        print("[CARLA] Clean exit.")

if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--map", default="Town04")
    p.add_argument("--spawn", type=int, default=0)
    p.add_argument("--goal", type=int, default=10)
    p.add_argument("--speed", type=float, default=30.0)
    p.add_argument("--cell-size", type=float, default=2.0)
    a = p.parse_args()
    run(map_name=a.map, spawn_idx=a.spawn, goal_idx=a.goal, target_speed_kmh=a.speed, cell_size=a.cell_size)