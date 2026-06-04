#!/usr/bin/env python
# -- coding: utf-8 --

import sys
import math
import heapq
from collections import deque
import numpy as np

try:
    import carla
except ImportError:
    sys.exit("ERROR: could not import the 'carla' module.")

# --- Configuration ---
HOST = "127.0.0.1"
PORT = 2000
TIMEOUT = 60.0
SPAWN_INDEX_A = 0
SPAWN_INDEX_B = 100
TARGET_SPEED_KMH = 22.0
WAYPOINT_REACHED_RADIUS = 3.0
VEHICLE_BLUEPRINT = "vehicle.tesla.model3"

class DijkstraRoutePlanner:
    def _init_(self, carla_map, sampling_resolution=2.0):
        self._map = carla_map
        self._resolution = sampling_resolution
        self._graph = {}
        self._id_to_location = {}
        self._build_graph()

    @staticmethod
    def _node_id(waypoint):
        loc = waypoint.transform.location
        return (round(loc.x), round(loc.y), round(loc.z))

    def _densify_segment(self, entry_wp, exit_wp):
        end_loc = exit_wp.transform.location
        path = [entry_wp]
        if entry_wp.transform.location.distance(end_loc) > self._resolution:
            nxt = entry_wp.next(self._resolution)
            if nxt:
                w = nxt[0]
                guard = 0
                while w.transform.location.distance(end_loc) > self._resolution and guard < 10000:
                    path.append(w)
                    nxt = w.next(self._resolution)
                    if not nxt: break
                    w = nxt[0]
                    guard += 1
                path.append(exit_wp)
        return path

    def _build_graph(self):
        for entry_wp, exit_wp in self._map.get_topology():
            n1, n2 = self._node_id(entry_wp), self._node_id(exit_wp)
            self._id_to_location[n1], self._id_to_location[n2] = entry_wp.transform.location, exit_wp.transform.location
            dense = self._densify_segment(entry_wp, exit_wp)
            weight = sum(a.transform.location.distance(b.transform.location) for a, b in zip(dense[:-1], dense[1:]))
            self._graph.setdefault(n1, {})
            existing = self._graph[n1].get(n2)
            if existing is None or weight < existing[0]:
                self._graph[n1][n2] = (weight, dense)

    def trace_route(self, start_loc, end_loc):
        start_id = min(self._id_to_location, key=lambda k: self._id_to_location[k].distance(start_loc))
        goal_id = min(self._id_to_location, key=lambda k: self._id_to_location[k].distance(end_loc))
        dist, prev, pq = {start_id: 0.0}, {}, [(0.0, start_id)]
        
        while pq:
            cost, u = heapq.heappop(pq)
            if u == goal_id: break
            for v, (w, _) in self._graph.get(u, {}).items():
                if cost + w < dist.get(v, float("inf")):
                    dist[v], prev[v] = cost + w, u
                    heapq.heappush(pq, (cost + w, v))
                    
        route, curr = [], goal_id
        while curr in prev:
            _, dense = self._graph[prev[curr]][curr]
            route = dense[-2:0:-1] + route
            curr = prev[curr]
        return route

class VehicleController:
    def _init_(self):
        self._lon_kp = 1.0
        self._lat_kp = 1.5

    def run_step(self, vehicle, target_loc, target_speed):
        v_tf = vehicle.get_transform()
        v_loc, v_f = v_tf.location, v_tf.get_forward_vector()
        
        # Steering (Lateral)
        to_target = np.array([target_loc.x - v_loc.x, target_loc.y - v_loc.y])
        to_target_norm = np.linalg.norm(to_target)
        if to_target_norm < 0.001:
            return carla.VehicleControl(throttle=0.0, steer=0.0, brake=1.0)
            
        forward = np.array([v_f.x, v_f.y])
        dot = np.dot(forward, to_target / to_target_norm)
        angle = math.acos(np.clip(dot, -1.0, 1.0))
        steer = np.clip(angle * self._lat_kp, -1.0, 1.0) * np.sign(forward[0]*to_target[1] - forward[1]*to_target[0])
        
        # Throttle (Longitudinal)
        speed = 3.6 * math.sqrt(vehicle.get_velocity().x*2 + vehicle.get_velocity().y*2)
        throttle = np.clip((target_speed - speed) * self._lon_kp, 0.0, 1.0)
        
        return carla.VehicleControl(throttle=throttle, steer=float(steer))

def main():
    client = carla.Client(HOST, PORT)
    client.set_timeout(TIMEOUT)
    world = client.get_world()
    
    # Store original settings to restore them later
    original_settings = world.get_settings()
    
    try:
        # Set synchronous mode
        settings = world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = 0.05
        world.apply_settings(settings)
        
        spawn_points = world.get_map().get_spawn_points()
        blueprint = world.get_blueprint_library().find(VEHICLE_BLUEPRINT)
        
        # Spawn the vehicle
        vehicle = world.spawn_actor(blueprint, spawn_points[SPAWN_INDEX_A])
        print(f"Vehicle spawned at Point A (Index {SPAWN_INDEX_A})")
        
        # Plan the route using custom Dijkstra
        planner = DijkstraRoutePlanner(world.get_map())
        print("Planning route to Point B... This may take a moment.")
        route = planner.trace_route(vehicle.get_location(), spawn_points[SPAWN_INDEX_B].location)
        route_q = deque(route)
        print(f"Route planned with {len(route)} waypoints.")
        
        # --- Draw the planned route as a continuous green line ---
        print("Drawing continuous route line in the simulator...")
        for i in range(len(route) - 1):
            # הוספת גובה כדי שהקווים יהיו מעל פני הכביש ולא ייבלעו באספלט
            loc1 = route[i].transform.location + carla.Location(z=0.5)
            loc2 = route[i+1].transform.location + carla.Location(z=0.5)
            world.debug.draw_line(
                loc1, 
                loc2, 
                thickness=0.2, 
                color=carla.Color(0, 255, 0), # ירוק
                life_time=120.0 # נשאר לזמן ארוך
            )
        
        controller = VehicleController()
        spectator = world.get_spectator()

        while route_q:
            world.tick()
            
            # --- Spectator Camera Follow Logic (FIXED) ---
            transform = vehicle.get_transform()
            fwd = transform.get_forward_vector()
            # מיקום המצלמה: 6 מטרים מאחורי הרכב, 3 מטרים מעליו
            cam_location = transform.location + carla.Location(x=-fwd.x * 6.0, y=-fwd.y * 6.0, z=3.0)
            
            spectator_transform = carla.Transform(cam_location, transform.rotation)
            spectator_transform.rotation.pitch -= 15.0 # הטיית המצלמה מעט כלפי מטה
            spectator.set_transform(spectator_transform)
            
            # Look-ahead: avoid stopping abruptly at each point
            lookahead_index = min(5, len(route_q) - 1)
            target = route_q[lookahead_index].transform.location
            
            if vehicle.get_location().distance(route_q[0].transform.location) < WAYPOINT_REACHED_RADIUS:
                route_q.popleft()
                
            vehicle.apply_control(controller.run_step(vehicle, target, TARGET_SPEED_KMH))
            
        print("Destination reached successfully!")
        
    finally:
        print("Cleaning up...")
        if 'vehicle' in locals() and vehicle is not None:
            vehicle.destroy()
        # Restore original world settings
        world.apply_settings(original_settings)

if __name__ == "__main__":
    main()