#!/usr/bin/env python
# -- coding: utf-8 --
"""
============================================================================
 START.py  -  Launcher for the CARLA Dijkstra navigation system
============================================================================

This is the file you run. It just configures and starts the main program in
carla_dijkstra_navigation.py.

Both files must sit in the SAME folder.

Quick start (uses the defaults inside the main script):
    python START.py

With options:
    python START.py --host 127.0.0.1 --port 2000 --a 0 --b 100 --speed 25

Use explicit coordinates instead of spawn-point indices:
    python START.py --ax 10 --ay 5 --bx 120 --by 60

Run "python START.py --help" to see everything.
============================================================================
"""

import sys
import argparse

# Import the main program as a module so we can override its configuration
# and then call its main() function.
try:
    import carla_dijkstra_navigation as nav
except ImportError:
    sys.exit("ERROR: 'carla_dijkstra_navigation.py' was not found.\n"
             "Put START.py in the SAME folder as carla_dijkstra_navigation.py.")


def parse_args():
    p = argparse.ArgumentParser(
        description="Launch the CARLA Dijkstra navigation (A -> B).")

    # Connection
    p.add_argument("--host", default=nav.HOST,
                   help="CARLA server host (default: %(default)s)")
    p.add_argument("--port", type=int, default=nav.PORT,
                   help="CARLA server port (default: %(default)s)")

    # Start/destination by spawn-point index (default mode)
    p.add_argument("--a", type=int, default=nav.SPAWN_INDEX_A,
                   help="Start spawn-point index (Point A)")
    p.add_argument("--b", type=int, default=nav.SPAWN_INDEX_B,
                   help="Destination spawn-point index (Point B)")

    # Start/destination by explicit coordinates (switches off index mode)
    p.add_argument("--ax", type=float, help="Point A x coordinate")
    p.add_argument("--ay", type=float, help="Point A y coordinate")
    p.add_argument("--bx", type=float, help="Point B x coordinate")
    p.add_argument("--by", type=float, help="Point B y coordinate")

    # Behaviour
    p.add_argument("--speed", type=float, default=nav.TARGET_SPEED_KMH,
                   help="Target cruising speed in km/h")
    p.add_argument("--vehicle", default=nav.VEHICLE_BLUEPRINT,
                   help="Vehicle blueprint id")
    p.add_argument("--no-draw", action="store_true",
                   help="Do not draw the planned route in the world")
    return p.parse_args()


def apply_config(args):
    """Push the command-line options into the main module's config."""
    nav.HOST = args.host
    nav.PORT = args.port
    nav.TARGET_SPEED_KMH = args.speed
    nav.VEHICLE_BLUEPRINT = args.vehicle
    nav.DRAW_ROUTE = not args.no_draw

    # If any explicit coordinate is given, switch to coordinate mode.
    coords_given = any(v is not None for v in (args.ax, args.ay,
                                               args.bx, args.by))
    if coords_given:
        if None in (args.ax, args.ay, args.bx, args.by):
            sys.exit("ERROR: when using coordinates you must pass all of "
                     "--ax --ay --bx --by.")
        nav.USE_SPAWN_INDICES = False
        import carla
        nav.LOCATION_A = carla.Location(x=args.ax, y=args.ay, z=0.5)
        nav.LOCATION_B = carla.Location(x=args.bx, y=args.by, z=0.5)
    else:
        nav.USE_SPAWN_INDICES = True
        nav.SPAWN_INDEX_A = args.a
        nav.SPAWN_INDEX_B = args.b


def main():
    args = parse_args()
    apply_config(args)

    print("=" * 60)
    print(" Starting CARLA Dijkstra navigation")
    print("   server : {}:{}".format(nav.HOST, nav.PORT))
    if nav.USE_SPAWN_INDICES:
        print("   route  : spawn[{}] -> spawn[{}]".format(
            nav.SPAWN_INDEX_A, nav.SPAWN_INDEX_B))
    else:
        print("   route  : A({:.1f},{:.1f}) -> B({:.1f},{:.1f})".format(
            nav.LOCATION_A.x, nav.LOCATION_A.y,
            nav.LOCATION_B.x, nav.LOCATION_B.y))
    print("   speed  : {:.0f} km/h".format(nav.TARGET_SPEED_KMH))
    print("=" * 60)

    # Hand off to the main program.
    nav.main()


if __name__ == "__main__":
    main()