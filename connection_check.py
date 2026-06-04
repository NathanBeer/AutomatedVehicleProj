import sys
import glob
import os
import time

# 1. מציאה וטעינה אוטומטית של ספריית ה-CARLA מהתיקייה המקומית
try:
    # מחפש את קובץ ה-egg בתיקייה שבה הקוד נמצא
    egg_files = glob.glob(os.path.join(os.path.dirname(__file__), 'carla-*.egg'))
    if not egg_files:
        print("ERROR: Could not find carla .egg file in this folder!")
        print("Please copy the .egg file from PythonAPI/carla/dist to this folder.")
        sys.exit()
    sys.path.append(egg_files[0])
    print(f"Loaded library: {egg_files[0]}")
except Exception as e:
    print(f"Error loading egg file: {e}")
    sys.exit()

import carla

def main():
    print("Attempting to connect to CARLA server at 127.0.0.1:2000...")
    
    # 2. הגדרת לקוח עם Timeout משמעותי
    client = carla.Client('127.0.0.1', 2000)
    client.set_timeout(10.0) 

    try:
        # 3. ניסיון קבלת העולם
        world = client.get_world()
        map_name = world.get_map().name
        print("-" * 40)
        print(f"SUCCESS! Connected to CARLA.")
        print(f"Current Map: {map_name}")
        print("-" * 40)
        
        # הדפסת רשימת שחקנים (Actors) כאימות נוסף
        actors = world.get_actors()
        print(f"Number of actors in the scene: {len(actors)}")
        
    except RuntimeError as e:
        print("-" * 40)
        print("CRITICAL ERROR: Connection failed!")
        print("Possible reasons:")
        print("1. CarlaUE4.exe is not running.")
        print("2. Version mismatch (Client .egg vs Server version).")
        print("3. Firewall is blocking port 2000.")
        print(f"Technical error: {e}")
        print("-" * 40)

if __name__ == "__main__":
    main()