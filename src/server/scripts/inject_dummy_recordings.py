#!/usr/bin/env python3
import sys
import os
import random
from datetime import datetime, timedelta
from server_pkg.server_db import init_db, save_recording, save_estimation_data

def create_dummy_recording(name, description, n_points=100):
    # Create dummy estimation data
    start_time = datetime.now().timestamp()
    estimations = []
    for i in range(n_points):
        t = start_time + i * 0.1
        estimations.append({
            "timestamp": t,
            "x": random.uniform(-5, 5),
            "y": random.uniform(-5, 5),
            "z": random.uniform(-1, 1),
            "yaw": random.uniform(-3.14, 3.14),
            "pitch": random.uniform(-1, 1),
            "roll": random.uniform(-1, 1),
            "acc_x": random.uniform(-2, 2),
            "acc_y": random.uniform(-2, 2),
            "acc_z": random.uniform(-2, 2),
            "acc_yaw": random.uniform(-0.5, 0.5),
            "acc_pitch": random.uniform(-0.5, 0.5),
            "acc_roll": random.uniform(-0.5, 0.5),
            "mag_x": random.uniform(-50, 50),
            "mag_y": random.uniform(-50, 50),
            "mag_z": random.uniform(-50, 50),
            "mag_strength": random.uniform(0, 100),
            "mouse_integrated_x": random.uniform(-10, 10),
            "mouse_integrated_y": random.uniform(-10, 10)
        })
    # Save recording metadata
    result = save_recording(name, {}, description)
    if result["success"]:
        rec_id = result["id"]
        save_estimation_data(rec_id, estimations)
        print(f"Inserted dummy recording '{name}' with {n_points} points (id={rec_id})")
    else:
        print(f"Failed to insert recording: {result}")

def main():
    init_db()
    for i in range(3):
        create_dummy_recording(f"Dummy Recording {i+1}", f"This is test recording #{i+1}", n_points=100)

if __name__ == "__main__":
    main()
