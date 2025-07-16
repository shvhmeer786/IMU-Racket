#!/usr/bin/env python3
"""
Fake IMU data generator for testing the visualization pipeline
without needing working hardware.
"""

import time
import math
import random

def generate_fake_data():
    """Generate realistic fake IMU orientation data."""
    start_time = time.time()
    
    while True:
        current_time = time.time() - start_time
        
        # Generate smooth, realistic orientation changes
        heading = (math.sin(current_time * 0.1) * 180) % 360
        roll = math.sin(current_time * 0.15) * 45
        pitch = math.cos(current_time * 0.12) * 30
        
        # Add some realistic noise
        heading += random.uniform(-0.5, 0.5)
        roll += random.uniform(-0.3, 0.3)
        pitch += random.uniform(-0.3, 0.3)
        
        # Generate fake accelerometer data
        ax = random.uniform(-0.2, 0.2)
        ay = random.uniform(-0.2, 0.2)
        az = random.uniform(9.5, 9.9)  # ~gravity
        
        # Generate fake linear acceleration
        lax = random.uniform(-0.1, 0.1)
        lay = random.uniform(-0.1, 0.1)
        laz = random.uniform(-0.1, 0.1)
        
        # Print in the same format as the real BNO055
        print(f"Orientation: {heading:.2f}, {roll:.2f}, {pitch:.2f}")
        print(f"Accel: {ax:.2f}, {ay:.2f}, {az:.2f}")
        print(f"LinAccel: {lax:.2f}, {lay:.2f}, {laz:.2f}")
        
        time.sleep(0.1)  # 10 Hz like the real sensor

if __name__ == "__main__":
    print("Fake BNO055 data generator")
    print("Generating realistic IMU data for testing...")
    print("Press Ctrl+C to stop")
    
    try:
        generate_fake_data()
    except KeyboardInterrupt:
        print("\nStopped fake data generation.") 