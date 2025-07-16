import time
import math

# Simple test IMU that generates fake but realistic orientation data
# This replaces the BNO055 code for testing purposes

print("Simple Test IMU - Generating fake orientation data")
print("Upload this to your Arduino as code.py to test the visualization")

start_time = time.monotonic()

while True:
    current_time = time.monotonic() - start_time
    
    # Generate smooth, realistic orientation changes
    heading = (50 + math.sin(current_time * 0.2) * 180) % 360
    roll = math.sin(current_time * 0.3) * 45  
    pitch = math.cos(current_time * 0.25) * 30
    
    # Generate fake accelerometer data
    ax = -0.1 + math.sin(current_time * 0.1) * 0.1
    ay = -0.2 + math.cos(current_time * 0.15) * 0.1  
    az = 9.5 + math.sin(current_time * 0.05) * 0.3
    
    # Generate fake linear acceleration
    lax = math.sin(current_time * 0.4) * 0.05
    lay = math.cos(current_time * 0.35) * 0.05
    laz = math.sin(current_time * 0.3) * 0.05
    
    # Print in the same format as the real BNO055
    print(f"Orientation: {heading:.2f}, {roll:.2f}, {pitch:.2f}")
    print(f"Accel: {ax:.2f}, {ay:.2f}, {az:.2f}")
    print(f"LinAccel: {lax:.2f}, {lay:.2f}, {laz:.2f}")
    
    time.sleep(0.1)  # 10 Hz like the real sensor 