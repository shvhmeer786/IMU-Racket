import time
import math

# More robust test IMU with error handling and continuous output
print("Robust Test IMU v2.0")

counter = 0

while True:
    try:
        # Simple counter-based animation instead of time-based
        t = counter * 0.1
        
        # Generate smooth orientation changes  
        heading = (45 + math.sin(t * 0.5) * 90) % 360
        roll = math.sin(t * 0.7) * 30
        pitch = math.cos(t * 0.6) * 25
        
        # Simple accelerometer simulation
        ax = math.sin(t * 0.3) * 0.2
        ay = math.cos(t * 0.4) * 0.2
        az = 9.8 + math.sin(t * 0.2) * 0.3
        
        # Linear acceleration simulation
        lax = math.sin(t * 0.8) * 0.1
        lay = math.cos(t * 0.9) * 0.1
        laz = math.sin(t * 0.5) * 0.1
        
        # Output in BNO055 format
        print("Orientation: {:.2f}, {:.2f}, {:.2f}".format(heading, roll, pitch))
        print("Accel: {:.2f}, {:.2f}, {:.2f}".format(ax, ay, az))
        print("LinAccel: {:.2f}, {:.2f}, {:.2f}".format(lax, lay, laz))
        
        counter += 1
        time.sleep(0.1)
        
    except Exception as e:
        print("Error:", e)
        time.sleep(1)
        continue 