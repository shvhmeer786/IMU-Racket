import time
import board
import busio
import adafruit_bno055

print("Starting BNO055 test...")

# Simple UART setup
uart = busio.UART(tx=board.TX, rx=board.RX, baudrate=115200)
time.sleep(1)

print("UART created, trying to connect to sensor...")

try:
    sensor = adafruit_bno055.BNO055_UART(uart)
    print("BNO055 connected!")
except Exception as e:
    print(f"Failed to connect: {e}")
    while True:
        time.sleep(1)

print("Starting data stream...")

while True:
    try:
        # Simple read - just quaternion
        quat = sensor.quaternion
        euler = sensor.euler
        gyro = sensor.gyro
        accel = sensor.acceleration
        lin_accel = sensor.linear_acceleration
        
        if quat and euler and gyro and accel and lin_accel:
            w, x, y, z = quat
            heading, roll, pitch = euler
            gx, gy, gz = gyro
            ax, ay, az = accel
            lx, ly, lz = lin_accel
            # Compact format for speed
            print(f"{w:.2f},{x:.2f},{y:.2f},{z:.2f}|{heading:.0f},{roll:.0f},{pitch:.0f}|{gx:.0f},{gy:.0f},{gz:.0f}|{ax:.0f},{ay:.0f},{az:.0f}|{lx:.1f},{ly:.1f},{lz:.1f}")
        else:
            print("0,0,0,0|0,0,0|0,0,0|0,0,0|0,0,0")
    except Exception as e:
        print(f"Read error: {e}")
    
    time.sleep(0.01)  # Back to 100Hz