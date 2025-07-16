import time
import board
import busio
import adafruit_bno055

# ── I2C connection to BNO055 (more reliable than UART) ──
i2c = busio.I2C(board.SCL, board.SDA)

time.sleep(1)  # let sensor boot

try:
    sensor = adafruit_bno055.BNO055_I2C(i2c)
    print("✅ BNO055 connected via I2C!")
    print("Sensor ID:", sensor.sensor_id)
    print("Software version:", sensor.sw_version)
    print("Bootloader version:", sensor.bootloader_version)
    print("")
except Exception as e:
    print("❌ BNO055 I2C connection failed:", e)
    print("Check wiring:")
    print("  BNO055 VIN → Arduino 3.3V")
    print("  BNO055 GND → Arduino GND") 
    print("  BNO055 SDA → Arduino SDA")
    print("  BNO055 SCL → Arduino SCL")
    sensor = None

# Test basic functionality
if sensor:
    print("Testing basic sensor readings...")
    for i in range(10):
        try:
            # Test temperature (should always work)
            temp = sensor.temperature
            print(f"Temperature: {temp}°C")
            
            # Test orientation
            euler = sensor.euler
            if euler:
                print(f"Orientation: {euler[0]:.1f}, {euler[1]:.1f}, {euler[2]:.1f}")
            else:
                print("Orientation: Not ready")
                
            time.sleep(1)
            
        except Exception as e:
            print(f"Reading error: {e}")
            time.sleep(1)
else:
    print("Cannot test - sensor not connected")
    time.sleep(10)  # Keep running to see error message 