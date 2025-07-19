import time
import board
import busio
import adafruit_bno055
import os
import neopixel
import storage

print("Starting BNO055 test...")

# Setup NeoPixel for status indication
pixel = neopixel.NeoPixel(board.NEOPIXEL, 1)

# Simple UART setup
uart = busio.UART(tx=board.TX, rx=board.RX, baudrate=115200)
time.sleep(1)

print("UART created, trying to connect to sensor...")

try:
    sensor = adafruit_bno055.BNO055_UART(uart)
    print("BNO055 connected!")
    pixel.fill((0, 255, 0))  # Green: sensor connected
    time.sleep(1)
    pixel.fill((0, 0, 0))
except Exception as e:
    print(f"Failed to connect: {e}")
    pixel.fill((255, 0, 0))  # Red: sensor failed
    while True:
        time.sleep(1)

# Check if filesystem is writable by CircuitPython
def check_filesystem_writable():
    try:
        # Try to get filesystem info
        fs_stat = os.statvfs("/")
        # Try a small write test
        with open("test_write.tmp", "w") as f:
            f.write("test")
        os.remove("test_write.tmp")
        print("Filesystem is writable by CircuitPython")
        return True
    except OSError as e:
        if e.args[0] == 30:  # Read-only filesystem
            print("Filesystem is read-only! Press button and reset to enable logging.")
            return False
        elif e.args[0] == 28:  # Filesystem full
            print("Filesystem is full!")
            return False
        print(f"Filesystem error: {e}")
        return False

# Generate filename with timestamp
def generate_filename():
    # Get current time since boot (monotonic time)
    current_time = time.monotonic()
    # Convert to a simple identifier (seconds since boot)
    time_id = int(current_time)
    return f"data_{time_id}.csv"

# Create new CSV file with header
def create_new_csv_file(filename):
    try:
        with open(filename, "w") as f:
            f.write("timestamp,quat_w,quat_x,quat_y,quat_z,euler_heading,euler_roll,euler_pitch,gyro_x,gyro_y,gyro_z,accel_x,accel_y,accel_z,lin_accel_x,lin_accel_y,lin_accel_z\n")
        print(f"Created new CSV file: {filename}")
        return True
    except OSError as e:
        print(f"Failed to create CSV file: {e}")
        return False

# Manage old CSV files (keep only last 5 files to save space)
def cleanup_old_files():
    try:
        files = [f for f in os.listdir("/") if f.startswith("data_") and f.endswith(".csv")]
        if len(files) > 5:
            # Sort by filename (which contains timestamp)
            files.sort()
            # Remove oldest files
            for old_file in files[:-5]:
                try:
                    os.remove(old_file)
                    print(f"Removed old file: {old_file}")
                except:
                    pass
    except:
        pass

# Check if we can write to filesystem
can_write = check_filesystem_writable()

if not can_write:
    print("Cannot write to filesystem! Running in read-only mode.")

print("Starting data stream...")

# Initialize timing variables
session_start_time = time.monotonic()
session_duration = 20  # 20 seconds per file for easier testing
current_filename = None
write_error_count = 0
led_counter = 0
sample_count = 0
data_buffer = []
buffer_size = 50  # Write every 50 samples (0.5 seconds at 100Hz)

while True:
    try:
        current_time = time.monotonic()
        
        # Check if we need to start a new CSV file or take a break
        if can_write and (current_filename is None or (current_time - session_start_time) >= session_duration):
            # If we just finished a 20-second session, take a 10-second break
            if current_filename is not None:
                # Write any remaining buffered data before closing file
                if data_buffer and can_write:
                    try:
                        with open(current_filename, "a") as csv_file:
                            for line in data_buffer:
                                csv_file.write(line)
                            csv_file.flush()
                        data_buffer = []
                        print(f"Final buffer written to {current_filename}")
                    except OSError as e:
                        print(f"Final write error: {e}")
                
                print(f"\n=== CSV FILE SAVED: {current_filename} ===")
                print(f"=== Logged {sample_count} samples over 20 seconds ===")
                print("=== Taking 10-second break... ===")
                
                # 10-second break with countdown and status LED - NO sensor reading during this time
                for countdown in range(10, 0, -1):
                    pixel.fill((255, 255, 0))  # Yellow during break
                    time.sleep(0.5)
                    pixel.fill((0, 0, 0))
                    time.sleep(0.5)
                    # Complete break - no sensor reading or serial output
                
                print("=== Resuming data collection ===\n")
            
            # Clean up old files first
            cleanup_old_files()
            
            # Generate new filename and create file
            current_filename = generate_filename()
            if create_new_csv_file(current_filename):
                session_start_time = time.monotonic()  # Reset session start time after break
                sample_count = 0
                data_buffer = []  # Clear buffer for new session
                print(f"Starting new 20-second logging session: {current_filename}")
                # Purple flash to indicate new file
                pixel.fill((128, 0, 128))
                time.sleep(0.2)
                pixel.fill((0, 0, 0))
            else:
                can_write = False
                current_filename = None
        else:
            # Only read sensors and output if we're not in break mode
            # Read sensor data
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

                # Get timestamp (time since session start)
                session_time = current_time - session_start_time

                # Compact format for speed (console output) with elapsed seconds
                elapsed_seconds = session_time
                print(f"{elapsed_seconds:.1f}s|{w:.2f},{x:.2f},{y:.2f},{z:.2f}|{heading:.0f},{roll:.0f},{pitch:.0f}|{gx:.0f},{gy:.0f},{gz:.0f}|{ax:.0f},{ay:.0f},{az:.0f}|{lx:.1f},{ly:.1f},{lz:.1f}")

                # Write to CSV file if filesystem is writable
                if can_write and current_filename:
                    # Add data to buffer instead of writing immediately
                    csv_line = f"{session_time:.2f},{w:.2f},{x:.2f},{y:.2f},{z:.2f},{heading:.0f},{roll:.0f},{pitch:.0f},{gx:.0f},{gy:.0f},{gz:.0f},{ax:.0f},{ay:.0f},{az:.0f},{lx:.1f},{ly:.1f},{lz:.1f}\n"
                    data_buffer.append(csv_line)
                    sample_count += 1
                    
                    # Write buffer to file when it's full
                    if len(data_buffer) >= buffer_size:
                        try:
                            with open(current_filename, "a") as csv_file:
                                for line in data_buffer:
                                    csv_file.write(line)
                                csv_file.flush()
                            data_buffer = []  # Clear buffer after successful write
                            write_error_count = 0
                            
                            # Blue flash every buffer write to indicate logging
                            pixel.fill((0, 0, 255))
                            time.sleep(0.01)
                            pixel.fill((0, 0, 0))
                            print(f"Logged {sample_count} samples to {current_filename}")
                            
                        except OSError as e:
                            write_error_count += 1
                            print(f"CSV write error: {e}")
                            if e.args[0] == 28:  # Filesystem full
                                print("Filesystem full! Stopping CSV logging.")
                                can_write = False
                                current_filename = None
                                data_buffer = []  # Clear buffer
                                # Very fast red blink for filesystem full
                                for _ in range(3):
                                    pixel.fill((255, 0, 0))
                                    time.sleep(0.05)
                                    pixel.fill((0, 0, 0))
                                    time.sleep(0.05)
            else:
                elapsed_seconds = current_time - session_start_time if can_write and current_filename else 0
                print(f"{elapsed_seconds:.1f}s|0,0,0,0|0,0,0|0,0,0|0,0,0|0,0,0")
                # Write zeros to CSV as well if filesystem is writable
                if can_write and current_filename:
                    csv_line = f"{session_time:.2f},0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n"
                    data_buffer.append(csv_line)
                    sample_count += 1
                    
                    # Write buffer to file when it's full
                    if len(data_buffer) >= buffer_size:
                        try:
                            with open(current_filename, "a") as csv_file:
                                for line in data_buffer:
                                    csv_file.write(line)
                                csv_file.flush()
                            data_buffer = []  # Clear buffer after successful write
                            write_error_count = 0
                        except OSError as e:
                            write_error_count += 1
                            print(f"CSV write error: {e}")
                    
    except Exception as e:
        print(f"Read error: {e}")

    # LED status indication for read-only filesystem (only if not actively logging)
    if not can_write:
        if led_counter % 50 == 0:  # Slow blink every 0.5 seconds
            pixel.fill((255, 100, 0))  # Orange: read-only mode
            time.sleep(0.1)
            pixel.fill((0, 0, 0))

    led_counter += 1
    
    # Only sleep if we're actively logging (not during break periods)
    if can_write and current_filename and (time.monotonic() - session_start_time) < session_duration:
        time.sleep(0.01)  # 100Hz