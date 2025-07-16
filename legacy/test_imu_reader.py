#!/usr/bin/env python3
"""
Test script for IMUReader with specific port and baud settings.
"""

from serial_reader import IMUReader
import time

# Your specific hardware settings
port = "/dev/cu.usbmodem1101"
baud = 115200

def main():
    print(f"Testing IMU Reader with:")
    print(f"  Port: {port}")
    print(f"  Baud: {baud}")
    print("  CSV rotation: every 10 minutes")
    print("\nPress Ctrl+C to stop\n")
    
    try:
        with IMUReader(port, baud, chunk_minutes=10) as reader:
            frame_count = 0
            start_time = time.time()
            
            while True:
                frame = reader.get_frame(timeout=1.0)
                if frame:
                    frame_count += 1
                    
                    # Print every 10th frame to avoid spam
                    if frame_count % 10 == 0:
                        quat = frame['quat']
                        acc = frame['acc']
                        mag = frame['mag']
                        
                        print(f"Frame #{frame_count}:")
                        
                        # Show Euler angles if available (from Arduino format)
                        if 'euler' in frame:
                            euler = frame['euler']
                            print(f"  Orientation: yaw={euler[0]:.2f}°, pitch={euler[1]:.2f}°, roll={euler[2]:.2f}°")
                        
                        print(f"  Quaternion: w={quat[0]:.3f}, x={quat[1]:.3f}, y={quat[2]:.3f}, z={quat[3]:.3f}")
                        print(f"  Accel:      x={acc[0]:.3f}, y={acc[1]:.3f}, z={acc[2]:.3f}")
                        print(f"  Mag:        x={mag[0]:.3f}, y={mag[1]:.3f}, z={mag[2]:.3f}")
                        print(f"  Timestamp:  {frame['ts']}")
                        print(f"  Queue size: {reader.queue_size}")
                        print(f"  Connected:  {reader.is_connected}")
                        print("-" * 50)
                        
                else:
                    # No data received
                    elapsed = time.time() - start_time
                    if elapsed > 5:  # Show status every 5 seconds if no data
                        print(f"No data received... (Connected: {reader.is_connected}, Queue: {reader.queue_size})")
                        start_time = time.time()
                    
    except KeyboardInterrupt:
        print("\nStopping IMU reader...")
    except Exception as e:
        print(f"Error: {e}")
        print("\nTroubleshooting tips:")
        print("1. Check if the USB device is connected")
        print("2. Verify the port name with: ls /dev/cu.*")
        print("3. Make sure no other program is using the serial port")
        print("4. Check if your IMU is sending data in the expected format")

if __name__ == "__main__":
    main() 