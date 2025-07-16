#!/usr/bin/env python3
"""
Debug script to see raw serial data from Arduino.
This helps determine what format the IMU is sending data in.
"""

import serial
import time
import argparse

def main():
    parser = argparse.ArgumentParser(description="Debug raw serial data from IMU")
    parser.add_argument('--port', default='/dev/cu.usbmodem1101', help='Serial port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    args = parser.parse_args()
    
    print(f"Debugging serial data from {args.port} at {args.baud} baud")
    print("This will show the raw data your Arduino is sending...")
    print("Press Ctrl+C to stop\n")
    
    try:
        with serial.Serial(args.port, args.baud, timeout=1) as ser:
            print("Connected! Listening for data...\n")
            
            line_count = 0
            start_time = time.time()
            
            while True:
                if ser.in_waiting > 0:
                    try:
                        # Read raw bytes
                        raw_data = ser.readline()
                        line_count += 1
                        
                        # Try to decode as text
                        try:
                            text_data = raw_data.decode('utf-8').strip()
                            print(f"Line {line_count}: '{text_data}'")
                        except UnicodeDecodeError:
                            print(f"Line {line_count}: Raw bytes: {raw_data}")
                            
                    except Exception as e:
                        print(f"Read error: {e}")
                        
                else:
                    # Check for timeout
                    if time.time() - start_time > 5:
                        print("No data received for 5 seconds...")
                        print("Your Arduino might not be programmed to send IMU data yet.")
                        start_time = time.time()
                    
                    time.sleep(0.1)
                    
    except KeyboardInterrupt:
        print("\nStopped debugging.")
    except Exception as e:
        print(f"Error: {e}")
        print("\nMake sure:")
        print("1. Arduino is connected")
        print("2. Arduino is programmed with IMU code")
        print("3. No other program is using the serial port")

if __name__ == "__main__":
    main() 