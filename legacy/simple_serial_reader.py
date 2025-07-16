#!/usr/bin/env python3
"""
Simple Serial Reader for Stable C++ Arduino Sketch

This version removes all the complex reconnection logic and just reads 
the clean data from the working C++ sketch.
"""

import serial
import time
import re
import csv
import os
from datetime import datetime
from threading import Thread, Event
import queue

class SimpleSerialReader:
    def __init__(self, port='/dev/cu.usbmodem1101', baud=115200):
        self.port = port
        self.baud = baud
        self.serial_conn = None
        self.data_queue = queue.Queue()
        self.stop_event = Event()
        self.csv_file = None
        self.csv_writer = None
        
        # Create data directory if it doesn't exist
        os.makedirs('data', exist_ok=True)
        
    def connect(self):
        """Simple connection without excessive resets"""
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
                
            print(f"Connecting to {self.port} at {self.baud} baud...")
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=2.0,
                write_timeout=2.0
            )
            
            # Small delay for connection to stabilize
            time.sleep(1.0)
            
            # Clear any existing data in buffer
            self.serial_conn.reset_input_buffer()
            
            print("✅ Connected successfully!")
            return True
            
        except Exception as e:
            print(f"❌ Connection failed: {e}")
            return False
    
    def setup_csv(self):
        """Setup CSV logging"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_filename = f"data/imu_data_{timestamp}.csv"
        
        self.csv_file = open(csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'heading', 'roll', 'pitch', 'accel_x', 'accel_y', 'accel_z'])
        
        print(f"📝 Logging to: {csv_filename}")
    
    def parse_orientation(self, line):
        """Parse orientation line: 'Orientation: heading, roll, pitch'"""
        match = re.search(r'Orientation:\s*([-+]?\d*\.?\d+),\s*([-+]?\d*\.?\d+),\s*([-+]?\d*\.?\d+)', line)
        if match:
            heading = float(match.group(1))
            roll = float(match.group(2))
            pitch = float(match.group(3))
            return heading, roll, pitch
        return None
    
    def parse_accel(self, line):
        """Parse acceleration line: 'Accel: x, y, z'"""
        match = re.search(r'Accel:\s*([-+]?\d*\.?\d+),\s*([-+]?\d*\.?\d+),\s*([-+]?\d*\.?\d+)', line)
        if match:
            x = float(match.group(1))
            y = float(match.group(2))
            z = float(match.group(3))
            return x, y, z
        return None
    
    def read_loop(self):
        """Main reading loop - simplified for stable C++ data"""
        current_orientation = None
        current_accel = None
        
        print("📡 Starting to read data...")
        
        while not self.stop_event.is_set():
            try:
                if not self.serial_conn or not self.serial_conn.is_open:
                    if not self.connect():
                        time.sleep(1.0)
                        continue
                
                # Read line with timeout
                try:
                    raw_line = self.serial_conn.readline()
                    if not raw_line:
                        continue
                        
                    # Decode with error handling
                    line = raw_line.decode('utf-8', errors='ignore').strip()
                    if not line:
                        continue
                        
                    print(f"📥 {line}")
                    
                    # Parse orientation data
                    orientation = self.parse_orientation(line)
                    if orientation:
                        current_orientation = orientation
                        print(f"🧭 Orientation: heading={orientation[0]:.2f}, roll={orientation[1]:.2f}, pitch={orientation[2]:.2f}")
                    
                    # Parse acceleration data
                    accel = self.parse_accel(line)
                    if accel:
                        current_accel = accel
                    
                    # If we have both orientation and accel, send complete frame
                    if current_orientation and current_accel:
                        timestamp = int(time.time() * 1000)
                        
                        frame = {
                            'timestamp': timestamp,
                            'heading': current_orientation[0],
                            'roll': current_orientation[1], 
                            'pitch': current_orientation[2],
                            'accel_x': current_accel[0],
                            'accel_y': current_accel[1],
                            'accel_z': current_accel[2]
                        }
                        
                        # Add to queue for visualization
                        try:
                            self.data_queue.put_nowait(frame)
                        except queue.Full:
                            pass  # Skip if queue is full
                        
                        # Log to CSV
                        if self.csv_writer:
                            self.csv_writer.writerow([
                                timestamp,
                                current_orientation[0],
                                current_orientation[1], 
                                current_orientation[2],
                                current_accel[0],
                                current_accel[1],
                                current_accel[2]
                            ])
                            self.csv_file.flush()
                        
                        # Reset for next frame
                        current_orientation = None
                        current_accel = None
                        
                except serial.SerialTimeoutException:
                    continue
                except Exception as e:
                    print(f"⚠️  Read error: {e}")
                    time.sleep(0.1)
                    
            except Exception as e:
                print(f"❌ Loop error: {e}")
                time.sleep(1.0)
    
    def start(self):
        """Start reading in background thread"""
        if not self.connect():
            return False
            
        self.setup_csv()
        
        self.read_thread = Thread(target=self.read_loop, daemon=True)
        self.read_thread.start()
        return True
    
    def stop(self):
        """Stop reading"""
        print("\n🛑 Stopping...")
        self.stop_event.set()
        
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            
        if self.csv_file:
            self.csv_file.close()
            
        print("✅ Stopped")
    
    def get_data(self):
        """Get latest data frame"""
        try:
            return self.data_queue.get_nowait()
        except queue.Empty:
            return None

def main():
    """Test the simple serial reader"""
    reader = SimpleSerialReader()
    
    try:
        if reader.start():
            print("🚀 Reading data from stable C++ sketch...")
            print("Press Ctrl+C to stop")
            
            while True:
                data = reader.get_data()
                if data:
                    print(f"🔄 Frame: heading={data['heading']:.1f}°, roll={data['roll']:.1f}°, pitch={data['pitch']:.1f}°")
                time.sleep(0.1)
                
    except KeyboardInterrupt:
        pass
    finally:
        reader.stop()

if __name__ == "__main__":
    main() 