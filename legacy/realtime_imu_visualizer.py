#!/usr/bin/env python3
"""
Real-Time IMU Visualizer with Connection Resilience
Handles Arduino connection instability while maintaining smooth visualization
"""

import serial
import time
import threading
import queue
import re
import argparse
from collections import deque
import numpy as np
from vpython import *

class ResilientIMUReader:
    def __init__(self, port, baud=115200, buffer_size=10):
        self.port = port
        self.baud = baud
        self.buffer_size = buffer_size
        
        # Data management
        self.data_queue = queue.Queue(maxsize=100)
        self.orientation_buffer = deque(maxlen=buffer_size)
        self.last_good_data = {'heading': 0.0, 'roll': 0.0, 'pitch': 0.0}
        
        # Connection management
        self.serial_conn = None
        self.running = False
        self.read_thread = None
        
        # Statistics
        self.frames_received = 0
        self.connection_attempts = 0
        self.last_data_time = time.time()
        
    def connect(self):
        """Quick connection attempt - don't block for long"""
        try:
            if self.serial_conn and self.serial_conn.is_open:
                return True
                
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=0.1,  # Very short timeout
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            self.connection_attempts += 1
            return True
            
        except Exception as e:
            if self.serial_conn:
                try:
                    self.serial_conn.close()
                except:
                    pass
            self.serial_conn = None
            return False
    
    def read_data(self):
        """Read available data without blocking"""
        if not self.serial_conn or not self.serial_conn.is_open:
            return None
            
        try:
            # Read whatever is available
            if self.serial_conn.in_waiting > 0:
                line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    return line
        except Exception:
            # Connection failed, close it
            try:
                self.serial_conn.close()
            except:
                pass
            self.serial_conn = None
            
        return None
    
    def parse_orientation(self, line):
        """Parse orientation data from line"""
        match = re.match(r'Orientation:\s*([+-]?\d*\.?\d+),\s*([+-]?\d*\.?\d+),\s*([+-]?\d*\.?\d+)', line)
        if match:
            heading = float(match.group(1))
            roll = float(match.group(2))
            pitch = float(match.group(3))
            return {'heading': heading, 'roll': roll, 'pitch': pitch}
        return None
    
    def data_reader_thread(self):
        """Background thread for reading data"""
        reconnect_delay = 0.0
        
        while self.running:
            # Try to connect if needed
            if not self.serial_conn or not self.serial_conn.is_open:
                if time.time() >= reconnect_delay:
                    if self.connect():
                        print(f"✅ Connected (attempt {self.connection_attempts})")
                        reconnect_delay = 0.0
                    else:
                        reconnect_delay = time.time() + 0.5  # Wait 0.5s before retry
                        
            # Read data
            line = self.read_data()
            if line:
                orientation = self.parse_orientation(line)
                if orientation:
                    self.frames_received += 1
                    self.last_data_time = time.time()
                    self.last_good_data = orientation
                    self.orientation_buffer.append(orientation)
                    
                    # Put in queue for visualization
                    try:
                        self.data_queue.put_nowait(orientation)
                    except queue.Full:
                        pass  # Skip if queue is full
                        
            # Small delay to prevent CPU spinning
            time.sleep(0.01)
    
    def get_latest_orientation(self):
        """Get the most recent orientation data"""
        # Try to get new data from queue
        try:
            while True:
                self.last_good_data = self.data_queue.get_nowait()
        except queue.Empty:
            pass
            
        # If no recent data, use interpolated/predicted data
        if time.time() - self.last_data_time > 0.5:
            print(f"⚠️  No data for {time.time() - self.last_data_time:.1f}s - using last known")
            
        return self.last_good_data
    
    def start(self):
        """Start the data reading thread"""
        self.running = True
        self.read_thread = threading.Thread(target=self.data_reader_thread, daemon=True)
        self.read_thread.start()
        print(f"🚀 Real-time IMU reader started on {self.port}")
    
    def stop(self):
        """Stop the data reading thread"""
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=1.0)
        if self.serial_conn:
            try:
                self.serial_conn.close()
            except:
                pass
    
    def get_stats(self):
        """Get connection statistics"""
        return {
            'frames_received': self.frames_received,
            'connection_attempts': self.connection_attempts,
            'buffer_size': len(self.orientation_buffer),
            'data_age': time.time() - self.last_data_time
        }

class RealTimeVisualizer:
    def __init__(self, imu_reader):
        self.imu_reader = imu_reader
        
        # VPython scene setup
        scene.title = "Real-Time IMU Motion Tracking"
        scene.width = 800
        scene.height = 600
        scene.background = color.black
        scene.forward = vector(0, 0, -1)
        scene.up = vector(0, 1, 0)
        
        # Create 3D arrow
        self.arrow = arrow(
            pos=vector(0, 0, 0),
            axis=vector(0, 0, 1),
            shaftwidth=0.1,
            headwidth=0.2,
            headlength=0.3,
            color=color.red
        )
        
        # Create coordinate system
        self.create_coordinate_system()
        
        # Statistics display
        self.stats_label = label(
            pos=vector(0, -2, 0),
            text="Starting...",
            color=color.white,
            height=16
        )
        
        # Timing
        self.last_update = time.time()
        self.frame_count = 0
        
    def create_coordinate_system(self):
        """Create reference coordinate system"""
        # X-axis (red)
        arrow(pos=vector(0, 0, 0), axis=vector(1, 0, 0), color=color.red, shaftwidth=0.02)
        label(pos=vector(1.2, 0, 0), text="X", color=color.red, height=12)
        
        # Y-axis (green)
        arrow(pos=vector(0, 0, 0), axis=vector(0, 1, 0), color=color.green, shaftwidth=0.02)
        label(pos=vector(0, 1.2, 0), text="Y", color=color.green, height=12)
        
        # Z-axis (blue)
        arrow(pos=vector(0, 0, 0), axis=vector(0, 0, 1), color=color.blue, shaftwidth=0.02)
        label(pos=vector(0, 0, 1.2), text="Z", color=color.blue, height=12)
        
        # Origin
        sphere(pos=vector(0, 0, 0), radius=0.05, color=color.white)
    
    def update_visualization(self):
        """Update the 3D visualization"""
        # Get latest orientation
        orientation = self.imu_reader.get_latest_orientation()
        
        # Convert to radians
        heading_rad = np.radians(orientation['heading'])
        roll_rad = np.radians(orientation['roll'])
        pitch_rad = np.radians(orientation['pitch'])
        
        # Calculate 3D orientation vector
        # Using aerospace convention: heading (yaw), roll, pitch
        x = np.cos(pitch_rad) * np.cos(heading_rad)
        y = np.cos(pitch_rad) * np.sin(heading_rad)
        z = np.sin(pitch_rad)
        
        # Update arrow
        self.arrow.axis = vector(x, y, z)
        
        # Update color based on movement
        speed = np.sqrt(x*x + y*y + z*z)
        if speed > 0.8:
            self.arrow.color = color.red
        elif speed > 0.5:
            self.arrow.color = color.orange
        else:
            self.arrow.color = color.green
        
        # Update statistics
        self.frame_count += 1
        current_time = time.time()
        if current_time - self.last_update >= 1.0:  # Update stats every second
            fps = self.frame_count / (current_time - self.last_update)
            stats = self.imu_reader.get_stats()
            
            self.stats_label.text = f"""FPS: {fps:.1f} | Frames: {stats['frames_received']} | Connections: {stats['connection_attempts']}
H: {orientation['heading']:7.2f}° | R: {orientation['roll']:7.2f}° | P: {orientation['pitch']:7.2f}°
Data age: {stats['data_age']:.1f}s | Buffer: {stats['buffer_size']}"""
            
            self.frame_count = 0
            self.last_update = current_time
    
    def run(self):
        """Main visualization loop"""
        print("🎯 Real-time visualization started")
        print("📱 Move your Arduino to see live orientation tracking")
        print("⚠️  If connection is unstable, visualization continues with last known data")
        print("Press Ctrl+C to stop")
        
        try:
            while True:
                rate(60)  # Target 60 FPS
                self.update_visualization()
                
        except KeyboardInterrupt:
            print("\n🛑 Stopping visualization...")
            self.imu_reader.stop()

def main():
    parser = argparse.ArgumentParser(description='Real-Time IMU Visualizer with Connection Resilience')
    parser.add_argument('--port', default='/dev/cu.usbmodem1101', help='Serial port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    args = parser.parse_args()
    
    print("🚀 Real-Time IMU Motion Tracking System")
    print("=" * 50)
    print(f"Port: {args.port}")
    print(f"Baud: {args.baud}")
    print("Features: Connection resilience, smooth visualization, real-time tracking")
    print("=" * 50)
    
    # Create resilient IMU reader
    imu_reader = ResilientIMUReader(args.port, args.baud)
    imu_reader.start()
    
    # Create and run visualizer
    visualizer = RealTimeVisualizer(imu_reader)
    visualizer.run()

if __name__ == "__main__":
    main() 