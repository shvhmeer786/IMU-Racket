#!/usr/bin/env python3
"""
Test version of live arrow visualization that works with fake data.
This helps isolate hardware vs software issues.
"""

import argparse
import time
import subprocess
import threading
import queue
import numpy as np
from scipy.spatial.transform import Rotation
import vpython as vp

class TestOrientationVisualizer:
    """Test orientation visualizer that works with fake data."""
    
    def __init__(self, length: float = 0.7):
        """Initialize the test visualizer."""
        self.length = length
        
        # Frame rate control
        self.target_fps = 60
        self.frame_time = 1.0 / self.target_fps
        self.last_update = 0
        
        # Data queue for fake data
        self.data_queue = queue.Queue()
        
        # Simple trail management
        self.trail_points = []
        self.max_trail_points = 500
        
        # VPython scene setup
        self.scene = vp.canvas(
            title="TEST: IMU Orientation (Fake Data)",
            width=800,
            height=600,
            background=vp.color.black,
            center=vp.vector(0, 0, 0),
            range=length * 1.5
        )
        
        # Create arrow
        self.arrow = vp.arrow(
            pos=vp.vector(0, 0, 0),
            axis=vp.vector(0, 0, length),
            shaftwidth=0.02,
            headwidth=0.04,
            headlength=0.06,
            color=vp.color.red
        )
        
        # Create trail
        self.trail = vp.curve(
            color=vp.color.yellow,
            radius=0.005
        )
        
        # Create coordinate system
        self._create_coordinate_system()
        
        # Add info text
        self.info_text = vp.wtext(text="Starting fake data test...\n")
        
        # Data tracking
        self.frame_count = 0
        self.start_time = time.time()
        
        print("Test visualizer initialized")
        print("Using fake data to test visualization pipeline")
    
    def _create_coordinate_system(self):
        """Create reference coordinate system axes."""
        axis_length = self.length * 0.3
        
        # X-axis (red)
        vp.arrow(pos=vp.vector(0, 0, 0), axis=vp.vector(axis_length, 0, 0),
                shaftwidth=0.005, color=vp.color.red, opacity=0.5)
        vp.label(pos=vp.vector(axis_length * 1.1, 0, 0), text="X", color=vp.color.red, height=12)
        
        # Y-axis (green)
        vp.arrow(pos=vp.vector(0, 0, 0), axis=vp.vector(0, axis_length, 0),
                shaftwidth=0.005, color=vp.color.green, opacity=0.5)
        vp.label(pos=vp.vector(0, axis_length * 1.1, 0), text="Y", color=vp.color.green, height=12)
        
        # Z-axis (blue)
        vp.arrow(pos=vp.vector(0, 0, 0), axis=vp.vector(0, 0, axis_length),
                shaftwidth=0.005, color=vp.color.blue, opacity=0.5)
        vp.label(pos=vp.vector(0, 0, axis_length * 1.1), text="Z", color=vp.color.blue, height=12)
    
    def euler_to_quaternion(self, yaw: float, pitch: float, roll: float):
        """Convert Euler angles to quaternion."""
        yaw_rad = np.radians(yaw)
        pitch_rad = np.radians(pitch)
        roll_rad = np.radians(roll)
        
        rotation = Rotation.from_euler('ZYX', [yaw_rad, pitch_rad, roll_rad])
        quat = rotation.as_quat()  # Returns [x, y, z, w]
        
        return (quat[3], quat[0], quat[1], quat[2])  # Convert to [w, x, y, z]
    
    def quaternion_to_rotation_matrix(self, quat: tuple):
        """Convert quaternion to rotation matrix."""
        w, x, y, z = quat
        
        if abs(w) < 1e-6 and abs(x) < 1e-6 and abs(y) < 1e-6 and abs(z) < 1e-6:
            return np.eye(3)
        
        rotation = Rotation.from_quat([x, y, z, w])
        return rotation.as_matrix()
    
    def parse_fake_line(self, line: str):
        """Parse fake data line into frame format."""
        line = line.strip()
        if line.startswith('Orientation:'):
            try:
                values_str = line[12:].strip()
                parts = [float(x.strip()) for x in values_str.split(',')]
                if len(parts) >= 3:
                    heading, roll, pitch = parts[0], parts[1], parts[2]
                    frame = {
                        'quat': (1.0, 0.0, 0.0, 0.0),  # Placeholder
                        'acc': (0.0, 0.0, 9.81),
                        'mag': (0.0, 0.0, 0.0),
                        'euler': (heading, roll, pitch),
                        'ts': int(time.time() * 1000)
                    }
                    return frame
            except ValueError:
                pass
        return None
    
    def start_fake_data_source(self):
        """Start the fake data generator subprocess."""
        def read_fake_data():
            try:
                process = subprocess.Popen(
                    ['python', 'fake_imu_test.py'],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    bufsize=1
                )
                
                while True:
                    line = process.stdout.readline()
                    if not line:
                        break
                    
                    frame = self.parse_fake_line(line)
                    if frame:
                        try:
                            self.data_queue.put_nowait(frame)
                        except queue.Full:
                            try:
                                self.data_queue.get_nowait()
                                self.data_queue.put_nowait(frame)
                            except queue.Empty:
                                pass
                        
            except Exception as e:
                print(f"Error reading fake data: {e}")
        
        # Start in background thread
        thread = threading.Thread(target=read_fake_data, daemon=True)
        thread.start()
        return thread
    
    def update_arrow_from_frame(self, frame: dict):
        """Update arrow orientation from frame data."""
        try:
            quat = frame['quat']
            
            if 'euler' in frame:
                euler = frame['euler']
                heading, roll, pitch = euler[0], euler[1], euler[2]
                quat = self.euler_to_quaternion(heading, roll, pitch)
            
            rotation_matrix = self.quaternion_to_rotation_matrix(quat)
            arrow_direction = np.array([0, 0, self.length])
            rotated_direction = rotation_matrix @ arrow_direction
            
            try:
                self.arrow.pos = vp.vector(0, 0, 0)
                self.arrow.axis = vp.vector(rotated_direction[0], rotated_direction[1], rotated_direction[2])
                
                tip_pos = vp.vector(rotated_direction[0], rotated_direction[1], rotated_direction[2])
                self.trail_points.append(tip_pos)
                
                if len(self.trail_points) > self.max_trail_points:
                    self.trail_points = self.trail_points[-self.max_trail_points:]
                
                self.trail.clear()
                for point in self.trail_points:
                    self.trail.append(pos=point)
                
                self.frame_count += 1
                elapsed_time = time.time() - self.start_time
                fps = self.frame_count / elapsed_time if elapsed_time > 0 else 0
                
                info = f"🧪 TEST MODE - FAKE DATA\n"
                info += f"Frame: {self.frame_count}\n"
                info += f"FPS: {fps:.1f}\n"
                info += f"Trail points: {len(self.trail_points)}\n\n"
                
                if 'euler' in frame:
                    euler = frame['euler']
                    info += f"Heading: {euler[0]:7.1f}°\n"
                    info += f"Roll:    {euler[1]:7.1f}°\n"
                    info += f"Pitch:   {euler[2]:7.1f}°\n\n"
                
                info += f"✅ Visualization pipeline working!\n"
                info += f"If this works, issue is hardware connection."
                
                self.info_text.text = info
                
            except Exception as vpython_error:
                print(f"VPython display error (continuing): {vpython_error}")
                
        except Exception as e:
            print(f"Error updating arrow: {e}")
    
    def run(self):
        """Run the test visualization."""
        print("Starting test visualization with fake data...")
        print("This will test if the visualization pipeline works")
        
        # Start fake data source
        data_thread = self.start_fake_data_source()
        
        try:
            self.info_text.text = "Loading fake data source..."
            
            while True:
                current_time = time.time()
                
                # Frame rate limiting
                if current_time - self.last_update < self.frame_time:
                    time.sleep(0.001)
                    continue
                
                # Get fake data
                try:
                    frame = self.data_queue.get_nowait()
                    self.update_arrow_from_frame(frame)
                    self.last_update = current_time
                except queue.Empty:
                    pass
                
                # Handle VPython events
                try:
                    vp.rate(self.target_fps)
                except Exception as vpython_error:
                    print(f"VPython rate error (continuing): {vpython_error}")
                    time.sleep(1.0 / self.target_fps)
                    
        except KeyboardInterrupt:
            print("\nStopping test visualization...")
        except Exception as e:
            print(f"Error in test visualization: {e}")


def main():
    """Main function."""
    parser = argparse.ArgumentParser(description="Test IMU visualization with fake data")
    parser.add_argument('--length', type=float, default=0.7, help='Arrow length')
    args = parser.parse_args()
    
    print("🧪 TEST: IMU Orientation Visualizer")
    print("=" * 50)
    print("PURPOSE: Test visualization pipeline with fake data")
    print("DATA SOURCE: Simulated IMU data (not real hardware)")
    print("=" * 50)
    print(f"Arrow length: {args.length}m")
    print("\nPress Ctrl+C to stop")
    print("=" * 50)
    
    visualizer = TestOrientationVisualizer(args.length)
    visualizer.run()


if __name__ == "__main__":
    main() 