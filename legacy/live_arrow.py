#!/usr/bin/env python3
"""
Simple VPython live arrow visualization for IMU orientation data.

Displays a red arrow representing the IMU orientation with minimal trail
to help detect orientation drift over time.
"""

import argparse
import time
import numpy as np
from scipy.spatial.transform import Rotation
import vpython as vp
from serial_reader import IMUReader
from reset_arduino import reset_arduino

class SimpleOrientationVisualizer:
    """Simple real-time 3D arrow visualization of IMU orientation data."""
    
    def __init__(self, port: str, baud: int, length: float = 0.7):
        """
        Initialize the visualizer.
        
        Args:
            port: Serial port for IMU
            baud: Baud rate
            length: Length of the arrow in meters
        """
        self.port = port
        self.baud = baud
        self.length = length
        
        # Frame rate control
        self.target_fps = 60
        self.frame_time = 1.0 / self.target_fps
        self.last_update = 0
        
        # Simple trail management for tip position
        self.trail_points = []
        self.max_trail_points = 500  # Reduced for simpler visualization
        
        # VPython scene setup
        self.scene = vp.canvas(
            title=f"IMU Orientation Only - {port}",
            width=800,
            height=600,
            background=vp.color.black,
            center=vp.vector(0, 0, 0),
            range=length * 1.5
        )
        
        # Create arrow (red shaft, starts pointing up in Z direction)
        self.arrow = vp.arrow(
            pos=vp.vector(0, 0, 0),  # Always at origin
            axis=vp.vector(0, 0, length),
            shaftwidth=0.02,
            headwidth=0.04,
            headlength=0.06,
            color=vp.color.red
        )
        
        # Create trail for tip (yellow) - shows orientation history
        self.trail = vp.curve(
            color=vp.color.yellow,
            radius=0.005
        )
        
        # Create coordinate system for reference
        self._create_coordinate_system()
        
        # Add info text
        self.info_text = vp.wtext(text="Connecting to IMU...\n")
        
        # Data tracking
        self.frame_count = 0
        self.start_time = time.time()
        
        print(f"Simple orientation visualizer initialized")
        print(f"Arrow length: {length}m")
        print(f"Target FPS: {self.target_fps}")
    
    def _create_coordinate_system(self):
        """Create reference coordinate system axes."""
        axis_length = self.length * 0.3
        
        # X-axis (red)
        vp.arrow(
            pos=vp.vector(0, 0, 0),
            axis=vp.vector(axis_length, 0, 0),
            shaftwidth=0.005,
            color=vp.color.red,
            opacity=0.5
        )
        vp.label(
            pos=vp.vector(axis_length * 1.1, 0, 0),
            text="X",
            color=vp.color.red,
            height=12
        )
        
        # Y-axis (green)
        vp.arrow(
            pos=vp.vector(0, 0, 0),
            axis=vp.vector(0, axis_length, 0),
            shaftwidth=0.005,
            color=vp.color.green,
            opacity=0.5
        )
        vp.label(
            pos=vp.vector(0, axis_length * 1.1, 0),
            text="Y",
            color=vp.color.green,
            height=12
        )
        
        # Z-axis (blue)
        vp.arrow(
            pos=vp.vector(0, 0, 0),
            axis=vp.vector(0, 0, axis_length),
            shaftwidth=0.005,
            color=vp.color.blue,
            opacity=0.5
        )
        vp.label(
            pos=vp.vector(0, 0, axis_length * 1.1),
            text="Z",
            color=vp.color.blue,
            height=12
        )
    
    def euler_to_quaternion(self, yaw: float, pitch: float, roll: float):
        """Convert Euler angles to quaternion."""
        # Convert degrees to radians
        yaw_rad = np.radians(yaw)
        pitch_rad = np.radians(pitch)
        roll_rad = np.radians(roll)
        
        # Create rotation from Euler angles (ZYX convention)
        rotation = Rotation.from_euler('ZYX', [yaw_rad, pitch_rad, roll_rad])
        quat = rotation.as_quat()  # Returns [x, y, z, w]
        
        # Convert to [w, x, y, z] format
        return (quat[3], quat[0], quat[1], quat[2])
    
    def quaternion_to_rotation_matrix(self, quat: tuple):
        """Convert quaternion to rotation matrix."""
        w, x, y, z = quat
        
        # Handle placeholder quaternions
        if abs(w) < 1e-6 and abs(x) < 1e-6 and abs(y) < 1e-6 and abs(z) < 1e-6:
            return np.eye(3)  # Identity matrix
        
        # Create scipy Rotation object (expects [x, y, z, w])
        rotation = Rotation.from_quat([x, y, z, w])
        return rotation.as_matrix()
    
    def update_arrow_from_frame(self, frame: dict):
        """Update arrow orientation from IMU frame data - orientation only."""
        try:
            # Get quaternion - prefer real quaternion, fall back to Euler conversion
            quat = frame['quat']
            
            # Check if we have placeholder quaternion data
            w, x, y, z = quat
            is_placeholder = (w == 1.0 and x == 0.0 and y == 0.0 and z == 0.0)
            
            if is_placeholder and 'euler' in frame:
                # Convert Euler angles to quaternion
                euler = frame['euler']
                yaw, pitch, roll = euler[0], euler[1], euler[2]
                quat = self.euler_to_quaternion(yaw, pitch, roll)
            
            # Convert quaternion to rotation matrix
            rotation_matrix = self.quaternion_to_rotation_matrix(quat)
            
            # Calculate arrow direction in sensor frame (always pointing "forward")
            arrow_direction = np.array([0, 0, self.length])
            rotated_direction = rotation_matrix @ arrow_direction
            
            # Update VPython arrow - position stays at origin, only orientation changes
            try:
                self.arrow.pos = vp.vector(0, 0, 0)  # Always at origin
                self.arrow.axis = vp.vector(rotated_direction[0], rotated_direction[1], rotated_direction[2])
                
                # Add tip position to trail for drift visualization
                tip_pos = vp.vector(rotated_direction[0], rotated_direction[1], rotated_direction[2])
                self.trail_points.append(tip_pos)
                
                # Limit trail length
                if len(self.trail_points) > self.max_trail_points:
                    self.trail_points = self.trail_points[-self.max_trail_points:]
                
                # Update trail
                self.trail.clear()
                for point in self.trail_points:
                    self.trail.append(pos=point)
                
                # Update info text - show only orientation data
                self.frame_count += 1
                elapsed_time = time.time() - self.start_time
                fps = self.frame_count / elapsed_time if elapsed_time > 0 else 0
                
                info = f"Frame: {self.frame_count}\n"
                info += f"FPS: {fps:.1f}\n"
                info += f"Trail points: {len(self.trail_points)}\n\n"
                
                if 'euler' in frame:
                    euler = frame['euler']
                    info += f"Yaw:   {euler[0]:7.1f}°\n"
                    info += f"Pitch: {euler[1]:7.1f}°\n"
                    info += f"Roll:  {euler[2]:7.1f}°\n\n"
                
                info += f"Quaternion:\n"
                info += f"  w: {quat[0]:6.3f}\n"
                info += f"  x: {quat[1]:6.3f}\n"
                info += f"  y: {quat[2]:6.3f}\n"
                info += f"  z: {quat[3]:6.3f}\n\n"
                
                # Show arrow tip position for drift checking
                info += f"Arrow tip:\n"
                info += f"  x: {rotated_direction[0]:6.3f}\n"
                info += f"  y: {rotated_direction[1]:6.3f}\n"
                info += f"  z: {rotated_direction[2]:6.3f}"
                
                self.info_text.text = info
                
            except Exception as vpython_error:
                # Handle VPython connection errors gracefully
                print(f"VPython display error (continuing): {vpython_error}")
                # Continue processing data even if display fails
            
        except Exception as e:
            print(f"Error updating arrow: {e}")
            import traceback
            traceback.print_exc()
    
    def run(self):
        """Run the live visualization."""
        print(f"Starting simple orientation visualization...")
        print(f"Connecting to {self.port} at {self.baud} baud")
        print(f"This version shows ONLY orientation to help detect drift")
        
        try:
            with IMUReader(self.port, self.baud, chunk_minutes=60) as reader:
                self.info_text.text = "Connected! Waiting for IMU data..."
                
                while True:
                    current_time = time.time()
                    
                    # Frame rate limiting
                    if current_time - self.last_update < self.frame_time:
                        time.sleep(0.001)
                        continue
                    
                    # Get latest frame
                    frame = reader.get_frame(timeout=0.001)  # Non-blocking
                    
                    if frame:
                        self.update_arrow_from_frame(frame)
                        self.last_update = current_time
                    
                    # Handle VPython events with error handling
                    try:
                        vp.rate(self.target_fps)
                    except Exception as vpython_error:
                        # VPython window might be closed, continue processing data
                        print(f"VPython rate error (continuing): {vpython_error}")
                        time.sleep(1.0 / self.target_fps)  # Manual rate limiting
                    
        except KeyboardInterrupt:
            print("\nShutting down visualization...")
        except Exception as e:
            print(f"Error in visualization: {e}")
            self.info_text.text = f"Error: {e}"


def main():
    """Main function with command line argument parsing."""
    parser = argparse.ArgumentParser(
        description="Simple VPython orientation visualization for drift detection"
    )
    
    parser.add_argument(
        '--port',
        type=str,
        default='/dev/cu.usbmodem1101',
        help='Serial port for IMU (default: /dev/cu.usbmodem1101)'
    )
    
    parser.add_argument(
        '--baud',
        type=int,
        default=115200,
        help='Baud rate (default: 115200)'
    )
    
    parser.add_argument(
        '--length',
        type=float,
        default=0.7,
        help='Arrow length in meters (default: 0.7)'
    )
    
    parser.add_argument(
        '--reset',
        action='store_true',
        help='Reset Arduino before connecting (fixes CircuitPython connection issues)'
    )
    
    args = parser.parse_args()
    
    print("Simple IMU Orientation Visualizer")
    print("=" * 40)
    print("PURPOSE: Check for orientation drift")
    print("REMOVED: Acceleration, velocity, position tracking")
    print("SHOWS: Pure orientation data only")
    print("=" * 40)
    print(f"Port: {args.port}")
    print(f"Baud: {args.baud}")
    print(f"Arrow length: {args.length}m")
    if args.reset:
        print("Reset: Enabled (will reset Arduino before connecting)")
    print("\nPress Ctrl+C to stop")
    print("=" * 40)
    
    # Reset Arduino if requested
    if args.reset:
        print("\n🔄 Resetting Arduino to clear CircuitPython state...")
        if not reset_arduino(args.port, args.baud):
            print("❌ Reset failed. You may need to manually reconnect USB.")
            print("Try running: python reset_arduino.py --port", args.port)
            return
        print("✅ Reset successful!\n")
    
    # Create and run visualizer
    visualizer = SimpleOrientationVisualizer(args.port, args.baud, args.length)
    visualizer.run()


if __name__ == "__main__":
    main() 