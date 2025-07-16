#!/usr/bin/env python3
"""
DEBUG VERSION: VPython live arrow visualization for IMU orientation data.

This version includes extensive debugging to help diagnose issues with
data reception, parsing, and 3D visualization.
"""

import argparse
import time
import numpy as np
from scipy.spatial.transform import Rotation
import vpython as vp
from serial_reader import IMUReader
from reset_arduino import reset_arduino

class DebugOrientationVisualizer:
    """Debug version with extensive logging of data flow and 3D visualization."""
    
    def __init__(self, port: str, baud: int, length: float = 0.7):
        """
        Initialize the debug visualizer.
        
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
        
        # Debug counters
        self.frames_received = 0
        self.frames_processed = 0
        self.frames_skipped = 0
        self.visualization_errors = 0
        self.data_errors = 0
        
        # Simple trail management for tip position
        self.trail_points = []
        self.max_trail_points = 500
        
        print(f"🔧 DEBUG: Initializing visualizer...")
        print(f"🔧 DEBUG: Port: {port}, Baud: {baud}, Arrow length: {length}")
        
        # VPython scene setup
        try:
            print(f"🔧 DEBUG: Creating VPython scene...")
            self.scene = vp.canvas(
                title=f"DEBUG IMU Orientation - {port}",
                width=800,
                height=600,
                background=vp.color.black,
                center=vp.vector(0, 0, 0),
                range=length * 1.5
            )
            print(f"✅ DEBUG: VPython scene created successfully")
        except Exception as e:
            print(f"❌ DEBUG: Failed to create VPython scene: {e}")
            raise
        
        # Create objects
        try:
            print(f"🔧 DEBUG: Creating 3D objects...")
            
            # Main arrow
            self.arrow = vp.arrow(
                pos=vp.vector(0, 0, 0),
                axis=vp.vector(0, 0, length),
                shaftwidth=0.02,
                color=vp.color.red,
                opacity=0.9
            )
            
            # Trail curve
            self.trail = vp.curve(color=vp.color.cyan, radius=0.002)
            
            # Debug info text
            self.info_text = vp.wtext(
                text="DEBUG: Starting...",
                pos=self.scene.title_anchor
            )
            
            # Coordinate axes for reference
            vp.arrow(pos=vp.vector(0,0,0), axis=vp.vector(0.3,0,0), color=vp.color.red, shaftwidth=0.005)   # X-axis
            vp.arrow(pos=vp.vector(0,0,0), axis=vp.vector(0,0.3,0), color=vp.color.green, shaftwidth=0.005) # Y-axis  
            vp.arrow(pos=vp.vector(0,0,0), axis=vp.vector(0,0,0.3), color=vp.color.blue, shaftwidth=0.005)  # Z-axis
            
            print(f"✅ DEBUG: 3D objects created successfully")
            
        except Exception as e:
            print(f"❌ DEBUG: Failed to create 3D objects: {e}")
            raise
            
        # Initialize counters
        self.frame_count = 0
        self.start_time = time.time()
        
        print(f"✅ DEBUG: Visualizer initialization complete")

    def euler_to_quaternion(self, yaw: float, pitch: float, roll: float) -> tuple:
        """Convert Euler angles to quaternion with debug output."""
        print(f"🔧 DEBUG: Converting Euler to quaternion - Yaw: {yaw:.2f}°, Pitch: {pitch:.2f}°, Roll: {roll:.2f}°")
        
        try:
            # Convert degrees to radians
            yaw_rad = np.radians(yaw)
            pitch_rad = np.radians(pitch)
            roll_rad = np.radians(roll)
            
            # Create scipy Rotation object from Euler angles (yaw, pitch, roll order)
            rotation = Rotation.from_euler('ZYX', [yaw_rad, pitch_rad, roll_rad])
            quat = rotation.as_quat()  # Returns [x, y, z, w]
            
            # Convert to [w, x, y, z] format
            result = (quat[3], quat[0], quat[1], quat[2])
            print(f"✅ DEBUG: Euler conversion result - w: {result[0]:.3f}, x: {result[1]:.3f}, y: {result[2]:.3f}, z: {result[3]:.3f}")
            return result
            
        except Exception as e:
            print(f"❌ DEBUG: Euler to quaternion conversion failed: {e}")
            self.data_errors += 1
            return (1.0, 0.0, 0.0, 0.0)  # Identity quaternion

    def quaternion_to_rotation_matrix(self, quat: tuple) -> np.ndarray:
        """Convert quaternion to rotation matrix with debug output."""
        w, x, y, z = quat
        print(f"🔧 DEBUG: Converting quaternion to rotation matrix - w: {w:.3f}, x: {x:.3f}, y: {y:.3f}, z: {z:.3f}")
        
        try:
            # Check for zero quaternion
            if abs(w) < 1e-6 and abs(x) < 1e-6 and abs(y) < 1e-6 and abs(z) < 1e-6:
                print(f"⚠️ DEBUG: Zero quaternion detected, using identity matrix")
                return np.eye(3)
            
            # Create scipy Rotation object (expects [x, y, z, w])
            rotation = Rotation.from_quat([x, y, z, w])
            matrix = rotation.as_matrix()
            print(f"✅ DEBUG: Rotation matrix conversion successful")
            return matrix
            
        except Exception as e:
            print(f"❌ DEBUG: Quaternion to rotation matrix conversion failed: {e}")
            self.data_errors += 1
            return np.eye(3)

    def update_arrow_from_frame(self, frame: dict):
        """Update arrow orientation from IMU frame data with extensive debugging."""
        print(f"\n🔧 DEBUG: ========== FRAME UPDATE {self.frames_processed + 1} ==========")
        print(f"🔧 DEBUG: Frame keys: {list(frame.keys())}")
        
        try:
            # Debug: Show raw frame data
            if 'euler' in frame:
                euler = frame['euler']
                print(f"🔧 DEBUG: Raw Euler data: {euler}")
            
            if 'quat' in frame:
                quat = frame['quat']
                print(f"🔧 DEBUG: Raw quaternion data: {quat}")
            
            # Get quaternion - prefer real quaternion, fall back to Euler conversion
            quat = frame['quat']
            
            # Check if we have placeholder quaternion data
            w, x, y, z = quat
            is_placeholder = (w == 1.0 and x == 0.0 and y == 0.0 and z == 0.0)
            print(f"🔧 DEBUG: Quaternion placeholder check - is_placeholder: {is_placeholder}")
            
            if is_placeholder and 'euler' in frame:
                print(f"🔧 DEBUG: Using Euler to quaternion conversion")
                # Convert Euler angles to quaternion
                euler = frame['euler']
                yaw, pitch, roll = euler[0], euler[1], euler[2]
                quat = self.euler_to_quaternion(yaw, pitch, roll)
                print(f"🔧 DEBUG: Converted quaternion: {quat}")
            else:
                print(f"🔧 DEBUG: Using direct quaternion data: {quat}")
            
            # Convert quaternion to rotation matrix
            print(f"🔧 DEBUG: Converting to rotation matrix...")
            rotation_matrix = self.quaternion_to_rotation_matrix(quat)
            
            # Calculate arrow direction in sensor frame (always pointing "forward")
            arrow_direction = np.array([0, 0, self.length])
            print(f"🔧 DEBUG: Base arrow direction: {arrow_direction}")
            
            rotated_direction = rotation_matrix @ arrow_direction
            print(f"🔧 DEBUG: Rotated arrow direction: {rotated_direction}")
            
            # Update VPython arrow - position stays at origin, only orientation changes
            try:
                print(f"🔧 DEBUG: Updating VPython arrow...")
                
                self.arrow.pos = vp.vector(0, 0, 0)  # Always at origin
                self.arrow.axis = vp.vector(rotated_direction[0], rotated_direction[1], rotated_direction[2])
                
                print(f"✅ DEBUG: Arrow updated - pos: (0,0,0), axis: ({rotated_direction[0]:.3f}, {rotated_direction[1]:.3f}, {rotated_direction[2]:.3f})")
                
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
                
                print(f"✅ DEBUG: Trail updated - {len(self.trail_points)} points")
                
                # Update info text - show debug information
                self.frame_count += 1
                elapsed_time = time.time() - self.start_time
                fps = self.frame_count / elapsed_time if elapsed_time > 0 else 0
                
                info = f"DEBUG VISUALIZATION\n"
                info += f"===================\n"
                info += f"Frame: {self.frame_count}\n"
                info += f"FPS: {fps:.1f}\n"
                info += f"Received: {self.frames_received}\n"
                info += f"Processed: {self.frames_processed + 1}\n"
                info += f"Skipped: {self.frames_skipped}\n"
                info += f"Viz Errors: {self.visualization_errors}\n"
                info += f"Data Errors: {self.data_errors}\n"
                info += f"Trail points: {len(self.trail_points)}\n\n"
                
                if 'euler' in frame:
                    euler = frame['euler']
                    info += f"ORIENTATION DATA:\n"
                    info += f"Yaw:   {euler[0]:7.1f}°\n"
                    info += f"Pitch: {euler[1]:7.1f}°\n"
                    info += f"Roll:  {euler[2]:7.1f}°\n\n"
                
                info += f"QUATERNION:\n"
                info += f"  w: {quat[0]:6.3f}\n"
                info += f"  x: {quat[1]:6.3f}\n"
                info += f"  y: {quat[2]:6.3f}\n"
                info += f"  z: {quat[3]:6.3f}\n\n"
                
                # Show arrow tip position for drift checking
                info += f"ARROW TIP:\n"
                info += f"  x: {rotated_direction[0]:6.3f}\n"
                info += f"  y: {rotated_direction[1]:6.3f}\n"
                info += f"  z: {rotated_direction[2]:6.3f}\n\n"
                
                info += f"STATUS: ✅ VISUALIZATION OK"
                
                self.info_text.text = info
                print(f"✅ DEBUG: Info text updated")
                
            except Exception as vpython_error:
                # Handle VPython connection errors gracefully
                self.visualization_errors += 1
                print(f"❌ DEBUG: VPython display error: {vpython_error}")
                print(f"⚠️ DEBUG: Visualization errors: {self.visualization_errors}")
                # Continue processing data even if display fails
            
            self.frames_processed += 1
            print(f"✅ DEBUG: Frame processing complete - Total processed: {self.frames_processed}")
            
        except Exception as e:
            self.data_errors += 1
            print(f"❌ DEBUG: Error updating arrow: {e}")
            print(f"❌ DEBUG: Data errors: {self.data_errors}")
            import traceback
            traceback.print_exc()

    def run(self):
        """Run the live visualization with extensive debugging."""
        print(f"\n🚀 DEBUG: Starting debug visualization...")
        print(f"🚀 DEBUG: Connecting to {self.port} at {self.baud} baud")
        print(f"🚀 DEBUG: This debug version shows detailed data flow information")
        
        try:
            with IMUReader(self.port, self.baud, chunk_minutes=60) as reader:
                print(f"✅ DEBUG: IMUReader connection established")
                self.info_text.text = "DEBUG: Connected! Waiting for IMU data..."
                
                # Debug loop counter
                loop_count = 0
                
                while True:
                    loop_count += 1
                    current_time = time.time()
                    
                    # Debug every 100 loops
                    if loop_count % 100 == 0:
                        print(f"🔧 DEBUG: Main loop iteration {loop_count}")
                    
                    # Frame rate limiting
                    if current_time - self.last_update < self.frame_time:
                        time.sleep(0.001)
                        continue
                    
                    # Get latest frame
                    frame = reader.get_frame(timeout=0.001)  # Non-blocking
                    
                    if frame:
                        self.frames_received += 1
                        print(f"\n📦 DEBUG: NEW FRAME RECEIVED #{self.frames_received}")
                        print(f"📦 DEBUG: Frame timestamp: {frame.get('ts', 'Unknown')}")
                        self.update_arrow_from_frame(frame)
                        self.last_update = current_time
                    else:
                        # Debug: No frame received
                        if loop_count % 1000 == 0:  # Log every 1000 empty reads
                            print(f"⏳ DEBUG: No frame received (loop {loop_count})")
                    
                    # Handle VPython events with error handling
                    try:
                        vp.rate(self.target_fps)
                    except Exception as vpython_error:
                        self.visualization_errors += 1
                        print(f"❌ DEBUG: VPython rate error: {vpython_error}")
                        print(f"⚠️ DEBUG: Total VPython errors: {self.visualization_errors}")
                        time.sleep(1.0 / self.target_fps)  # Manual rate limiting
                    
        except KeyboardInterrupt:
            print(f"\n🛑 DEBUG: Shutting down visualization...")
            print(f"📊 DEBUG: Final stats - Received: {self.frames_received}, Processed: {self.frames_processed}")
        except Exception as e:
            print(f"❌ DEBUG: Error in visualization: {e}")
            import traceback
            traceback.print_exc()
            if hasattr(self, 'info_text'):
                self.info_text.text = f"ERROR: {e}"


def main():
    """Main function with command line argument parsing."""
    parser = argparse.ArgumentParser(
        description="DEBUG version of VPython orientation visualization"
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
        default=9600,  # Changed default to CircuitPython baud rate
        help='Baud rate (default: 9600 for CircuitPython)'
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
    
    print("🔧 DEBUG IMU Orientation Visualizer")
    print("=" * 50)
    print("PURPOSE: Debug orientation data flow and visualization")
    print("FEATURES: Detailed logging of all data processing steps")
    print("=" * 50)
    print(f"Port: {args.port}")
    print(f"Baud: {args.baud}")
    print(f"Arrow length: {args.length}m")
    if args.reset:
        print("Reset: Enabled (will reset Arduino before connecting)")
    print("\nPress Ctrl+C to stop")
    print("=" * 50)
    
    # Reset Arduino if requested
    if args.reset:
        print("\n🔄 DEBUG: Resetting Arduino to clear CircuitPython state...")
        if not reset_arduino(args.port, args.baud):
            print("❌ DEBUG: Reset failed. You may need to manually reconnect USB.")
            print("Try running: python reset_arduino.py --port", args.port)
            return
        print("✅ DEBUG: Reset successful!\n")
    
    # Create and run visualizer
    print("🔧 DEBUG: Creating visualizer...")
    visualizer = DebugOrientationVisualizer(args.port, args.baud, args.length)
    print("🚀 DEBUG: Starting visualization...")
    visualizer.run()


if __name__ == "__main__":
    main() 