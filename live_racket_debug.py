#!/usr/bin/env python3
"""
DEBUG VERSION: VPython live racket visualization for IMU orientation data.

Refactored version that separates 3D scene management from data processing.
"""

import argparse
import time
import numpy as np
from scipy.spatial.transform import Rotation
import vpython as vp
from serial_reader import IMUReader
from reset_arduino import reset_arduino


class RacketScene:
    """3D scene management for racket visualization."""
    
    def __init__(self, length: float = 0.70, head_radius: float = 0.15):
        """
        Initialize the 3D racket scene.
        
        Args:
            length: Length of the racket in meters
            head_radius: Radius of the racket head in meters
        """
        self.length = length
        self.head_radius = head_radius
        
        # Simple trail management for tip position
        self.trail_points = []
        self.max_trail_points = 500
        
        print(f"🔧 DEBUG: Creating RacketScene with length={length}, head_radius={head_radius}")
        
        # VPython scene setup
        try:
            print(f"🔧 DEBUG: Creating VPython scene...")
            self.scene = vp.canvas(
                title="DEBUG IMU Racket Visualization",
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
        
        # Create 3D objects
        try:
            print(f"🔧 DEBUG: Creating 3D squash racket objects...")
            
            # Racket components
            handle_length = 0.15  # 15cm handle
            shaft_length = 0.30   # 30cm shaft
            total_length = handle_length + shaft_length
            
            # Handle: brown cylinder
            handle = vp.cylinder(
                pos=vp.vector(0, 0, 0),
                axis=vp.vector(0, 0, handle_length),
                radius=0.015,
                color=vp.color.orange * 0.7  # Brown-ish color
            )
            
            # Shaft: grey cylinder, attached to handle
            shaft = vp.cylinder(
                pos=vp.vector(0, 0, handle_length),
                axis=vp.vector(0, 0, shaft_length),
                radius=0.01,
                color=vp.color.gray(0.6)
            )
            
            # Head: thin torus-like ring at end of shaft
            head_center_z = handle_length + shaft_length
            head_ring = vp.ring(
                pos=vp.vector(0, 0, head_center_z),
                axis=vp.vector(0, 0, 1),
                radius=head_radius,
                thickness=0.01,
                color=vp.color.cyan * 0.8  # Light blue
            )
            
            # Create compound object so single rotate() moves all parts
            self.racket_compound = vp.compound([handle, shaft, head_ring])
            
            # Store head center position for trail tracking
            self.head_center_local = vp.vector(0, 0, head_center_z)
            
            print(f"✅ DEBUG: Compound squash racket created - Handle: {handle_length}m, Shaft: {shaft_length}m, Head radius: {head_radius}m")
            
            # Create squash court environment
            print(f"🔧 DEBUG: Creating squash court environment...")
            
            # Court dimensions (simplified squash singles court)
            court_length = 8.0  # 8m depth
            court_width = 5.0   # 5m width
            wall_height = 3.0   # 3m height
            
            # Floor plane (translucent grey)
            floor = vp.box(
                pos=vp.vector(0, -0.01, -court_length/2),  # Slightly below origin
                size=vp.vector(court_width, 0.02, court_length),
                color=vp.color.gray(0.7),
                opacity=0.3
            )
            
            # Front wall at z = -4m (semi-transparent white)
            front_wall = vp.box(
                pos=vp.vector(0, wall_height/2, -4.0),
                size=vp.vector(court_width, wall_height, 0.1),
                color=vp.color.white,
                opacity=0.15
            )
            
            # Side walls at x = ±2.75m (semi-transparent white)
            left_wall = vp.box(
                pos=vp.vector(-2.75, wall_height/2, -court_length/2),
                size=vp.vector(0.1, wall_height, court_length),
                color=vp.color.white,
                opacity=0.15
            )
            
            right_wall = vp.box(
                pos=vp.vector(2.75, wall_height/2, -court_length/2),
                size=vp.vector(0.1, wall_height, court_length),
                color=vp.color.white,
                opacity=0.15
            )
            
            # Service box markings (thin red curves)
            print(f"🔧 DEBUG: Adding service box markings...")
            
            # Front wall service line at 1.78m high
            front_service_line = vp.curve(
                color=vp.color.red,
                radius=0.005
            )
            front_service_line.append(pos=vp.vector(-court_width/2, 1.78, -3.99))
            front_service_line.append(pos=vp.vector(court_width/2, 1.78, -3.99))
            
            # Service line (typically ~5.18m from front wall, but adjusted for 8m court)
            service_line_z = -4.0 + 3.2  # About 3.2m from front wall
            service_line = vp.curve(
                color=vp.color.red,
                radius=0.005
            )
            service_line.append(pos=vp.vector(-court_width/2, 0.01, service_line_z))
            service_line.append(pos=vp.vector(court_width/2, 0.01, service_line_z))
            
            # Short line (typically ~4.26m from front wall, but adjusted)
            short_line_z = -4.0 + 2.0  # About 2m from front wall
            short_line = vp.curve(
                color=vp.color.red,
                radius=0.005
            )
            short_line.append(pos=vp.vector(-court_width/2, 0.01, short_line_z))
            short_line.append(pos=vp.vector(court_width/2, 0.01, short_line_z))
            
            # Center line (divides service boxes)
            center_line = vp.curve(
                color=vp.color.red,
                radius=0.005
            )
            center_line.append(pos=vp.vector(0, 0.01, service_line_z))
            center_line.append(pos=vp.vector(0, 0.01, short_line_z))
            
            print(f"✅ DEBUG: Squash court environment created")
            
            # Enhanced lighting
            print(f"🔧 DEBUG: Setting up enhanced lighting...")
            
            # Distant light pointing downwards (simulates ceiling lighting)
            distant_light = vp.distant_light(
                direction=vp.vector(0, -1, -0.2),  # Pointing down and slightly forward
                color=vp.color.white * 0.8
            )
            
            # Local light near camera position for softer shadows
            local_light = vp.local_light(
                pos=vp.vector(2, 2, 2),  # Above and to the side
                color=vp.color.white * 0.6
            )
            
            print(f"✅ DEBUG: Enhanced lighting configured")
            
            
            # Trail curve
            self.trail = vp.curve(color=vp.color.yellow, radius=0.002)
            
            # Debug info text
            self.info_text = vp.wtext(
                text="DEBUG: Starting...",
                pos=self.scene.title_anchor
            )
            
            # Coordinate axes for reference
            vp.arrow(pos=vp.vector(0,0,0), axis=vp.vector(0.3,0,0), color=vp.color.red, shaftwidth=0.005)   # X-axis
            vp.arrow(pos=vp.vector(0,0,0), axis=vp.vector(0,0.3,0), color=vp.color.green, shaftwidth=0.005) # Y-axis  
            vp.arrow(pos=vp.vector(0,0,0), axis=vp.vector(0,0,0.3), color=vp.color.blue, shaftwidth=0.005)  # Z-axis
            
            print(f"✅ DEBUG: 3D squash racket objects created successfully")
            
        except Exception as e:
            print(f"❌ DEBUG: Failed to create 3D objects: {e}")
            raise
    
    def update(self, quat: tuple, tip_vec: np.ndarray, speed_mps: float):
        """
        Update racket orientation and position.
        
        Args:
            quat: Quaternion (w, x, y, z)
            tip_vec: 3D vector to racket tip position (not used with compound)
            speed_mps: Speed in meters per second (legacy parameter, recalculated internally)
        """
        try:
            print(f"🔧 DEBUG: Updating RacketScene - quat: {quat}")
            
            # Convert quaternion to scipy Rotation
            w, x, y, z = quat
            rotation = Rotation.from_quat([x, y, z, w])
            
            # Set compound object orientation using rotation matrix
            rotation_matrix = rotation.as_matrix()
            
            # Calculate new axis (Z direction) and up (Y direction) vectors
            # The racket points along Z axis in local coordinates
            new_axis = rotation_matrix @ np.array([0, 0, 1])
            new_up = rotation_matrix @ np.array([0, 1, 0])
            
            # Set compound object orientation directly
            self.racket_compound.axis = vp.vector(new_axis[0], new_axis[1], new_axis[2])
            self.racket_compound.up = vp.vector(new_up[0], new_up[1], new_up[2])
            
            # Calculate head center position in world coordinates
            head_center_world = rotation_matrix @ np.array([
                self.head_center_local.x,
                self.head_center_local.y, 
                self.head_center_local.z
            ])
            
            # Compute instantaneous linear speed of head center over last two frames
            current_time = time.time()
            instantaneous_speed = 0.0
            
            if hasattr(self, 'previous_head_pos') and hasattr(self, 'previous_time'):
                # Calculate distance moved
                distance = np.linalg.norm(head_center_world - self.previous_head_pos)
                # Calculate time elapsed
                time_delta = current_time - self.previous_time
                
                if time_delta > 0:
                    instantaneous_speed = distance / time_delta
                    print(f"🔧 DEBUG: Instantaneous speed: {instantaneous_speed:.2f} m/s")
            
            # Store current position and time for next frame
            self.previous_head_pos = head_center_world.copy()
            self.previous_time = current_time
            
            # Map speed to color: 0-5 m/s → blue, 5-10 m/s → gradient to yellow, >10 m/s → red
            def speed_to_color(speed):
                if speed < 5.0:
                    return vp.color.blue
                elif speed <= 10.0:
                    # Gradient from blue to yellow to red
                    ratio = (speed - 5.0) / 5.0  # 0.0 to 1.0
                    if ratio <= 0.5:
                        # Blue to yellow (0.0 to 0.5)
                        blend = ratio * 2.0
                        return vp.vector(blend, blend, 1.0 - blend)  # Blue to yellow
                    else:
                        # Yellow to red (0.5 to 1.0)
                        blend = (ratio - 0.5) * 2.0
                        return vp.vector(1.0, 1.0 - blend, 0.0)  # Yellow to red
                else:
                    return vp.color.red
            
            # Apply color to racket compound
            racket_color = speed_to_color(instantaneous_speed)
            self.racket_compound.color = racket_color
            
            print(f"🔧 DEBUG: Applied color for speed {instantaneous_speed:.2f} m/s: {racket_color}")
            
            # Store speed for external access (for stats display)
            self.current_speed = instantaneous_speed
            
            # Add head center position to trail for drift visualization
            head_pos = vp.vector(head_center_world[0], head_center_world[1], head_center_world[2])
            self.trail_points.append(head_pos)
            
            # Limit trail length
            if len(self.trail_points) > self.max_trail_points:
                self.trail_points = self.trail_points[-self.max_trail_points:]
            
            # Update trail
            self.trail.clear()
            for point in self.trail_points:
                self.trail.append(pos=point)
            
            print(f"✅ DEBUG: RacketScene update complete - {len(self.trail_points)} trail points, {instantaneous_speed:.2f} m/s")
            
        except Exception as e:
            print(f"❌ DEBUG: Error updating RacketScene: {e}")
            import traceback
            traceback.print_exc()
    
    def update_info(self, debug_info: str):
        """Update the debug info text display."""
        try:
            self.info_text.text = debug_info
        except Exception as e:
            print(f"❌ DEBUG: Error updating info text: {e}")


class DebugRacketVisualizer:
    """Debug version with extensive logging of data flow and 3D racket visualization."""
    
    def __init__(self, port: str, baud: int, length: float = 0.7, head_radius: float = 0.15):
        """
        Initialize the debug racket visualizer.
        
        Args:
            port: Serial port for IMU
            baud: Baud rate
            length: Length of the racket in meters
            head_radius: Radius of the racket head in meters
        """
        self.port = port
        self.baud = baud
        
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
        
        print(f"🔧 DEBUG: Initializing racket visualizer...")
        print(f"🔧 DEBUG: Port: {port}, Baud: {baud}, Racket length: {length}, Head radius: {head_radius}")
        
        # Create racket scene (but don't start reader yet)
        self.scene = RacketScene(length, head_radius)
        
        # Initialize counters
        self.frame_count = 0
        self.start_time = time.time()
        
        print(f"✅ DEBUG: Racket visualizer initialization complete")

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

    def update_racket_from_frame(self, frame: dict):
        """Update racket orientation from IMU frame data with extensive debugging."""
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
            
            # Calculate racket head center direction (for reference and speed calculation)
            # Head is at Z = 0.15 + 0.30 = 0.45m in local coordinates
            head_center_local = np.array([0, 0, 0.45])
            print(f"🔧 DEBUG: Base head center position: {head_center_local}")
            
            rotated_head_center = rotation_matrix @ head_center_local
            print(f"🔧 DEBUG: Rotated head center: {rotated_head_center}")
            
            # Update scene using the new interface (speed calculated internally)
            try:
                print(f"🔧 DEBUG: Updating RacketScene...")
                self.scene.update(quat, rotated_head_center, 0.0)  # Speed calculated internally now
                print(f"✅ DEBUG: RacketScene updated successfully")
                
                # Update info text - show debug information
                self.frame_count += 1
                elapsed_time = time.time() - self.start_time
                fps = self.frame_count / elapsed_time if elapsed_time > 0 else 0
                
                info = f"DEBUG RACKET VISUALIZATION\n"
                info += f"==========================\n"
                info += f"Frame: {self.frame_count}\n"
                info += f"FPS: {fps:.1f}\n"
                info += f"Received: {self.frames_received}\n"
                info += f"Processed: {self.frames_processed + 1}\n"
                info += f"Skipped: {self.frames_skipped}\n"
                info += f"Viz Errors: {self.visualization_errors}\n"
                info += f"Data Errors: {self.data_errors}\n"
                info += f"Trail points: {len(self.scene.trail_points)}\n\n"
                
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
                
                # Show racket head center position for drift checking
                info += f"RACKET HEAD CENTER:\n"
                info += f"  x: {rotated_head_center[0]:6.3f}\n"
                info += f"  y: {rotated_head_center[1]:6.3f}\n"
                info += f"  z: {rotated_head_center[2]:6.3f}\n"
                
                # Get speed from scene (calculated internally)
                scene_speed = getattr(self.scene, 'current_speed', 0.0)
                info += f"Speed: {scene_speed:6.3f} m/s\n"
                
                # Speed color indicator
                if scene_speed < 5.0:
                    speed_color = "BLUE (slow)"
                elif scene_speed <= 10.0:
                    speed_color = "YELLOW (medium)"
                else:
                    speed_color = "RED (fast)"
                info += f"Color: {speed_color}\n\n"
                
                info += f"STATUS: ✅ VISUALIZATION OK"
                
                self.scene.update_info(info)
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
            print(f"❌ DEBUG: Error updating racket: {e}")
            print(f"❌ DEBUG: Data errors: {self.data_errors}")
            import traceback
            traceback.print_exc()

    def run(self):
        """Run the live racket visualization with extensive debugging."""
        print(f"\n🚀 DEBUG: Starting debug racket visualization...")
        print(f"🚀 DEBUG: Connecting to {self.port} at {self.baud} baud")
        print(f"🚀 DEBUG: This debug version shows detailed data flow information")
        
        try:
            with IMUReader(self.port, self.baud, chunk_minutes=60) as reader:
                print(f"✅ DEBUG: IMUReader connection established")
                self.scene.update_info("DEBUG: Connected! Waiting for IMU data...")
                
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
                        self.update_racket_from_frame(frame)
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
            if hasattr(self, 'scene'):
                self.scene.update_info(f"ERROR: {e}")


def main():
    """Main function with command line argument parsing."""
    parser = argparse.ArgumentParser(
        description="DEBUG version of VPython racket orientation visualization"
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
        default=115200,  # Changed default to CircuitPython baud rate
        help='Baud rate (default: 115200 for CircuitPython)'
    )
    
    parser.add_argument(
        '--length',
        type=float,
        default=0.7,
        help='Racket length in meters (default: 0.7)'
    )
    
    parser.add_argument(
        '--head-radius',
        type=float,
        default=0.15,
        help='Racket head radius in meters (default: 0.15)'
    )
    
    parser.add_argument(
        '--reset',
        action='store_true',
        help='Reset Arduino before connecting (fixes CircuitPython connection issues)'
    )
    
    args = parser.parse_args()
    
    print("🔧 DEBUG IMU Racket Orientation Visualizer")
    print("=" * 50)
    print("PURPOSE: Debug racket orientation data flow and visualization")
    print("FEATURES: Detailed logging of all data processing steps")
    print("=" * 50)
    print(f"Port: {args.port}")
    print(f"Baud: {args.baud}")
    print(f"Racket length: {args.length}m")
    print(f"Head radius: {args.head_radius}m")
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
    print("🔧 DEBUG: Creating racket visualizer...")
    visualizer = DebugRacketVisualizer(args.port, args.baud, args.length, args.head_radius)
    print("🚀 DEBUG: Starting visualization...")
    visualizer.run()


if __name__ == "__main__":
    main() 