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
import serial.tools.list_ports
import serial


class BoardErrorRecovery:
    """Handles various board errors and recovery mechanisms for BNO055/KB2040."""
    
    def __init__(self, port: str, baud: int = 115200):
        self.port = port
        self.baud = baud
        self.recovery_attempts = 0
        self.max_recovery_attempts = 3
        self.last_recovery_time = 0
        self.recovery_cooldown = 5  # seconds - reduced from 30
        self.error_patterns = {
            'uart_error_7': [
                'uart error 7',
                'UART error 7',
                'Error 7',
                'error 7',
                'BNO055 error',
                'BNO055 Error',
                'I2C error',
                'I2C Error'
            ],
            'no_data': [
                'No data',
                'no data',
                'timeout',
                'Timeout'
            ],
            'connection_lost': [
                'Connection lost',
                'connection lost',
                'Serial port closed',
                'serial port closed'
            ],
            'red_led': [
                'red LED',
                'RED LED',
                'red led',
                'RED led'
            ]
        }
        
    def detect_error(self, data_line: str, no_data_timeout: float = 10.0) -> tuple[str, bool]:
        """
        Detect various error conditions from data or lack thereof.
        More selective to avoid false positives.
        
        Args:
            data_line: Latest data line from serial
            no_data_timeout: Seconds without data to trigger no_data error
            
        Returns:
            tuple: (error_type, needs_recovery)
        """
        if not data_line:
            return 'no_data', False  # Don't trigger recovery immediately
            
        data_lower = data_line.lower()
        
        # Check for CircuitPython crash indicators (red LED blinking)
        circuitpython_crash_patterns = [
            'code ended due to an exception',
            'traceback (most recent call last)',
            'traceback',
            'exception',
            'runtimeerror',
            'valueerror',
            'attributeerror',
            'keyerror',
            'importerror',
            'syntaxerror'
        ]
        
        for pattern in circuitpython_crash_patterns:
            if pattern in data_lower:
                print(f"🚨 CIRCUITPYTHON CRASH: {pattern}")
                return 'circuitpython_crash', True
        
        # Check for specific hardware errors that need recovery
        hardware_error_patterns = {
            'uart_error_7': ['uart error 7', 'error 7'],
            'bno055_error': ['bno055 error', 'bno055 fail', 'bno055 initialization failed'],
            'i2c_error': ['i2c error', 'i2c fail', 'i2c timeout']
        }
        
        for error_type, patterns in hardware_error_patterns.items():
            for pattern in patterns:
                if pattern in data_lower:
                    print(f"🚨 HARDWARE ERROR: Detected {error_type} - {pattern}")
                    return error_type, True
                
        return 'none', False
    
    def can_attempt_recovery(self) -> bool:
        """Check if we can attempt recovery based on cooldown and attempt limits."""
        current_time = time.time()
        
        if self.recovery_attempts >= self.max_recovery_attempts:
            return False
            
        # Allow first attempt immediately, then apply cooldown
        if self.recovery_attempts > 0 and current_time - self.last_recovery_time < self.recovery_cooldown:
            return False
            
        return True
    
    def perform_circuitpython_soft_reset(self) -> bool:
        """Perform CircuitPython soft reset (Ctrl+C, Ctrl+D)."""
        print(f"🔄 DEBUG: Performing CircuitPython soft reset on {self.port}")
        
        try:
            with serial.Serial(self.port, self.baud, timeout=2) as ser:
                print("✅ DEBUG: Connected for soft reset")
                
                # Send Ctrl+C to stop current code
                ser.write(b'\x03')
                time.sleep(1)
                
                # Send Ctrl+D for soft reboot
                ser.write(b'\x04')
                time.sleep(3)
                
                print("✅ DEBUG: Soft reset commands sent")
                
                # Check for recovery
                print("🔍 DEBUG: Checking for recovery...")
                recovered = False
                for i in range(10):
                    if ser.in_waiting > 0:
                        line = ser.readline().decode('utf-8', errors='ignore').strip()
                        print(f"📥 DEBUG: {line}")
                        if 'orientation:' in line.lower() or 'bno055' in line.lower():
                            recovered = True
                            break
                    time.sleep(0.5)
                
                if recovered:
                    print("🎉 DEBUG: Soft reset recovery successful!")
                    return True
                else:
                    print("⚠️  DEBUG: Soft reset sent, but no data detected yet")
                    return False
                    
        except Exception as e:
            print(f"❌ DEBUG: Soft reset failed: {e}")
            return False
    
    def perform_hardware_reset(self) -> bool:
        """Perform hardware reset via DTR/RTS signals."""
        print(f"🔄 DEBUG: Performing hardware reset on {self.port}")
        
        try:
            with serial.Serial(
                port=self.port,
                baudrate=self.baud,
                timeout=1.0,
                dsrdtr=True,
                rtscts=True
            ) as ser:
                print("✅ DEBUG: Connected for hardware reset")
                
                # Hardware reset sequence
                ser.dtr = False
                ser.rts = False
                time.sleep(0.1)
                
                ser.dtr = True
                ser.rts = True
                time.sleep(0.1)
                
                ser.dtr = False
                ser.rts = False
                time.sleep(0.1)
                
                print("✅ DEBUG: Hardware reset signal sent")
                time.sleep(3.0)  # Wait for boot
                
                # Clear boot messages
                ser.reset_input_buffer()
                
                return True
                
        except Exception as e:
            print(f"❌ DEBUG: Hardware reset failed: {e}")
            return False
    
    def attempt_recovery(self, error_type: str) -> bool:
        """
        Attempt recovery based on error type.
        
        Args:
            error_type: Type of error detected
            
        Returns:
            bool: True if recovery was attempted and might have succeeded
        """
        if not self.can_attempt_recovery():
            print(f"⚠️  DEBUG: Cannot attempt recovery (attempts: {self.recovery_attempts}/{self.max_recovery_attempts})")
            return False
            
        self.recovery_attempts += 1
        self.last_recovery_time = time.time()
        
        print(f"🔧 DEBUG: Attempting recovery for {error_type} (attempt {self.recovery_attempts}/{self.max_recovery_attempts})")
        
        recovery_success = False
        
        # CircuitPython crashes (red LED) - always try soft reset first
        if error_type == 'circuitpython_crash':
            print("🔄 DEBUG: CircuitPython crashed (red LED), performing soft reset")
            recovery_success = self.perform_circuitpython_soft_reset()
            
        # Hardware errors - try soft reset first, then hardware reset
        elif error_type in ['uart_error_7', 'bno055_error', 'i2c_error']:
            recovery_success = self.perform_circuitpython_soft_reset()
            if not recovery_success:
                recovery_success = self.perform_hardware_reset()
            
        # Connection issues - try hardware reset
        elif error_type == 'connection_lost':
            recovery_success = self.perform_hardware_reset()
            
        # For no_data errors, try soft reset first, then let main loop reconnect
        elif error_type == 'no_data':
            print("🔄 DEBUG: No data received, trying soft reset")
            recovery_success = self.perform_circuitpython_soft_reset()
            if not recovery_success:
                print("🔄 DEBUG: Soft reset failed, attempting reconnection")
                time.sleep(2)
                recovery_success = True  # Let the main loop handle reconnection
            
        if recovery_success:
            print(f"✅ DEBUG: Recovery attempt for {error_type} completed")
            # Reset counter on successful recovery
            self.recovery_attempts = 0
        else:
            print(f"❌ DEBUG: Recovery attempt for {error_type} failed")
            
        return recovery_success
    
    def reset_recovery_state(self):
        """Reset recovery state after successful data reception."""
        if self.recovery_attempts > 0:
            print("🎉 DEBUG: Data flow restored, resetting recovery state")
            self.recovery_attempts = 0
            
    def force_reset_recovery_state(self):
        """Force reset recovery state when stuck."""
        print("🔄 DEBUG: Force resetting recovery state")
        self.recovery_attempts = 0
        self.last_recovery_time = 0


def detect_and_select_port(default_port: str = '/dev/cu.usbmodem1101'):
    """
    Detect available serial ports and allow user to select one.
    
    Args:
        default_port: The default port to prefer
        
    Returns:
        str: Selected port path
    """
    print("🔍 Scanning for available serial ports...")
    ports = serial.tools.list_ports.comports()
    
    if not ports:
        print("❌ No serial ports found!")
        return default_port
    
    # Filter for likely IMU ports (USB modem ports)
    usb_modem_ports = [p for p in ports if 'usbmodem' in p.device]
    
    # Show all ports but highlight USB modem ports
    print("\n📡 Available serial ports:")
    print("=" * 60)
    
    port_options = []
    recommended_port = None
    
    # Show USB modem ports first (most likely to be IMU)
    if usb_modem_ports:
        print("🎯 RECOMMENDED (USB modem ports):")
        for i, port in enumerate(usb_modem_ports):
            port_options.append(port.device)
            print(f"  [{len(port_options)}] {port.device}")
            if port.description:
                print(f"      Description: {port.description}")
            if port.manufacturer:
                print(f"      Manufacturer: {port.manufacturer}")
            if port.serial_number:
                print(f"      Serial: {port.serial_number}")
            print()
            
            # Set first USB modem port as recommended
            if recommended_port is None:
                recommended_port = port.device
    
    # Show other ports
    other_ports = [p for p in ports if 'usbmodem' not in p.device]
    if other_ports:
        print("📱 Other ports:")
        for port in other_ports:
            port_options.append(port.device)
            print(f"  [{len(port_options)}] {port.device}")
            if port.description:
                print(f"      Description: {port.description}")
            print()
    
    print("=" * 60)
    
    # If we have a recommended port, use it
    if recommended_port:
        print(f"✅ Recommended port: {recommended_port}")
        try:
            response = input(f"Press Enter to use {recommended_port}, or enter port number [1-{len(port_options)}]: ").strip()
            
            if response == "":
                return recommended_port
            
            try:
                choice = int(response)
                if 1 <= choice <= len(port_options):
                    selected_port = port_options[choice - 1]
                    print(f"Selected port: {selected_port}")
                    return selected_port
                else:
                    print("❌ Invalid selection. Using recommended port.")
                    return recommended_port
            except ValueError:
                print("❌ Invalid input. Using recommended port.")
                return recommended_port
        except EOFError:
            print(f"Using recommended port: {recommended_port}")
            return recommended_port
    
    # No recommended port, ask user to choose
    if port_options:
        print("Please select a port:")
        try:
            choice = int(input(f"Enter port number [1-{len(port_options)}]: "))
            if 1 <= choice <= len(port_options):
                selected_port = port_options[choice - 1]
                print(f"Selected port: {selected_port}")
                return selected_port
            else:
                print("❌ Invalid selection. Using default port.")
                return default_port
        except (ValueError, EOFError):
            print("❌ Invalid input or no input. Using default port.")
            return default_port
    
    # Fallback to default
    print(f"Using default port: {default_port}")
    return default_port


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
        
        # Trail management for tip position (2 seconds)
        self.trail_points = []
        self.trail_timestamps = []
        self.trail_duration = 2.0  # seconds
        
        # Speed tracking with 3-second averaging
        self.speed_history = []  # (timestamp, speed_kmh)
        self.speed_history_duration = 3.0  # seconds
        self.last_speed_update = 0
        self.speed_update_interval = 1.0  # seconds
        self.current_display_speed = 0.0
        self.max_speed_kmh = 0.0
        self.max_acceleration_kmh_s = 0.0
        self.last_speed_for_accel = 0.0
        self.last_speed_time = 0.0
        
        # Reasonable caps for squash racket movement
        self.max_reasonable_speed_kmh = 200.0  # ~55 m/s, very fast but possible
        self.max_reasonable_accel_kmh_s = 2000.0  # ~556 m/s², high but reasonable
        
        # Color update tracking
        self.last_color_update = 0
        self.current_color = vp.color.blue
        
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
            
            # Store head center position for trail tracking (center of the ring)
            self.head_ring_center_local = vp.vector(0, 0, head_center_z)
            
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
            
            # Calculate head ring center position in world coordinates
            head_ring_center_world = rotation_matrix @ np.array([
                self.head_ring_center_local.x,
                self.head_ring_center_local.y, 
                self.head_ring_center_local.z
            ])
            
            # Compute instantaneous linear speed of head center over last two frames
            current_time = time.time()
            instantaneous_speed_mps = 0.0
            
            if hasattr(self, 'previous_head_pos') and hasattr(self, 'previous_time'):
                # Calculate distance moved
                distance = np.linalg.norm(head_ring_center_world - self.previous_head_pos)
                # Calculate time elapsed
                time_delta = current_time - self.previous_time
                
                if time_delta > 0:
                    instantaneous_speed_mps = distance / time_delta
                    instantaneous_speed_kmh = instantaneous_speed_mps * 3.6  # Convert m/s to km/h
                    
                    # Cap speed at reasonable maximum
                    if instantaneous_speed_kmh > self.max_reasonable_speed_kmh:
                        print(f"⚠️  DEBUG: Capped speed from {instantaneous_speed_kmh:.2f} to {self.max_reasonable_speed_kmh:.2f} km/h")
                        instantaneous_speed_kmh = self.max_reasonable_speed_kmh
                    
                    print(f"🔧 DEBUG: Instantaneous speed: {instantaneous_speed_kmh:.2f} km/h")
                    
                    # Add to speed history
                    self.speed_history.append((current_time, instantaneous_speed_kmh))
                    
                    # Calculate acceleration (km/h per second)
                    if self.last_speed_time > 0 and current_time > self.last_speed_time:
                        accel_time_delta = current_time - self.last_speed_time
                        accel_kmh_s = (instantaneous_speed_kmh - self.last_speed_for_accel) / accel_time_delta
                        
                        # Cap acceleration at reasonable maximum
                        if abs(accel_kmh_s) > self.max_reasonable_accel_kmh_s:
                            print(f"⚠️  DEBUG: Capped acceleration from {abs(accel_kmh_s):.2f} to {self.max_reasonable_accel_kmh_s:.2f} km/h/s")
                            accel_kmh_s = self.max_reasonable_accel_kmh_s if accel_kmh_s > 0 else -self.max_reasonable_accel_kmh_s
                        
                        if abs(accel_kmh_s) > self.max_acceleration_kmh_s:
                            self.max_acceleration_kmh_s = abs(accel_kmh_s)
                            print(f"🔧 DEBUG: New max acceleration: {self.max_acceleration_kmh_s:.2f} km/h/s")
                    
                    self.last_speed_for_accel = instantaneous_speed_kmh
                    self.last_speed_time = current_time
            
            # Store current position and time for next frame
            self.previous_head_pos = head_ring_center_world.copy()
            self.previous_time = current_time
            
            # Clean up old speed history
            self._clean_speed_history(current_time)
            
            # Update display speed and color every second based on 3-second average
            if current_time - self.last_speed_update >= self.speed_update_interval:
                self.current_display_speed = self._calculate_average_speed(current_time)
                
                # Update max speed
                if self.current_display_speed > self.max_speed_kmh:
                    self.max_speed_kmh = self.current_display_speed
                    print(f"🔧 DEBUG: New max speed: {self.max_speed_kmh:.2f} km/h")
                
                self.last_speed_update = current_time
                print(f"🔧 DEBUG: Updated display speed: {self.current_display_speed:.2f} km/h")
            
            # Update color every second based on current display speed
            if current_time - self.last_color_update >= self.speed_update_interval:
                self.current_color = self._speed_to_color_kmh(self.current_display_speed)
                self.last_color_update = current_time
                print(f"🔧 DEBUG: Updated color for speed {self.current_display_speed:.2f} km/h: {self.current_color}")
            
            # Apply current color to racket compound
            self.racket_compound.color = self.current_color
            
            # Add head ring center position to trail for drift visualization with timestamp
            head_ring_pos = vp.vector(head_ring_center_world[0], head_ring_center_world[1], head_ring_center_world[2])
            self.trail_points.append(head_ring_pos)
            self.trail_timestamps.append(current_time)
            
            # Clean up old trail points (keep only last 2 seconds)
            self._clean_trail_points(current_time)
            
            # Update trail
            self.trail.clear()
            for point in self.trail_points:
                self.trail.append(pos=point)
            
            print(f"✅ DEBUG: RacketScene update complete - {len(self.trail_points)} trail points, {self.current_display_speed:.2f} km/h")
            
        except Exception as e:
            print(f"❌ DEBUG: Error updating RacketScene: {e}")
            import traceback
            traceback.print_exc()
    
    def _clean_speed_history(self, current_time: float):
        """Remove speed history entries older than 3 seconds."""
        cutoff_time = current_time - self.speed_history_duration
        self.speed_history = [(t, s) for t, s in self.speed_history if t > cutoff_time]
    
    def _clean_trail_points(self, current_time: float):
        """Remove trail points older than 2 seconds."""
        cutoff_time = current_time - self.trail_duration
        
        # Find the first index to keep
        keep_from = 0
        for i, timestamp in enumerate(self.trail_timestamps):
            if timestamp > cutoff_time:
                keep_from = i
                break
        
        # Keep only recent points
        self.trail_points = self.trail_points[keep_from:]
        self.trail_timestamps = self.trail_timestamps[keep_from:]
    
    def _calculate_average_speed(self, current_time: float) -> float:
        """Calculate the maximum speed over the last 3 seconds."""
        if not self.speed_history:
            return 0.0
        
        # Get speeds from the last 3 seconds
        cutoff_time = current_time - self.speed_history_duration
        recent_speeds = [speed for timestamp, speed in self.speed_history if timestamp > cutoff_time]
        
        if not recent_speeds:
            return 0.0
        
        # Return the maximum speed in the recent history
        return max(recent_speeds)
    
    def _speed_to_color_kmh(self, speed_kmh: float) -> vp.vector:
        """Map speed in km/h to color: 0-18 km/h → blue, 18-36 km/h → gradient to yellow, >36 km/h → red."""
        if speed_kmh < 18.0:
            return vp.color.blue
        elif speed_kmh <= 36.0:
            # Gradient from blue to yellow to red
            ratio = (speed_kmh - 18.0) / 18.0  # 0.0 to 1.0
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
        
        # Initialize error recovery system
        self.error_recovery = BoardErrorRecovery(port, baud)
        self.last_valid_data_time = time.time()
        self.no_data_timeout = 10.0  # seconds - increased to reduce false positives
        self.consecutive_errors = 0
        self.max_consecutive_errors = 50  # Increased to reduce false recovery attempts
        self.error_check_interval = 5.0  # Only check for errors every 5 seconds
        self.last_error_check = time.time()
        self.no_data_frames = 0  # Track consecutive empty frames
        
        print(f"✅ DEBUG: Racket visualizer initialization complete")
        print(f"🔧 DEBUG: Error recovery system initialized")

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
            
            # Calculate racket head ring center direction (for reference and speed calculation)
            # Head ring is at Z = 0.15 + 0.30 = 0.45m in local coordinates
            head_ring_center_local = np.array([0, 0, 0.45])
            print(f"🔧 DEBUG: Base head ring center position: {head_ring_center_local}")
            
            rotated_head_center = rotation_matrix @ head_ring_center_local
            print(f"🔧 DEBUG: Rotated head ring center: {rotated_head_center}")
            
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
                
                # Show racket head ring center position for drift checking
                info += f"RACKET HEAD RING CENTER:\n"
                info += f"  x: {rotated_head_center[0]:6.3f}\n"
                info += f"  y: {rotated_head_center[1]:6.3f}\n"
                info += f"  z: {rotated_head_center[2]:6.3f}\n"
                
                # Get speed from scene (calculated internally)
                scene_speed = getattr(self.scene, 'current_display_speed', 0.0)
                max_speed = getattr(self.scene, 'max_speed_kmh', 0.0)
                max_accel = getattr(self.scene, 'max_acceleration_kmh_s', 0.0)
                
                info += f"Speed: {scene_speed:6.1f} km/h\n"
                info += f"Max Speed: {max_speed:6.1f} km/h\n"
                info += f"Max Accel: {max_accel:6.1f} km/h/s\n"
                
                # Speed color indicator
                if scene_speed < 18.0:
                    speed_color = "BLUE (slow)"
                elif scene_speed <= 36.0:
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
                        self.no_data_frames = 0  # Reset no data counter
                        
                        # Only show frame info occasionally to reduce spam
                        if self.frames_received % 100 == 0:
                            print(f"📦 DEBUG: Frame #{self.frames_received} (ts: {frame.get('ts', 'Unknown')})")
                        
                        # Check for errors in frame data, but only periodically
                        should_check_errors = (current_time - self.last_error_check) > self.error_check_interval
                        
                        if should_check_errors:
                            self.last_error_check = current_time
                            raw_data = frame.get('raw_data', '')
                            error_type, needs_recovery = self.error_recovery.detect_error(raw_data)
                            
                            if needs_recovery:
                                print(f"🚨 ERROR DETECTED: {error_type}")
                                # Immediate recovery for critical errors
                                if error_type in ['circuitpython_crash', 'uart_error_7', 'bno055_error']:
                                    print("🔄 Attempting immediate recovery for critical error")
                                    if self.error_recovery.attempt_recovery(error_type):
                                        print("✅ Recovery successful, continuing...")
                                    else:
                                        print("❌ Recovery failed, but continuing...")
                                else:
                                    self.consecutive_errors += 1
                                    
                                    if self.consecutive_errors >= self.max_consecutive_errors:
                                        print(f"⚠️ Too many consecutive errors ({self.consecutive_errors}), attempting recovery")
                                        if self.error_recovery.attempt_recovery(error_type):
                                            print("✅ Recovery completed")
                                            self.consecutive_errors = 0
                            else:
                                # Valid data received, reset error counters
                                if self.consecutive_errors > 0:
                                    print(f"✅ Valid data restored, resetting error counters")
                                    self.consecutive_errors = 0
                                    self.error_recovery.reset_recovery_state()
                        
                        self.last_valid_data_time = current_time
                        self.update_racket_from_frame(frame)
                        self.last_update = current_time
                    else:
                        # No frame received
                        self.no_data_frames += 1
                        
                        # Check for no data timeout (but less frequently)
                        if current_time - self.last_valid_data_time > self.no_data_timeout:
                            print(f"⚠️ No data timeout ({self.no_data_timeout}s), attempting recovery")
                            
                            # If we've been stuck for too long, force reset recovery state
                            if current_time - self.last_valid_data_time > self.no_data_timeout * 3:
                                print("🔄 Been stuck too long, force resetting recovery state")
                                self.error_recovery.force_reset_recovery_state()
                            
                            if self.error_recovery.attempt_recovery('no_data'):
                                print("✅ No data recovery completed")
                                self.last_valid_data_time = current_time  # Reset timeout
                            else:
                                print("❌ No data recovery failed, will retry in 5s")
                                time.sleep(5)
                        elif loop_count % 5000 == 0:  # Log every 5000 empty reads (less frequent)
                            print(f"⏳ No frame received (loop {loop_count}, no_data_frames: {self.no_data_frames})")
                    
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
            
            # Try to recover from connection errors
            if "Serial port" in str(e) or "Connection" in str(e) or "UART" in str(e):
                print("🚨 DEBUG: Connection error detected, attempting recovery")
                if self.error_recovery.attempt_recovery('connection_lost'):
                    print("✅ DEBUG: Connection recovery completed, restarting...")
                    time.sleep(2)
                    return self.run()  # Restart the run method
                else:
                    print("❌ DEBUG: Connection recovery failed")
            
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
        default=None,
        help='Serial port for IMU (if not specified, will auto-detect and prompt)'
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
    
    # Auto-detect port if not specified
    if args.port is None:
        selected_port = detect_and_select_port()
    else:
        selected_port = args.port
    
    print(f"Port: {selected_port}")
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
        if not reset_arduino(selected_port, args.baud):
            print("❌ DEBUG: Reset failed. You may need to manually reconnect USB.")
            print("Try running: python reset_arduino.py --port", selected_port)
            return
        print("✅ DEBUG: Reset successful!\n")
    
    # Create and run visualizer
    print("🔧 DEBUG: Creating racket visualizer...")
    visualizer = DebugRacketVisualizer(selected_port, args.baud, args.length, args.head_radius)
    print("🚀 DEBUG: Starting visualization...")
    visualizer.run()


if __name__ == "__main__":
    main() 