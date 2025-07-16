#!/usr/bin/env python3
"""
Clean IMU Orientation Visualizer
Uses the stable simple_serial_reader.py for reliable data
"""

import vpython as vp
import time
import argparse
import signal
import sys
from simple_serial_reader import SimpleSerialReader

class CleanOrientationVisualizer:
    def __init__(self, arrow_length=0.7):
        self.arrow_length = arrow_length
        self.running = False
        self.reader = None
        
        # Set up 3D scene
        self.setup_scene()
        
        # Create 3D objects
        self.setup_objects()
        
        # Track for drift detection
        self.initial_heading = None
        self.frame_count = 0
        self.last_data_time = time.time()
        
    def setup_scene(self):
        """Initialize the 3D visualization scene"""
        vp.scene.title = "Clean IMU Orientation Visualizer"
        vp.scene.width = 800
        vp.scene.height = 600
        vp.scene.background = vp.color.black
        vp.scene.forward = vp.vector(-1, -1, -1)
        vp.scene.up = vp.vector(0, 0, 1)
        
    def setup_objects(self):
        """Create 3D visualization objects"""
        # Main orientation arrow (red)
        self.arrow = vp.arrow(
            pos=vp.vector(0, 0, 0),
            axis=vp.vector(1, 0, 0),
            length=self.arrow_length,
            shaftwidth=0.05,
            color=vp.color.red,
            opacity=0.9
        )
        
        # Reference axes (dim)
        vp.arrow(pos=vp.vector(0, 0, 0), axis=vp.vector(0.3, 0, 0), 
                color=vp.color.gray(0.3), shaftwidth=0.01)  # X-axis
        vp.arrow(pos=vp.vector(0, 0, 0), axis=vp.vector(0, 0.3, 0), 
                color=vp.color.gray(0.3), shaftwidth=0.01)  # Y-axis  
        vp.arrow(pos=vp.vector(0, 0, 0), axis=vp.vector(0, 0, 0.3), 
                color=vp.color.gray(0.3), shaftwidth=0.01)  # Z-axis
        
        # Trail for drift detection (yellow points)
        self.trail_points = []
        self.max_trail_points = 100  # Reduced for better performance
        
        # Info display
        self.info_text = vp.wtext(text="Initializing...\n")
        
    def euler_to_vector(self, heading, roll, pitch):
        """Convert Euler angles to 3D direction vector"""
        import math
        
        # Convert degrees to radians
        h = math.radians(heading)
        r = math.radians(roll) 
        p = math.radians(pitch)
        
        # Convert to direction vector (NED convention)
        x = math.cos(p) * math.sin(h)
        y = math.cos(p) * math.cos(h)
        z = -math.sin(p)
        
        return vp.vector(x, y, z)
        
    def update_visualization(self, data):
        """Update the 3D visualization with new orientation data"""
        heading = data['heading']
        roll = data['roll']
        pitch = data['pitch']
        
        # Track initial heading for drift detection
        if self.initial_heading is None:
            self.initial_heading = heading
            
        # Calculate heading drift
        heading_drift = heading - self.initial_heading
        if heading_drift > 180:
            heading_drift -= 360
        elif heading_drift < -180:
            heading_drift += 360
            
        # Update arrow orientation
        direction = self.euler_to_vector(heading, roll, pitch)
        self.arrow.axis = direction * self.arrow_length
        
        # Add trail point for drift tracking (less frequently for performance)
        if self.frame_count % 3 == 0:  # Only add every 3rd frame
            trail_pos = direction * (self.arrow_length * 1.1)
            if len(self.trail_points) >= self.max_trail_points:
                # Remove oldest point
                old_point = self.trail_points.pop(0)
                old_point.visible = False
                
            # Add new trail point
            trail_point = vp.sphere(
                pos=trail_pos,
                radius=0.008,
                color=vp.color.yellow,
                opacity=0.6
            )
            self.trail_points.append(trail_point)
        
        # Update info display
        self.frame_count += 1
        current_time = time.time()
        data_rate = 1.0 / (current_time - self.last_data_time) if self.last_data_time else 0
        self.last_data_time = current_time
        
        self.info_text.text = (
            f"🎯 CLEAN LIVE ARROW (Using SimpleSerialReader)\n"
            f"🧭 ORIENTATION:\n"
            f"Heading: {heading:.1f}°\n" 
            f"Roll: {roll:.1f}°\n"
            f"Pitch: {pitch:.1f}°\n\n"
            f"📊 PERFORMANCE:\n"
            f"Data Rate: {data_rate:.1f} Hz\n"
            f"Frames: {self.frame_count}\n"
            f"Trail Points: {len(self.trail_points)}\n\n"
            f"📈 DRIFT TRACKING:\n"
            f"Heading Drift: {heading_drift:.1f}°\n"
            f"⚠️  Watch yellow trail for drift patterns\n"
        )
        
    def start(self, port='/dev/cu.usbmodem1101'):
        """Start the visualization"""
        print("\n" + "="*50)
        print("🎯 Clean IMU Orientation Visualizer")
        print("="*50)
        print("✅ USING: SimpleSerialReader (stable)")
        print("❌ NOT USING: serial_reader.py (problematic)")
        print("="*50)
        print("PURPOSE: Real-time orientation drift detection")
        print("DISPLAY: Red arrow shows orientation")
        print("TRAIL: Yellow dots track movement patterns") 
        print("DRIFT: Watch for spiral patterns in trail")
        print("="*50)
        print(f"Port: {port}")
        print(f"Arrow length: {self.arrow_length}m")
        print("Press Ctrl+C to stop")
        print("="*50)
        
        # Start serial reader
        print("🔄 Starting SimpleSerialReader...")
        self.reader = SimpleSerialReader(port=port, baud=115200)
        if not self.reader.start():
            print("❌ Failed to start SimpleSerialReader")
            return False
            
        print("✅ SimpleSerialReader started successfully")
        print("🚀 Starting 3D visualization...")
        
        self.running = True
        
        # Main visualization loop
        try:
            while self.running:
                # Get latest data
                data = self.reader.get_data()
                if data:
                    self.update_visualization(data)
                else:
                    # No data available, small delay
                    time.sleep(0.01)
                
                # Maintain higher FPS for smoother visualization
                vp.rate(120)
                
        except KeyboardInterrupt:
            print("\n🛑 Stopping visualization...")
        finally:
            self.stop()
            
        return True
        
    def stop(self):
        """Stop the visualization"""
        self.running = False
        if self.reader:
            self.reader.stop()
        print("✅ Clean visualization stopped")

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print("\n🛑 Interrupt received, stopping...")
    sys.exit(0)

def main():
    parser = argparse.ArgumentParser(description='Clean IMU Orientation Visualizer')
    parser.add_argument('--port', default='/dev/cu.usbmodem1101', 
                       help='Serial port (default: /dev/cu.usbmodem1101)')
    parser.add_argument('--arrow-length', type=float, default=0.7,
                       help='Arrow length in meters (default: 0.7)')
    
    args = parser.parse_args()
    
    # Handle Ctrl+C gracefully
    signal.signal(signal.SIGINT, signal_handler)
    
    # Create and start visualizer
    visualizer = CleanOrientationVisualizer(arrow_length=args.arrow_length)
    
    try:
        visualizer.start(port=args.port)
    except Exception as e:
        print(f"❌ Error: {e}")
        return 1
        
    return 0

if __name__ == "__main__":
    sys.exit(main()) 