#!/usr/bin/env python3
"""
Setup script to help switch from unstable CircuitPython to stable C++ Arduino sketch.
"""

import os
import sys
import subprocess
import platform

def check_arduino_ide():
    """Check if Arduino IDE is installed."""
    print("🔍 Checking for Arduino IDE...")
    
    system = platform.system()
    if system == "Darwin":  # macOS
        # Check for Arduino IDE in Applications
        arduino_paths = [
            "/Applications/Arduino.app/Contents/MacOS/Arduino",
            "/Applications/Arduino IDE.app/Contents/MacOS/Arduino IDE"
        ]
        
        for path in arduino_paths:
            if os.path.exists(path):
                print(f"✅ Found Arduino IDE at: {path}")
                return path
    
    elif system == "Linux":
        # Check common Linux locations
        try:
            result = subprocess.run(['which', 'arduino'], capture_output=True, text=True)
            if result.returncode == 0:
                path = result.stdout.strip()
                print(f"✅ Found Arduino IDE at: {path}")
                return path
        except:
            pass
    
    elif system == "Windows":
        # Check common Windows locations
        arduino_paths = [
            "C:\\Program Files (x86)\\Arduino\\arduino.exe",
            "C:\\Program Files\\Arduino\\arduino.exe"
        ]
        
        for path in arduino_paths:
            if os.path.exists(path):
                print(f"✅ Found Arduino IDE at: {path}")
                return path
    
    print("❌ Arduino IDE not found")
    return None

def print_instructions():
    """Print detailed setup instructions."""
    print("\n" + "="*60)
    print("📋 SETUP INSTRUCTIONS: Switch to Stable C++ IMU")
    print("="*60)
    
    print("\n🔧 STEP 1: Install Arduino IDE (if not installed)")
    print("   Download from: https://www.arduino.cc/en/software")
    
    print("\n📋 STEP 2: Set up Arduino IDE for Nano RP2040 Connect")
    print("   1. Open Arduino IDE")
    print("   2. Go to File → Preferences")
    print("   3. In 'Additional Boards Manager URLs', add:")
    print("      https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json")
    print("   4. Go to Tools → Board → Boards Manager")
    print("   5. Search for 'pico' and install 'Raspberry Pi Pico/RP2040'")
    print("   6. Select Board: Tools → Board → Raspberry Pi Pico/RP2040 → Arduino Nano RP2040 Connect")
    
    print("\n📁 STEP 3: Upload the stable sketch")
    print("   1. Connect your Arduino")
    print("   2. Open the file: stable_test_imu.ino")
    print("   3. Select the correct port in Tools → Port")
    print("   4. Click Upload (➡️) button")
    
    print("\n✅ STEP 4: Test the connection")
    print("   After upload, run:")
    print("   python live_arrow.py --port /dev/cu.usbmodem1101 --baud 115200")
    
    print("\n💡 WHY THIS FIXES THE ISSUE:")
    print("   • CircuitPython has inherent serial stability issues")
    print("   • C++ Arduino sketches have rock-solid serial communication")
    print("   • No more 'Device not configured' errors")
    print("   • No more random disconnections")
    print("   • Consistent data streaming")

def check_current_setup():
    """Check what's currently on the Arduino."""
    print("\n🔍 Checking current Arduino setup...")
    
    port = "/dev/cu.usbmodem1101"
    
    # Check if port exists
    if not os.path.exists(port):
        print(f"❌ Arduino not found at {port}")
        print("   Try checking available ports with: python check_ports.py")
        return False
    
    print(f"✅ Arduino found at {port}")
    
    # Try to connect and see what's running
    try:
        import serial
        import time
        
        print("   Connecting to check current firmware...")
        with serial.Serial(port, 115200, timeout=2) as ser:
            time.sleep(1)
            
            # Read a few lines to see what's running
            lines = []
            start_time = time.time()
            while len(lines) < 5 and time.time() - start_time < 3:
                if ser.in_waiting > 0:
                    try:
                        line = ser.readline().decode('utf-8', errors='ignore').strip()
                        if line:
                            lines.append(line)
                    except:
                        pass
                time.sleep(0.1)
            
            if lines:
                print("   Current output:")
                for i, line in enumerate(lines[:3]):
                    print(f"     {line}")
                if len(lines) > 3:
                    print(f"     ... ({len(lines)} total lines)")
                
                # Detect what's running
                output_text = ' '.join(lines)
                if "CircuitPython" in output_text or "Adafruit" in output_text:
                    print("   📍 Detected: CircuitPython (unstable)")
                    print("   🎯 Recommendation: Switch to C++ sketch for stability")
                elif "Stable Test IMU" in output_text:
                    print("   📍 Detected: C++ sketch (stable)")
                    print("   ✅ Already running stable firmware!")
                elif "Orientation:" in output_text:
                    print("   📍 Detected: Some IMU code (unknown stability)")
                else:
                    print("   📍 Detected: Unknown firmware")
            else:
                print("   ❌ No output detected")
                print("   Arduino may not be programmed or connection failed")
                
    except ImportError:
        print("   ⚠️  pyserial not available for testing")
    except Exception as e:
        print(f"   ❌ Connection test failed: {e}")
    
    return True

def main():
    """Main function."""
    print("🚀 Arduino Nano RP2040 Connect - Stable IMU Setup")
    print("="*60)
    print("This script helps you switch from unstable CircuitPython")
    print("to a stable C++ Arduino sketch for reliable IMU data.")
    print("="*60)
    
    # Check current setup
    check_current_setup()
    
    # Check for Arduino IDE
    arduino_ide = check_arduino_ide()
    
    # Print instructions
    print_instructions()
    
    if not arduino_ide:
        print("\n⚠️  Arduino IDE not found. Please install it first.")
    
    print("\n📂 FILES CREATED:")
    print("   stable_test_imu.ino - C++ Arduino sketch (RECOMMENDED)")
    print("   This file should be opened in Arduino IDE")
    
    print("\n🔄 NEXT STEPS:")
    if arduino_ide:
        print("   1. Arduino IDE is installed ✅")
        print("   2. Follow the setup instructions above")
        print("   3. Upload stable_test_imu.ino")
    else:
        print("   1. Install Arduino IDE first")
        print("   2. Follow the setup instructions above")
    
    print("\nAfter uploading the C++ sketch, your connection should be stable!")

if __name__ == "__main__":
    main() 