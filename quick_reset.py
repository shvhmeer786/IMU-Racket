#!/usr/bin/env python3
"""
Quick Arduino Recovery Script
=============================
Use this when your Arduino crashes (red LED, no data output).
Much faster than unplugging/replugging USB cable.
"""

import serial
import time
import sys

def reset_arduino(port='/dev/cu.usbmodem1101', baud=115200):
    """Reset Arduino using CircuitPython soft reset commands."""
    print(f"🔄 Resetting Arduino on {port}...")
    
    try:
        # Connect to Arduino
        ser = serial.Serial(port, baud, timeout=2)
        print("✅ Connected to Arduino")
        
        # Send reset commands
        ser.write(b'\x03')  # Ctrl+C (stop current code)
        time.sleep(1)
        ser.write(b'\x04')  # Ctrl+D (soft reboot)
        time.sleep(3)
        
        print("✅ Reset commands sent")
        
        # Check for recovery
        print("🔍 Checking for recovery...")
        recovered = False
        for i in range(10):
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                print(f"📥 {line}")
                if 'Orientation:' in line or 'BNO055' in line:
                    recovered = True
            time.sleep(0.5)
        
        ser.close()
        
        if recovered:
            print("🎉 Arduino recovered successfully!")
            print("You can now run your visualization script.")
        else:
            print("⚠️  Arduino reset sent, but no data detected yet.")
            print("Try running your script - it may take a moment to start.")
            
        return True
        
    except Exception as e:
        print(f"❌ Reset failed: {e}")
        print("Try manually unplugging and replugging the USB cable.")
        return False

if __name__ == "__main__":
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        port = '/dev/cu.usbmodem1101'
    
    reset_arduino(port) 