#!/usr/bin/env python3
"""
Arduino Reset Utility

Resets the Arduino to clear any stuck CircuitPython states
that prevent serial communication.
"""

import serial
import time
import argparse

def reset_arduino(port: str, baud: int = 115200):
    """Reset Arduino via DTR/RTS signals."""
    print(f"Resetting Arduino on {port}...")
    
    try:
        # Open serial connection
        with serial.Serial(
            port=port,
            baudrate=baud,
            timeout=1.0,
            dsrdtr=True,  # Enable DTR/DSR
            rtscts=True   # Enable RTS/CTS
        ) as ser:
            
            print("Sending reset signal...")
            
            # Reset sequence
            ser.dtr = False
            ser.rts = False
            time.sleep(0.1)
            
            ser.dtr = True
            ser.rts = True
            time.sleep(0.1)
            
            ser.dtr = False
            ser.rts = False
            time.sleep(0.1)
            
            print("Reset signal sent. Waiting for Arduino to boot...")
            time.sleep(3.0)  # Wait for CircuitPython to boot
            
            # Clear any boot messages
            ser.reset_input_buffer()
            
            print("✅ Arduino reset complete!")
            
    except Exception as e:
        print(f"❌ Reset failed: {e}")
        print("You may need to manually unplug/replug the USB cable")
        return False
    
    return True

def main():
    """Main function with command line argument parsing."""
    parser = argparse.ArgumentParser(description="Reset Arduino for serial communication")
    
    parser.add_argument(
        '--port',
        type=str,
        default='/dev/cu.usbmodem1101',
        help='Serial port for Arduino (default: /dev/cu.usbmodem1101)'
    )
    
    parser.add_argument(
        '--baud',
        type=int,
        default=115200,
        help='Baud rate (default: 115200)'
    )
    
    args = parser.parse_args()
    
    print("Arduino Reset Utility")
    print("=" * 30)
    print(f"Port: {args.port}")
    print(f"Baud: {args.baud}")
    print("=" * 30)
    
    success = reset_arduino(args.port, args.baud)
    
    if success:
        print("\n🎉 Arduino is ready for serial communication!")
        print("You can now run:")
        print(f"python live_arrow.py --port {args.port}")
    else:
        print("\n💡 If reset failed, try:")
        print("1. Unplug USB cable")
        print("2. Wait 2 seconds") 
        print("3. Plug USB cable back in")
        print("4. Wait 3 seconds")
        print("5. Run your script")

if __name__ == "__main__":
    main() 