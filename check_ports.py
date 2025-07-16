#!/usr/bin/env python3
"""
Utility to check available serial ports and verify your IMU port.
"""

import serial.tools.list_ports
import os

def main():
    target_port = "/dev/cu.usbmodem1101"
    
    print("Available serial ports:")
    print("=" * 50)
    
    ports = serial.tools.list_ports.comports()
    found_target = False
    
    for port in ports:
        is_target = port.device == target_port
        if is_target:
            found_target = True
            
        print(f"{'→ ' if is_target else '  '}{port.device}")
        if port.description:
            print(f"    Description: {port.description}")
        if port.manufacturer:
            print(f"    Manufacturer: {port.manufacturer}")
        if port.serial_number:
            print(f"    Serial: {port.serial_number}")
        print()
    
    print("=" * 50)
    
    if found_target:
        print(f"✅ Target port {target_port} is available!")
        print("You can run: python test_imu_reader.py")
    else:
        print(f"❌ Target port {target_port} not found")
        print("\nTroubleshooting:")
        print("1. Make sure your IMU device is connected")
        print("2. Check USB cable connection")
        print("3. Try unplugging and reconnecting the device")
        print("4. Look for similar port names like /dev/cu.usbmodemXXXX")
        
        # Show macOS-style USB modem ports
        usb_modem_ports = [p for p in ports if 'usbmodem' in p.device]
        if usb_modem_ports:
            print(f"\nFound {len(usb_modem_ports)} USB modem port(s):")
            for port in usb_modem_ports:
                print(f"  {port.device}")
    
    print(f"\nNote: On macOS, you might also see corresponding /dev/tty.usbmodemXXXX ports.")
    print("Use the /dev/cu.* version for serial communication.")

if __name__ == "__main__":
    main() 