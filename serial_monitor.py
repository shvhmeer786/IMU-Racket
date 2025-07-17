#!/usr/bin/env python3
"""
Simple Serial Monitor
====================
Monitor raw serial data from the IMU board to diagnose issues.
"""

import serial
import time
import sys
import serial.tools.list_ports

def detect_and_select_port(default_port: str = '/dev/cu.usbmodem1101'):
    """
    Detect available serial ports and allow user to select one.
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
        for port in usb_modem_ports:
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
    
    # Fallback to default
    print(f"Using default port: {default_port}")
    return default_port

def monitor_serial(port: str, baud: int = 115200):
    """
    Monitor serial port and display raw data.
    """
    print(f"\n🔍 Serial Monitor Starting")
    print(f"Port: {port}")
    print(f"Baud: {baud}")
    print("=" * 60)
    print("📡 Raw serial data (Press Ctrl+C to stop):")
    print("=" * 60)
    
    try:
        with serial.Serial(port, baud, timeout=1) as ser:
            print("✅ Connected to serial port")
            
            line_count = 0
            while True:
                try:
                    if ser.in_waiting > 0:
                        # Read raw bytes
                        raw_data = ser.readline()
                        
                        # Try to decode as UTF-8
                        try:
                            line = raw_data.decode('utf-8', errors='ignore').strip()
                            if line:
                                line_count += 1
                                timestamp = time.strftime("%H:%M:%S")
                                print(f"[{timestamp}] #{line_count:04d}: {line}")
                        except Exception as decode_error:
                            print(f"[DECODE ERROR] {raw_data}: {decode_error}")
                    
                    time.sleep(0.01)  # Small delay to prevent CPU spinning
                    
                except KeyboardInterrupt:
                    print("\n🛑 Stopping serial monitor...")
                    break
                except Exception as e:
                    print(f"❌ Error reading serial data: {e}")
                    time.sleep(1)
                    
    except Exception as e:
        print(f"❌ Failed to connect to {port}: {e}")
        print("Check that the port is correct and the device is connected.")

def main():
    """Main function."""
    print("🔧 Serial Monitor for IMU Board Diagnostics")
    print("=" * 50)
    
    # Detect and select port
    selected_port = detect_and_select_port()
    
    # Default baud rate
    baud = 115200
    
    # Allow user to change baud rate
    try:
        baud_input = input(f"Enter baud rate (default: {baud}): ").strip()
        if baud_input:
            baud = int(baud_input)
    except (ValueError, EOFError):
        print(f"Using default baud rate: {baud}")
    
    # Start monitoring
    monitor_serial(selected_port, baud)

if __name__ == "__main__":
    main()