#!/usr/bin/env python3
"""Basic ESP32 Remote Test (Standalone, No ROS 2).

This tests the ESP32 remote serial output parsing without requiring ROS 2 or turtlesim.

Usage:
    pip install pyserial
    python test_remote_basic.py <serial_port>

Or just run:
    python test_remote_basic.py /dev/cu.usbserial-0001
"""

import sys
import re
import time
from datetime import datetime

def parse_remote_line(line):
    """Parse firmware output line into structured data."""
    if not line.strip():
        return None
    
    result = {}
    
    # Match patterns like: state=RUN, x=-0.5, y=-0.5, btn=1, robot=1, pi=0, failsafe=0, manual=1
    for match in re.finditer(r'(\w+)=([+-]?\d+(?:\.\d+)?)', line):
        key, val = match.groups()
        if '.' in val:
            result[key] = float(val)
        else:
            result[key] = int(val)
    
    return result

def test_remote_serial(port='/dev/cu.usbserial-0001'):
    """Test ESP32 remote by reading serial output."""
    import serial
    
    print(f"Connecting to ESP32 remote at {port}...")
    
    try:
        ser = serial.Serial(port, 115200, timeout=3)
        time.sleep(1)  # Wait for firmware to initialize
        print("Connected! Testing remote...")
        print()
    except serial.SerialException as e:
        print(f"Failed to connect to {port}: {e}")
        print("\nPossible solutions:")
        print("  - Check if ESP32 is powered (USB power bank)")
        print("  - Try a different USB port")
        print("  - Find your actual port: ls /dev/tty.usbserial*")
        return
    
    # Display header
    print("=" * 70)
    print(f"ESP32 Remote Serial Test")
    print(f"Port: {port}")
    print(f"Baud: 115200")
    print()
    print("Joystick test:")
    print("  Push stick away from you → x should go to +1.00")
    print("  Push stick left → y should go to +1.00")
    print("  Click button → btn becomes 1")
    print()
    print("Button hold test (stick centered):")
    print("  Press and HOLD button for 1.5 seconds with stick at rest")
    print("  → state changes from STOP to RUN")
    print()
    print("-" * 70)
    print()
    
    # Main loop - display output
    last_normal = time.time()
    
    try:
        while True:
            line = ser.readline().decode('utf-8', errors='replace').strip()
            
            if not line:
                continue
            
            data = parse_remote_line(line)
            
            # Skip garbage / initialization noise
            if 'state' not in data or len(data) < 2:
                continue
            
            # Format output nicely
            now = time.strftime('%H:%M:%S')
            state = data.get('state', 'STOP').upper()
            x = f"{data.get('x', 0):+.2f}" if data.get('x') is not None else "N/A"
            y = f"{data.get('y', 0):+.2f}" if data.get('y') is not None else "N/A"
            btn = data.get('btn', 0)
            manual = data.get('manual', 0)
            
            print(f"[{now}] state={state:5s} x={x:6s} y={y:6s} btn={btn:3d} manual={manual}")
            
            # Highlight interesting events
            if state == 'RUN':
                print("       ↑ RUN MODE! (LED solid on remote)")
            elif state == 'STOP':
                print("       ↑ STOP mode")
            
            if manual:
                print("       ↑ MANUAL OVERRIDE active - joystick controls robot!")
            
            if btn == 1 and state == 'RUN':
                print("       ↑ Button pressed while running - preparing to stop...")
            
            # Track last normal output
            if data.get('state') in ['STOP', 'RUN'] and len(data) >= 2:
                last_normal = time.time()
    
    except KeyboardInterrupt:
        print("\n" + "-" * 70)
        print("Test ended by user. Press Ctrl+C again to close serial port.")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        ser.close()
        print("Serial port closed.")

def test_simulated_remote():
    """Simulate ESP-NOW output without hardware."""
    import time
    
    print("\n" + "=" * 70)
    print("Simulated Remote Output Test")
    print("=" * 70)
    print()
    
    # Simulate startup (STOP mode, stick centered)
    print("POWER UP: Remote initializing...")
    for _ in range(5):
        t = time.strftime('%H:%M:%S')
        print(f"[{t}] state=STOP   x={-0.02:+.2f} y={-0.01:+.2f} btn=0  manual=0")
        time.sleep(0.5)
    
    print()
    print("Testing joystick... (press any key to advance)")
    
    try:
        while True:
            import sys
            line = input("Command? ").strip()
            
            if not line:
                continue
            
            t = time.strftime('%H:%M:%S')
            
            if line in ['forward', 'fwd', 'FORWARD']:
                print(f"[{t}] state=STOP   x={-0.50:+.2f} y={-1.00:+.2f} btn=0  manual=0")
            elif line in ['backward', 'back', 'BACKWARD']:
                print(f"[{t}] state=STOP   x={-0.50:+.2f} y={+1.00:+.2f} btn=0  manual=0")
            elif line in ['left', 'l', 'LEFT']:
                print(f"[{t}] state=STOP   x={-0.50:+.2f} y={-0.50:+.2f} btn=0  manual=0")
            elif line in ['right', 'r', 'RIGHT']:
                print(f"[{t}] state=STOP   x={+0.50:+.2f} y={-0.50:+.2f} btn=0  manual=0")
            elif line in ['stop', 's', 'STOP']:
                print(f"[{t}] state=STOP   x={0.00:+.2f} y={0.00:+.2f} btn=1  manual=0")
            elif line in ['hold', 'h', 'HOLD']:
                print(f"[{t}] state=RUN    x={0.00:+.2f} y={0.00:+.2f} btn=0  manual=1")
                print("         ↑ RUN mode engaged! (press STOP or move stick to test manual)")
            else:
                # Default centered position
                print(f"[{t}] state=STOP   x={-0.01:+.2f} y={-0.01:+.2f} btn=0  manual=0")
    
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    import argparse
    
    parser = argparse.ArgumentParser(
        description='Test ESP32 remote serial output'
    )
    parser.add_argument('port', nargs='?', default='/dev/cu.usbserial-0001',
                       help='Serial port (e.g., /dev/cu.usbserial-0001)')
    parser.add_argument('--sim', action='store_true',
                       help='Run simulation without hardware')
    
    args = parser.parse_args()
    
    if args.sim:
        test_simulated_remote()
    else:
        test_remote_serial(args.port)
