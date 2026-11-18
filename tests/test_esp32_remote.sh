#!/bin/bash
# Test ESP32 remote with turtlesim (no physical robot needed)
# Run this on your Raspberry Pi

set -e

echo "=== ESP32 Remote Turtlesim Test ==="
echo ""
echo "This tests the ESP32 remote firmware using turtlesim instead of the physical robot."
echo ""

# Step 1: Check ROS 2 is available
if command -v ros2 &> /dev/null; then
    echo "✓ ROS 2 found"
else
    echo "✗ ROS 2 not found. Run: sudo apt install ros-jazzy-desktop"
    exit 1
fi

# Step 2: Check if you have ESP32 boards connected
if arduino-cli board list 2>&1 | grep -q "cu.usbserial"; then
    echo "✓ ESP32 board detected on Mac"
    
    # Find the port
    SERIAL_PORT=$(arduino-cli board list 2>/dev/null | grep "cu.usbserial" | head -1 | awk '{print $5}')
    echo "  Port: $SERIAL_PORT"
else
    echo "⚠ ESP32 not detected. Make sure both boards are plugged into your Mac."
fi

# Step 3: Flash the remote (on Mac via SSH or local)
echo ""
echo "=== Step 3: Flash Remote ==="
echo "Run on your Mac:"
echo 'arduino-cli compile -f --fqbn esp32:esp32:esp32 /Users/josephvillavicencio/skatefollow/firmware/esp32_remote'
echo 'arduino-cli upload -p <your_port> --fqbn esp32:esp32:esp32 /Users/josephvillavicencio/skatefollow/firmware/esp32_remote'
echo ""

# Step 4: Create the test Python script
cat > ~/test_esp32_turtle.py << 'PYEOF'
#!/usr/bin/env python3
"""ESP32 Remote Serial Bridge for Turtlesim.

Reads ESP-NOW remote output from serial and publishes to /turtle1/cmd_vel.
Run this on your Raspberry Pi.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import re

class RemoteSerialBridge(Node):
    """Bridge ESP32 serial output to ROS 2 turtle commands."""
    
    def __init__(self):
        super().__init__('esp32_turtle_bridge')
        
        # Parameters (can be overridden at launch)
        self.declare_parameter('serial_port', '/dev/tty.usbserial-0001')  # Update this!
        self.serial_port = self.get_parameter('serial_port').value
        
        # Joystick scaling (m/s per full stick deflection)
        self.bpf = self.declare_parameter('bpf_factor', 0.25).value
        
        self.get_logger().info(f"Serial port: {self.serial_port}")
        self.get_logger().info(f"BPF factor: {self.bpf} m/s at full stick")
        
        # Setup serial
        try:
            self.ser = serial.Serial(self.serial_port, 115200, timeout=1)
            self.get_logger().info("Serial port opened successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial: {e}")
            raise
        
        # Setup publisher
        self.cmd_pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        self.state = 'STOP'
        self.manual = False
        self.x = 0.0
        self.y = 0.0
    
    def parse_remote_line(self, line):
        """Parse firmware output: state=RUN x=-0.5 y=-0.5 btn=1 robot=1 pi=0..."""
        result = {}
        
        for match in re.finditer(r'(\w+)=([+-]?\d+(?:\.\d+)?)', line):
            key, val = match.groups()
            if '.' in val:
                result[key] = float(val)
            else:
                result[key] = int(val)
        
        return result
    
    def loop(self):
        """Read serial and publish turtle commands."""
        try:
            line = self.ser.readline().decode('utf-8').strip()
            if not line:
                return
            
            # Log occasionally (every ~4 reads to avoid spam)
            import time
            if len(line) > 10 or abs(self.x) > 0.02 or abs(self.y) > 0.02:
                self.get_logger().debug(f"ESP-NOW: {line[:60]}...")
            
            data = self.parse_remote_line(line)
            if not data:
                return
            
            # Track state from firmware
            manual = data.get('manual', 0) == 1
            robot_connected = data.get('robot', 0) == 1
            
            # Update state (simplified - just track manual mode)
            self.manual = manual
            x = data.get('x', 0.0)
            y = data.get('y', 0.0)
            
            # Only publish when in MANUAL override mode
            if manual:
                twist = Twist()
                # Invert X: stick away from you = forward
                twist.linear.x = -x * self.bpf
                # Y positive = turn left (negative angular z)
                twist.angular.z = y * self.bpf
                
                # Log current command
                self.get_logger().info(f"Twist: x={twist.linear.x:.2f}, z={twist.angular.z:.2f}")
                
                self.cmd_pub.publish(twist)
                
        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    print("Starting ESP32 Remote Serial Bridge...")
    print("This reads from your physical ESP32 remote's serial output")
    print("and drives turtlesim for testing.")
    print()
    print("Make sure:")
    print("  1. ESP32 remote is flashed with esp32_remote.ino firmware")
    print("  2. Serial port path matches your USB adapter (/dev/tty.usbserial-XXX)")
    print()
    print("On another terminal (Pi):")
    print("  ros2 run turtlesim turtlesim_node")
    print("  ros2 launch honebot_follow turtle_follow.launch.py use_safety:=false")
    print()
    
    bridge = RemoteSerialBridge()
    
    try:
        while rclpy.ok():
            bridge.loop()
            rclpy.spin_once(bridge, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy()
        print("Goodbye!")

if __name__ == '__main__':
    main()
PYEOF

chmod +x ~/test_esp32_turtle.py
echo ""
echo "✓ Created test script at ~/test_esp32_turtle.py"
echo ""

# Step 5: Instructions for running
echo "=== Steps to Run ==="
echo ""
echo "1. Update serial port in ~/test_esp32_turtle.py (find actual port with:")
echo '   ls /dev/tty.usbserial*'
echo ""
echo "2. On your Mac, flash the remote:"
echo '   arduino-cli upload -p <your_port> --fqbn esp32:esp32:esp32 firmware/esp32_remote'
echo ""
echo "3. Monitor remote output (Mac):"
echo '   arduino-cli monitor -p <your_port> -c baudrate=115200'
echo ""
echo "4. On Raspberry Pi:"
echo "   cd ~/honebot_ws/src/honebot_bringup/esp32_remote_bridge"
echo "   source install/setup.bash"
echo "   ros2 run test_esp32_turtle"
echo ""
echo "5. In another terminal on Pi:"
echo "   ros2 run turtlesim turtlesim_node"
echo "   ros2 launch honebot_follow turtle_follow.launch.py use_safety:=false"
echo ""
echo "6. Test the remote!"
echo "   - Push stick: turtle moves forward/backward"
echo "   - Click button: stops immediately"
