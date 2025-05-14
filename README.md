# esp32_servo_ws

This workspace contains the `servo_serial_bridge_cpp` ROS package, which provides a bridge between ROS and a servo controller via a serial port. It is designed to send servo position commands from a ROS topic to a servo controller connected over serial (e.g., to an ESP32 or similar microcontroller).

## Package: servo_serial_bridge_cpp

### Description
This package subscribes to a ROS topic (`/sroi_gripper/command`) of type `std_msgs/UInt16` and sends the received values as formatted serial commands to a specified serial port. The intended use is to control a servo motor via a microcontroller that interprets the serial commands.

### Dependencies
- ROS (tested with Kinetic/Melodic/Noetic)
- `roscpp`
- `std_msgs`
- `boost`

### Node: `serial_bridge_node`

#### Parameters
- `~port` (string, default: `/dev/ttyUSB1`): The serial port to which the servo controller is connected.
- `~baud_rate` (int, default: 115200): The baud rate for serial communication.

#### Subscribed Topics
- `/sroi_gripper/command` (`std_msgs/UInt16`): The desired servo position (0-180 degrees). Each message is sent as a formatted command over the serial port.

#### Published Topics
- `/sroi_gripper/state` (`std_msgs/UInt16`): The current state/position of the servo.

### Launch Files
The package includes a launch file for easy startup:
```bash
roslaunch servo_serial_bridge_cpp servo_bridge.launch
```

### Test Scripts

#### test_gripper.py
A Python script located in `src/servo_serial_bridge_cpp/scripts/test_gripper.py` that tests the servo response by publishing commands at 50Hz. The script creates a smooth transition from 0 to 90 degrees over 5 seconds.

Usage:
```bash
# Make the script executable first
chmod +x src/servo_serial_bridge_cpp/scripts/test_gripper.py
# Run the test
rosrun servo_serial_bridge_cpp test_gripper.py
```

### Usage Example
1. Build the workspace:
   ```bash
   catkin_make
   ```
2. Source your workspace:
   ```bash
   source devel/setup.bash
   ```
3. Run the node (either using launch file or directly):
   ```bash
   # Using launch file (recommended)
   roslaunch servo_serial_bridge_cpp servo_bridge.launch
   
   # Or run node directly
   rosrun servo_serial_bridge_cpp serial_bridge_node _port:=/dev/ttyUSB1 _baud_rate:=115200
   ```
4. Publish a servo command:
   ```bash
   rostopic pub /sroi_gripper/command std_msgs/UInt16 "data: 90"
   ```

### Maintainer
- root (<root@todo.todo>)

### License
- TODO (please update in `package.xml`)

---

For more details, see the source code in `src/servo_serial_bridge_cpp/src/serial_bridge.cpp` and the package manifest in `src/servo_serial_bridge_cpp/package.xml`.
