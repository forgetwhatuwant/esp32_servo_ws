# esp32_servo_ws

This workspace contains the `servo_serial_bridge_cpp` ROS package, which provides a bridge between ROS and a servo controller via a serial port. It is designed to send servo position commands from a ROS topic to a servo controller connected over serial (e.g., to an ESP32 or similar microcontroller).

## Package: servo_serial_bridge_cpp

### Description
This package subscribes to a ROS topic (`/servo`) of type `std_msgs/UInt16` and sends the received values as formatted serial commands to a specified serial port. The intended use is to control a servo motor via a microcontroller that interprets the serial commands.

### Dependencies
- ROS (tested with Kinetic/Melodic/Noetic)
- `roscpp`
- `std_msgs`
- `boost`

### Node: `servo_serial_bridge_cpp_node`

#### Parameters
- `~port` (string, default: `/dev/ttyUSB1`): The serial port to which the servo controller is connected.

#### Subscribed Topics
- `/servo` (`std_msgs/UInt16`): The desired servo position (0-999). Each message is sent as a formatted string (e.g., `S:123\n`) over the serial port.

### Usage Example
1. Build the workspace:
   ```bash
   catkin_make
   ```
2. Source your workspace:
   ```bash
   source devel/setup.bash
   ```
3. Run the node:
   ```bash
   rosrun servo_serial_bridge_cpp servo_serial_bridge_cpp_node _port:=/dev/ttyUSB1
   ```
4. Publish a servo command:
   ```bash
   rostopic pub /servo std_msgs/UInt16 "data: 150"
   ```

### Maintainer
- root (<root@todo.todo>)

### License
- TODO (please update in `package.xml`)

---

For more details, see the source code in `src/servo_serial_bridge_cpp/src/servo_serial_bridge_cpp.cpp` and the package manifest in `src/servo_serial_bridge_cpp/package.xml`.
