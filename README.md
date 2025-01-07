# 2D LiDAR to 3D Scanner with Stepper Motor + ROS2

This project transforms a **2D Slamtec RPLIDAR** into an **approximate 3D LiDAR** by physically rotating the sensor with a **stepper motor**. As the stepper tilts the LiDAR over a range of angles, each 2D “slice” of data is rotated in 3D space, accumulating to form a 3D point cloud.

We **do not** use an encoder in this design—instead, we compute the tilt angle from the step count of the stepper motor (the “theoretical angle”). If the stepper never stalls or slips, this yields reasonably accurate 3D scans.

---

## Table of Contents

1. [Project Overview](#project-overview)  
2. [Hardware Requirements](#hardware-requirements)  
3. [Wiring Diagram](#wiring-diagram)  
4. [Theory & Math: 2D to 3D Conversion](#theory--math-2d-to-3d-conversion)  
   1. [2D LiDAR Data Model](#2d-lidar-data-model)  
   2. [Polar to Cartesian in 2D](#polar-to-cartesian-in-2d)  
   3. [Adding a Rotation for 3D](#adding-a-rotation-for-3d)  
   4. [No-Encoder Angle Computation](#no-encoder-angle-computation)  
   5. [Summary of Formulas](#summary-of-formulas)  
5. [ESP32 Code (Stepper + Angle)](#esp32-code-stepper--angle)  
6. [ROS2 Package Setup](#ros2-package-setup)  
   1. [Folder Structure](#folder-structure)  
   2. [esp32_angle_node.py](#esp32_angle_nodepy)  
   3. [lidar_2d_to_3d.py](#lidar_2d_to_3dpy)  
   4. [setup.py & package.xml](#setuppy--packagexml)  
7. [Building and Running](#building-and-running)  
8. [RPLIDAR Driver](#rplidar-driver)  
9. [Visualization in RViZ](#visualization-in-rviz)  
10. [Optional Launch File](#optional-launch-file)  
11. [Future Improvements](#future-improvements)  
12. [License](#license)  

---

## Project Overview

We have:  
- **Slamtec RPLIDAR** connected to a **PC** via USB for 2D scans.  
- A **stepper motor** driven by an **ESP32** to tilt the LiDAR from 0° to 180° (or 360°).  
- A ROS2 node on the PC merges `/scan` data with the “tilt angle” published by the ESP32, generating a 3D point cloud.  

**End Result**: As the LiDAR sweeps in a 2D plane, we rotate that plane in 3D space, accumulating a “shell” or dome of points.

---

## Hardware Requirements

- **Slamtec RPLIDAR** (e.g., C1, A1, A2, etc.).  
- **ESP32** (any typical development board).  
- **Stepper Motor** + **Microstepping Driver** (A4988, DRV8825, TMC-series, etc.).  
- **Power Supply** (e.g. 12V for the stepper driver).  
- **Wires/Cables** for connections.  
- (Optional) **Mounting bracket** to attach the LiDAR to the stepper motor shaft.

No encoder is used—only step counting.

---

## Wiring Diagram

1. **RPLIDAR → PC**  
   - Use the official USB cable or USB-to-serial adapter from Slamtec.  
   - Appears on the PC as `/dev/ttyUSB0` (or similar).  

2. **ESP32 → Stepper Driver**  
   - **STEP_PIN** (GPIO25) → **STEP** on driver  
   - **DIR_PIN** (GPIO26) → **DIR** on driver  
   - **ENABLE_PIN** (GPIO27) → **ENABLE** on driver (active LOW on many drivers)  
   - Motor driver power: **VMOT** from 12V supply, **GND** shared with ESP32  
   - Stepper coils to A+/A-, B+/B- on driver  

3. **ESP32 → PC**  
   - Standard USB cable for programming and serial data output (angle publishing).

---

## Theory & Math: 2D to 3D Conversion

The core idea is to take each point from the **2D LiDAR plane** and rotate it around an axis (often the Y-axis) by an angle \(\theta\) that corresponds to the tilt of the LiDAR.

### 2D LiDAR Data Model

A 2D LiDAR provides **range measurements** (r) at **angular positions** (alpha). In ROS2, this is a `LaserScan` message with:

- `angle_min`, `angle_max`, `angle_increment`  
- `ranges[]` (each range is `r[i]`)

**For beam** (i):


alpha_i = {angle_min} + i times angle_increment, quad
r_i = {ranges}[i].

### Polar to Cartesian in 2D

We convert polar ((r_i, alpha_i)) to 2D Cartesian ((x{2d}, y{2d})) as:

x{2d} = ri cos(alpha_i), 
quad
y{2d} = ri sin(alpha_i).

Here, (z=0) because it’s a flat plane.

### Adding a Rotation for 3D

When the LiDAR plane is tilted by **(theta)** around the Y-axis, we apply the **3D rotation matrix** (Ry(theta)). For a point ((x{2d}, y{2d}, 0)):

begin{aligned}
x{3d} &= x{2d}cos(theta) + 0 cdot sin(theta) = x{2d}cos(theta), [6pt]
y{3d} &= y{2d}, [6pt]
z{3d} &= -,x{2d} sin(theta) + 0 cdot cos(theta) = -,x{2d} sin(theta).
end{aligned}


If we consider different orientation conventions, we might mix (x{2d}) and (y{2d}) a bit differently; the essence is that each 2D point is **rotated** in 3D by **(theta\)**.

### No-Encoder Angle Computation

Since we have **no encoder**, we assume no missed steps. The tilt angle \(\theta\) is derived from the step count:

\[
 \theta \;=\; \bigl(\text{step count}\bigr) 
             \times \left(\frac{2\pi}{\text{steps per revolution}}\right).
\]

- For half a revolution (0→180°), we only go up to half that many steps.  
- The ESP32 increments the step count each time it pulses the step pin.

## ESP32 Code (Stepper + Angle)

Below is the **Arduino** code for the ESP32 (`stepper_2d_to_3d.ino`). It:
- Sweeps from 0° → 180° → 0°.  
- Calculates angle from step count.  
- Prints the angle over Serial at 115200 baud.

```cpp
/*
   stepper_2d_to_3d.ino

   1) Rotates a stepper motor back and forth between 0° and 180°.
   2) Computes theoretical angle from step count.
   3) Sends angle over Serial so ROS2 can read it.
*/

#include <Arduino.h>

// --- Config ---
const int STEPS_PER_REV = 1600;  // e.g. 200 steps * 8 microstepping
const float SWEEP_ANGLE = 180.0; // 0 to 180° sweep
const int STEP_DELAY_MICROS = 500; // microseconds for each step pulse

// Pins
const int STEP_PIN   = 25;
const int DIR_PIN    = 26;
const int ENABLE_PIN = 27;

// Internal vars
int currentSteps = 0;
bool movingForward = true;
int maxSteps = 0;

void setup() {
  Serial.begin(115200);

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // Enable driver (active low)
  digitalWrite(DIR_PIN, HIGH);   // Move forward initially

  maxSteps = (int)((SWEEP_ANGLE / 360.0) * STEPS_PER_REV);

  Serial.println("=== Stepper-based 3D Lidar ===");
  Serial.print("Steps per rev: "); Serial.println(STEPS_PER_REV);
  Serial.print("Sweep angle: ");    Serial.println(SWEEP_ANGLE);
  Serial.print("Max steps: ");      Serial.println(maxSteps);
}

void loop() {
  // Calculate angle
  float angle = (float)currentSteps / (float)STEPS_PER_REV * 360.0;
  // Print angle
  Serial.println(angle);

  // Check bounds
  if (movingForward && currentSteps >= maxSteps) {
    movingForward = false;
    digitalWrite(DIR_PIN, LOW);
  } else if (!movingForward && currentSteps <= 0) {
    movingForward = true;
    digitalWrite(DIR_PIN, HIGH);
  }

  // Single step
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_DELAY_MICROS);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(STEP_DELAY_MICROS);

  // Update step count
  if (movingForward) currentSteps++;
  else currentSteps--;
}
```

---

## ROS2 Package Setup

We’ll create a **Python-based** ROS2 package with two main nodes:

1. **`esp32_angle_node.py`**: Reads the angle from the ESP32 serial port → publishes `std_msgs/Float32` on `/lidar_angle`.  
2. **`lidar_2d_to_3d.py`**: Subscribes to `/scan` + `/lidar_angle` → rotates each 2D point → publishes `/point_cloud_3d`.

### Folder Structure

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_3d_lidar_pkg
```

Generates:
```
my_3d_lidar_pkg
 ┣ package.xml
 ┣ setup.py
 ┣ resource
 ┃  ┗ my_3d_lidar_pkg
 ┣ my_3d_lidar_pkg
 ┃  ┗ __init__.py
 ┗ tests
```

Place your Python scripts in `my_3d_lidar_pkg/my_3d_lidar_pkg/`.

---

### `esp32_angle_node.py`

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial

class ESP32AngleNode(Node):
    def __init__(self):
        super().__init__('esp32_angle_node')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)

        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud_rate').get_parameter_value().integer_value

        self.publisher_ = self.create_publisher(Float32, 'lidar_angle', 10)

        try:
            self.serial_port = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Opened serial port {port} at {baud}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open {port}: {e}")
            exit(1)

        # read angle ~10Hz
        self.timer = self.create_timer(0.1, self.read_angle)

    def read_angle(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('utf-8').strip()
            if line:
                try:
                    angle = float(line)
                    msg = Float32()
                    msg.data = angle
                    self.publisher_.publish(msg)
                except ValueError:
                    self.get_logger().warn(f"Invalid float from ESP32: {line}")

def main(args=None):
    rclpy.init(args=args)
    node = ESP32AngleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

*(Make it executable: `chmod +x esp32_angle_node.py`.)*

---

### `lidar_2d_to_3d.py`

```python
#!/usr/bin/env python3
"""
lidar_2d_to_3d.py
Subscribes to:
  /scan        (LaserScan)
  /lidar_angle (Float32)
Publishes:
  /point_cloud_3d (PointCloud2)

Rotates each 2D LiDAR point by the tilt angle, producing a 3D cloud.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import Float32
from sensor_msgs_py import point_cloud2
import numpy as np

class Lidar2Dto3DNode(Node):
    def __init__(self):
        super().__init__('lidar_2d_to_3d')
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.angle_sub = self.create_subscription(Float32, '/lidar_angle', self.angle_callback, 10)
        self.cloud_pub = self.create_publisher(PointCloud2, '/point_cloud_3d', 10)

        self.current_angle_rad = 0.0

    def angle_callback(self, msg):
        self.current_angle_rad = np.radians(msg.data)

    def lidar_callback(self, scan_msg):
        points_3d = []
        angle = scan_msg.angle_min

        for r in scan_msg.ranges:
            if scan_msg.range_min < r < scan_msg.range_max:
                # Convert to 2D cartesian
                x_2d = r * np.cos(angle)
                y_2d = r * np.sin(angle)
                # Rotate around Y-axis
                x_3d = x_2d * np.cos(self.current_angle_rad) + y_2d * np.sin(self.current_angle_rad)
                z_3d = -x_2d * np.sin(self.current_angle_rad) + y_2d * np.cos(self.current_angle_rad)
                y_3d = 0.0
                points_3d.append((x_3d, y_3d, z_3d))

            angle += scan_msg.angle_increment

        pc2_msg = self.create_pointcloud2(points_3d, scan_msg.header)
        self.cloud_pub.publish(pc2_msg)

    def create_pointcloud2(self, points, header):
        fields = [
            point_cloud2.PointField(name='x', offset=0,  datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='y', offset=4,  datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='z', offset=8,  datatype=point_cloud2.PointField.FLOAT32, count=1),
        ]
        header.frame_id = 'base_link'  # Adjust to your TF
        return point_cloud2.create_cloud(header, fields, points)

def main(args=None):
    rclpy.init(args=args)
    node = Lidar2Dto3DNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

*(Make it executable: `chmod +x lidar_2d_to_3d.py`.)*

---

### `setup.py & package.xml`

**`setup.py`** (top-level in `my_3d_lidar_pkg`):

```python
from setuptools import setup

package_name = 'my_3d_lidar_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Convert 2D LiDAR data into 3D by stepper-based tilt',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'esp32_angle_node = my_3d_lidar_pkg.esp32_angle_node:main',
            'lidar_2d_to_3d = my_3d_lidar_pkg.lidar_2d_to_3d:main',
        ],
    },
)
```

**`package.xml`**:

```xml
<package format="3">
  <name>my_3d_lidar_pkg</name>
  <version>0.0.0</version>
  <description>Stepper-based 3D LiDAR scanning</description>
  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>Apache License 2.0</license>

  <buildtool_depend>ament_python</buildtool_depend>
  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>rclpy</exec_depend>
  <exec_depend>std_msgs</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>sensor_msgs_py</exec_depend>
</package>
```

---

## Building and Running

1. **Build**:
   ```bash
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```
2. **Run** the angle node (adjust port if needed):
   ```bash
   ros2 run my_3d_lidar_pkg esp32_angle_node --ros-args -p serial_port:=/dev/ttyUSB1
   ```
3. **Run** the 2D-to-3D point cloud node:
   ```bash
   ros2 run my_3d_lidar_pkg lidar_2d_to_3d
   ```

---

## RPLIDAR Driver

Install the ROS2 driver for RPLIDAR:

```bash
sudo apt update
sudo apt install ros-humble-rplidar-ros
```

Connect your LiDAR via USB (likely `/dev/ttyUSB0`), then run:

```bash
ros2 run rplidar_ros rplidar_node --ros-args -p serial_port:=/dev/ttyUSB0 -p serial_baudrate:=256000
```
*(Some models might use 115200 instead of 256000.)*

Check if `/scan` is published:

```bash
ros2 topic list
ros2 topic echo /scan
```

---

## Visualization in RViZ

```bash
ros2 run rviz2 rviz2
```

1. **Fixed Frame**: set to `base_link` (or your chosen frame).  
2. **Add** a `PointCloud2` display: topic = `/point_cloud_3d`.  
3. As the stepper motor tilts the LiDAR, you’ll see points stacked into a 3D shape.

---

## Optional Launch File

You can create a single launch file to start everything at once:

```python
# my_3d_lidar_pkg/launch/scan3d.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyUSB0'},
                {'serial_baudrate': 256000},
            ]
        ),
        Node(
            package='my_3d_lidar_pkg',
            executable='esp32_angle_node',
            name='esp32_angle_node',
            output='screen',
            parameters=[
                {'serial_port': '/dev/ttyUSB1'},
                {'baud_rate': 115200}
            ]
        ),
        Node(
            package='my_3d_lidar_pkg',
            executable='lidar_2d_to_3d',
            name='lidar_2d_to_3d',
            output='screen'
        ),
    ])
```

Run:

```bash
ros2 launch my_3d_lidar_pkg scan3d.launch.py
```

---

## Future Improvements

1. **Encoder Feedback**: Use an encoder (like AS5600) to track actual angle, avoiding missed steps.  
2. **Full 360 Tilt**: Spin the LiDAR for a full 360° coverage or combine multiple axes.  
3. **Data Accumulation**: Currently, each `/scan` + angle produces a partial 3D cloud. You can store it in a larger buffer for a complete environment map.  
4. **micro-ROS**: If you want an entirely standalone microcontroller solution, you can port the LiDAR driver to the ESP32 (complex) and publish LaserScan messages over micro-ROS.  
5. **Timing & Synchronization**: If precise angle-timestamp alignment is needed, consider a hardware approach or an IMU/encoder for more accurate transforms.

---

## License

Licensed under [Apache 2.0](LICENSE) or the license of your choice.

---

### Conclusion

This project demonstrates how to **tilt a 2D LiDAR** with a stepper motor to create a **3D scanning system**. It merges the LiDAR’s `/scan` data and the tilt angle into a **3D point cloud** using ROS2, allowing for real-time visualization and mapping. Enjoy experimenting and feel free to submit improvements or ideas!
