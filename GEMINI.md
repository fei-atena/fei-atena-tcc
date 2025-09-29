# GEMINI.md - Project Analysis

## Project Overview

This is a ROS 2 project designed to control a pair of robotic hands. It functions by receiving hand status commands (e.g., open/closed) from an external source via MQTT, processing these commands within the ROS 2 ecosystem, and finally actuating the physical hand hardware.

The system is divided into two main ROS 2 packages:

*   `communication_pkg`: A Python package responsible for bridging MQTT communication to the ROS 2 network. It reads data from an MQTT client and publishes it as ROS 2 messages.
*   `controller_pkg`: A package containing the hardware abstraction layer. It subscribes to ROS 2 topics and translates the commands into low-level control signals for a Dynamixel servo and a PCA9685-based servo controller.

The primary technologies used are ROS 2 (rclpy), Python, MQTT (paho-mqtt), Dynamixel SDK, and Adafruit ServoKit.

## Architecture Flow

1.  An external application publishes a JSON message to the `pingpong/ros` MQTT topic (e.g., `{"righthand": false, "lefthand": true}`).
2.  The `receivejson.py` script, acting as an MQTT client, receives this message and saves it to the `recebido.json` file.
3.  The `processamento_node` from `communication_pkg` reads `recebido.json` periodically.
4.  This node then publishes the boolean status for each hand to their respective ROS 2 topics: `/right/hand_joint` and `/left/hand_joint`.
5.  The `pca_hand_controller` (for the right hand) and `hand_controller` (for the left hand) nodes in `controller_pkg` subscribe to these topics.
6.  Upon receiving a message, each controller node commands its hardware (servos via PCA9685 or a Dynamixel motor) to open or close the corresponding hand.

## Building and Running

### 1. Setup Environment

Before building or running, source your main ROS 2 installation. Replace `jazzy` with your specific ROS 2 distribution if different.

```bash
source /opt/ros/jazzy/setup.bash
```

### 2. Build the Workspace

From the root of the project (`/home/atena/fei-atena-tcc`), run the `colcon build` command.

```bash
colcon build
```

### 3. Source the Local Workspace

After a successful build, source the local setup file to make the new packages and nodes available in your terminal.

```bash
source install/setup.bash
```

### 4. Run the System

The following components need to be run in separate terminals.

**Terminal 1: MQTT Client**
Start the script that listens for MQTT messages.

```bash
python3 receivejson.py
```

**Terminal 2: Processing Node**
Start the node that reads the JSON file and publishes to ROS 2 topics.

```bash
ros2 run communication_pkg processamento_node
```

**Terminal 3: Left Hand Controller (Dynamixel)**
Start the controller for the Dynamixel-based hand.

```bash
ros2 run controller_pkg hand_controller.py
```

**Terminal 4: Right Hand Controller (PCA Servos)**
Start the controller for the servo-based hand.

```bash
ros2 run controller_pkg pca_hand_controller.py
```

## Development Conventions

*   **ROS 2 Parameters:** The `hand_controller` for the Dynamixel motor is configured using ROS 2 parameters (e.g., `devicename`, `baudrate`), which is a best practice for configurability.
*   **QoS Settings:** The project correctly uses QoS profiles with `TRANSIENT_LOCAL` durability for state-like topics. This ensures that new subscribers receive the last published message immediately, which is ideal for controlling state.
*   **Code Duplication:** The `receivejson.py` script in the root directory appears to be identical to `src/communication_pkg/communication_pkg/receivejson_node.py`. This redundancy could be cleaned up.
*   **Hardcoded Path:** The `processamento_node` contains a hardcoded absolute path to the `recebido.json` file. For better portability, this should be changed to a relative path or, even better, configured via a ROS 2 parameter.
