# Project Overview

This project is a ROS 2 application for controlling robotic hands. It receives hand status (open/closed) from an MQTT broker, processes it in a ROS 2 node, and then controls the hand actuators (Dynamixel motors or servos) accordingly.

The project is divided into two main ROS 2 packages:

*   **`communication_pkg`**: Handles the communication with the MQTT broker and publishes the hand status to ROS 2 topics.
*   **`controller_pkg`**: Subscribes to the hand status topics and controls the robotic hand hardware.

## Key Technologies

*   **ROS 2**: The core framework for the robotic application.
*   **Python**: The programming language used for the ROS 2 nodes and scripts.
*   **MQTT**: The protocol used for receiving hand status data.
*   **Dynamixel SDK**: Used for controlling Dynamixel motors.
*   **Adafruit ServoKit**: Used for controlling servos via a PCA9685 board.

# Building and Running

## Building the Project

To build the project, use the standard ROS 2 build tools:

```bash
colcon build
```

## Running the Project

The `control.sh` script is provided to start and stop the application.

**To start the application:**

```bash
./control.sh start
```

This will:
1.  Source the ROS 2 environment.
2.  Source the workspace environment.
3.  Start the `receivejson.py` MQTT client.
4.  Start the `processamento_node` ROS 2 node.
5.  Start the `hand_controller.py` ROS 2 node.

**To stop the application:**

```bash
./control.sh stop
```

This will kill the processes started with `./control.sh start`.

# Development Conventions

*   The project follows the standard ROS 2 package structure.
*   Python is used for all nodes and scripts.
*   The `ament_flake8` and `ament_pep257` linters are used for code quality.

## Utility Scripts

### `utils/scan_dynamixel.py`

This script can be used to scan for connected Dynamixel motors.

**Usage:**

```bash
python3 utils/scan_dynamixel.py --port /dev/ttyUSB0 --baud 1000000
```

*   `--port`: The serial port to scan.
*   `--baud`: The baud rate to use.
