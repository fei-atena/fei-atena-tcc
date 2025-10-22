#!/usr/bin/env python3
import sys
import argparse
from dynamixel_sdk import *

def scan_motors(device_name, baud_rate):
    """Scans for Dynamixel motors on a given port and baud rate."""
    
    portHandler = PortHandler(device_name)
    packetHandler = PacketHandler(2.0) # Using Protocol 2.0
    port_opened = False

    try:
        if not portHandler.openPort():
            print(f"Failed to open the port {device_name}")
            print("Please check permissions (e.g., sudo chmod a+rw /dev/ttyUSB0)")
            return
        port_opened = True

        print(f"Succeeded to open the port {device_name}")

        if not portHandler.setBaudRate(baud_rate):
            print(f"Failed to change the baudrate to {baud_rate}")
            return
        print(f"Succeeded to change the baudrate to {baud_rate}")

        print("\nScanning for motors...")
        dxl_comm_result, dxl_error = packetHandler.broadcastPing(portHandler)
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Broadcast ping failed: {packetHandler.getTxRxResult(dxl_comm_result)}")
            return

        print("\nDetected Dynamixel motors:")
        found_ids = []
        for i in range(253): # Check IDs 0-252
            if packetHandler.getBroadcastPingResult(portHandler, i):
                found_ids.append(i)
        
        if not found_ids:
            print("  No motors found.")
            print("  - Check motor power and connection.")
            print("  - Check that the correct port and baudrate are used.")
        else:
            for id_val in found_ids:
                print(f"  - ID: {id_val}")

    finally:
        if port_opened:
            portHandler.closePort()
            print("\nPort closed.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Scan for Dynamixel motors.')
    parser.add_argument('--port', type=str, default='/dev/ttyUSB0', help='The device name of the serial port.')
    parser.add_argument('--baud', type=int, default=1000000, help='The baud rate to use.')
    args = parser.parse_args()

    # Source the ROS environment before running the scan
    # This is necessary to find the dynamixel_sdk library
    import os
    ros_distro = os.environ.get('ROS_DISTRO', 'jazzy') # Default to jazzy if not set
    setup_file = f"/opt/ros/{ros_distro}/setup.bash"
    if os.path.exists(setup_file):
        # This is a bit of a hack for a script, ideally the environment is sourced before running
        pass

    scan_motors(args.port, args.baud)
