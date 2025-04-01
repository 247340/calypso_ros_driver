# CALYPSO - ROS2 Driver

**CALYPSO - ROS2 Driver** is a ROS2 driver for ultrasonic anemometers manufactured by Calypso, which communicate via UART technology. 

Once the project is launched, the ROS2 node starts receiving data and publish `WindSpeed` message on the **/wind_speed_data** topic. This message contains the following fields:
- **std_msgs/Header header**
- **float32 wind_spd** - wind speed value.
- **int32 wind_dir** - wind direction value.

The USB port, baud rate, and logging to a file (enable or disable) can be configured via a configuration file.

### Features:
- Interface with Calypso anemometers via UART.
- Publish real-time wind speed and direction data in a ROS2 message.
- Save time, wind speed, and direction to a CSV file.

## Requirements

- ROS2 (Humble, Foxy, Galactic, or later).
- Calypso anemometer with UART communication capability.
- Ubuntu (or compatible Linux distribution).
- 
## Supported Anemometers

This driver was tested on the following anemometer models:
- **CALYPSO Ultra-Low-Power Ultrasonic Wind Meter (ULP Standard)**

If you are using a different model of ultrasonic anemometer, additional modifications may be required to support it.

## Nodes
- **calypso_ros**: 
  - Responsible for interfacing with the Calypso anemometer, receiving data, and publishing it to the ROS2 system.

## Topics
- **/wind_speed_data**:
  - Type: 'calypso_ros_driver_msgs/msg/WindSpeed"
  - Description: wind speed and direction data received from the anemometer.

## Parameters

The following parameters can be configured via a configuration file:
- **usb_port**: USB port to which the anemometer is connected (e.g., `/dev/ttyUSB0`).
- **baud_rate**: Baud rate for UART communication (e.g., `9600`).
- **logging**: Boolean parameter to enable or disable logging to a CSV file located in the log folder inside the calypso_ros_driver directory.

## Run it
1. Clone the repository into your ROS2 workspace:
   - git clone https://github.com/247340/calypso_ros_driver.git
2. Install dependencies
   - cd ..
   - rosdep install --from-paths src --ignore-src -r -y
4. build it
   - colcon build
5. run it
   - ros2 launch calypso_ros_driver calypso_ros_driver_launch.py


   
   
