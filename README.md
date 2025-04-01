# CALYPSO - ROS2 Driver

## About the Project

**CALYPSO - ROS2 Driver** is a ROS2 driver for anemometers manufactured by Calypso, which communicate via UART (Universal Asynchronous Receiver/Transmitter) technology. 

The development is based on the **Ultra-Low-Power Ultrasonic Wind Meter (ULP Standard)** model. Once the project is launched, the ROS2 node starts receiving data and publishes it into a `WindSpeed` message. This message contains the following fields:
- **std_msgs/Header header**
- **float32 wind_spd**: Wind speed value.
- **int32 wind_dir**: Wind direction value.

### Features:
- Interface with Calypso anemometers via UART.
- Publish real-time wind speed and direction data on ROS2 message.
- Save time, windspeed and direction to csv file

## Requirements

- ROS2 (Humble, Foxy, Galactic or later).
- Calypso anemometer with UART communication capability.
- Ubuntu 

## Building

1. Clone the repository into your ROS2 workspace:
   git clone https://github.com/247340/calypso_ros_driver.git
2. Install dependencies
   cd..
   rosdep install --from-paths src --ignore-src -r -y
3. build it
  colcon build
4. run it
   ros2 launch calypso_ros_driver calypso_ros_driver_launch.py
## Config file

   
   
