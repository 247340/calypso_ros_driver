# CALYPSO - ROS2 Driver

## About the Project

**CALYPSO - ROS2 Driver** is a ROS2 driver for anemometers manufactured by Calypso, which communicate via UART (Universal Asynchronous Receiver/Transmitter) technology. 

The development is based on the **Ultra-Low-Power Ultrasonic Wind Meter (ULP Standard)** model. Once the project is launched, the ROS2 node starts receiving data and publishes it into a `WindSpeed` message. This message contains the following fields:
- **std_msgs/Header header**
- **float32 wind_spd**: Wind speed value.
- **int32 wind_dir**: Wind direction value.

### Features:
- Interface with Calypso anemometers via UART.
- Real-time wind speed and direction data.
- Integration with ROS2 ecosystem, utilizing `std_msgs/Header` for message structure.

## Requirements

- ROS2 (Foxy, Galactic or later).
- Calypso anemometer with UART communication capability.
- A system capable of connecting to the anemometer via UART (e.g., Raspberry Pi, PC with a UART interface).

## Installation

1. Clone the repository into your ROS2 workspace:
   ```bash
   git clone https://github.com/247340/calypso_ros_driver.git
