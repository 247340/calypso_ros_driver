import rclpy
import os
from rclpy.node import Node
from std_msgs.msg import String
from calypso_ros_driver.uart_communication import UART_communication
from calypso_ros_driver.decode_dat import decode_data
from calypso_ros_driver.file_data_logger import Data_logger
from calypso_ros_driver_msgs.msg import WindSpeed
import time


class calypso_ros(Node):
    def __init__(self):
        super().__init__('calypso_ros')
        self.declare_parameter('logging', False)
        self.declare_parameter('usb_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('log_directory', '')

        self.logging_param = self.get_parameter('logging').get_parameter_value().bool_value
        self.usb_port = self.get_parameter('usb_port').get_parameter_value().string_value
        self.baudrate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # Ošetření log_directory
        log_dir_param = self.get_parameter('log_directory').get_parameter_value().string_value
        if log_dir_param and os.path.isdir(log_dir_param):
            self.log_directory = log_dir_param
        else:
            if log_dir_param:
                self.get_logger().warn(f"Invalid log directory path: '{log_dir_param}', using default path.")
            self.log_directory = ''  # použije se výchozí cesta v Data_logger

        self.publisher = self.create_publisher(WindSpeed, 'wind_speed_data', 10)

        self.uart = UART_communication(port=self.usb_port, baudrate=self.baudrate, timeout=1)
        self.uart.set_callback(self.process_data)

        self.get_logger().info(
            f'Successfully connected to serial port: {self.usb_port}, Baud rate: {self.baudrate}, Logging is {"enabled" if self.logging_param else "disabled"}'
        )

        if self.logging_param:
            logger_name = time.strftime("%Y%m%d_%H%M%S", time.localtime())
            self.logger = Data_logger(
                filename=f"calypso_log_{logger_name}.csv",
                flush_interval=2,
                log_directory=self.log_directory,
                logger=self.get_logger()
            )

        self.uart.connect()

    def process_data(self, data):
        wind_dir, wind_spd = decode_data(data)
        if wind_spd != None and wind_dir != None:
            msg = WindSpeed()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.wind_spd = wind_spd
            msg.wind_dir = wind_dir
            self.publisher.publish(msg)
            if self.logging_param:
                timestamp = self.get_clock().now().nanoseconds // 1000
                self.logger.write_data_to_file(timestamp, wind_dir, wind_spd)
            
    def __del__(self):
        self.uart.close()

def main(args=None):
    rclpy.init(args=args)
    node = calypso_ros()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

