import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from calypso_ros_driver.uart_communication import UART_communication 
from calypso_ros_driver.decode_dat import decode_data, get_time_microseconds 
from calypso_ros_driver.file_data_logger import Data_logger
from custom_msgs.msg import WindSpeed
import time


class LoggerNode(Node):
    def __init__(self):
        super().__init__('logger_node')
        self.declare_parameter('logging', False) #parameter to enable or disable logging data to file
        self.logging_param = self.get_parameter('logging').get_parameter_value().bool_value
        self.publisher = self.create_publisher(WindSpeed, 'wind_speed_data', 10)
        self.uart = UART_communication(port='/dev/ttyUSB0', baudrate=115200, timeout=1) # UART communication object
        self.uart.connect()
        self.timer = self.create_timer(0.01, self.read_and_publish_data)
        if self.logging_param == True:
            logger_name = time.strftime("%Y%m%d_%H%M%S", time.localtime())
            self.logger = Data_logger(filename=f"uart_data_log_{logger_name}.csv", flush_interval=3)

    def read_and_publish_data(self):
        data = self.uart.read_data()
        if data:
            
            wind_spd, wind_dir = decode_data(data)
            if wind_spd != "None" and wind_dir != "None":
                if self.logging_param == True:
                    time = get_time_microseconds()
                    self.logger.write_data_to_file(time,wind_spd,wind_dir)
                    #self.get_logger().info(f"Data z UART: {time},{wind_spd},{wind_dir}")
                    
                #Create and publish the WindSpeed message
                msg = WindSpeed()
                msg.header.stamp = self.get_clock().now().to_msg()  # Přidání časové značky
                msg.wind_spd = wind_spd
                msg.wind_dir = wind_dir
                self.publisher.publish(msg)
                #self.get_logger().info(f"Publikována zpráva: {msg}")
                
#Closes the UART communication when the node is destroyed.
    def __del__(self):
        self.uart.close()


def main(args=None):
    rclpy.init(args=args)
    node = LoggerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

