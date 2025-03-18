import rclpy
from rclpy.node import Node
from custom_msgs.msg import WindSpeed

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            WindSpeed,  # Subscribe to the WindSpeed message
            'wind_speed_data',  # Topic name to listen to
            self.listener_callback,  # Callback function
            10)  # QoS history depth
        self.subscription  # Prevent unused variable warning

    def listener_callback(self, msg):
        # Log the received WindSpeed message
        self.get_logger().info(f"I heard wind speed: {msg.wind_spd} {msg.wind_dir}")

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly (optional - otherwise it will be done automatically when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

