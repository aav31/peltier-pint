import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64

class RelayControlSubscriber(Node):
    def __init__(self):
        super().__init__("relay_control_subscriber")
        self.subscription = self.create_subscription(Float64, "temperature", self.listener_callback, 10)
        
    def listener_callback(self, msg):
        temperature = msg.data
        if temperature > 8.0:
            self.get_logger().info(f"Temperature: {msg.data}. Switching relay on.")
        elif temperature < 6.0:
            self.get_logger().info(f"Temperature: {msg.data}. Switching relay off.")
        else:
            self.get_logger().info(f"Temperature: {msg.data}. Relay remains in previous state.")
        
        
def main(args=None):
    rclpy.init(args=args)
    relay_control_subscriber = RelayControlSubscriber()
    rclpy.spin(relay_control_subscriber)
    relay_control_subscriber.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
    
