import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from gpiozero import DigitalOutputDevice

class RelayControlSubscriber(Node):
    def __init__(self):
        super().__init__("relay_control_subscriber")
        self.subscription = self.create_subscription(Float64, "temperature", self.listener_callback, 10)
        self.relay_1 = DigitalOutputDevice(17)
        self.relay_2 = DigitalOutputDevice(27)
        
    def listener_callback(self, msg):
        temperature = msg.data
        if temperature > 8.0:
            self.get_logger().info(f"Temperature: {msg.data}. Switching relay on.")
            self.relay_1.on()
            self.relay_2.on()
        elif temperature < 6.0:
            self.get_logger().info(f"Temperature: {msg.data}. Switching relay off.")
            self.relay_1.off()
            self.relay_2.off()
        else:
            self.get_logger().info(f"Temperature: {msg.data}. Relay remains in previous state.")
        
        
def main(args=None):
    rclpy.init(args=args)
    relay_control_subscriber = RelayControlSubscriber()
    rclpy.spin(relay_control_subscriber)
    relay_control_subscriber.destroy_node()
    relay_control_subscriber.relay_1.off()
    relay_control_subscriber.relay_2.off()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
    
