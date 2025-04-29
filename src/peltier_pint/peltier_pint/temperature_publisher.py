import rclpy
import os
from rclpy.node import Node
from std_msgs.msg import Float64

class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__("temperature_publisher")
        
        # Try to locate the DS18B20 sensor device
        base_path = "/sys/bus/w1/devices"
        found_sensor = False
        
        if os.path.exists(base_path):
            for name in os.listdir(base_path):
                if name.startswith("28-"):
                    self.sensor_file = os.path.join(base_path, name, "w1_slave")
                    found_sensor = True
                    break
        
        if not found_sensor:
            self.get_logger().error(f"DS18B20 sensor not found in '{base_path}'. Did you enable 1-wire?")
            raise RuntimeError("DS18B20 sensor not found")
        
        self.publisher = self.create_publisher(Float64, "temperature", 10)
        # Call every second
        self.timer = self.create_timer(1.0, self.timer_callback)
        
    def timer_callback(self):
        try:
            with open(self.sensor_file, "r") as f:
                lines = f.readlines()
                if lines[0].strip()[-3:] != "YES":
                    self.get_logger().warn("Not publishing: Sensor reading not valid")
                    return
                
                temp_str = lines[1].split("t=")[-1]
                temperature = float(temp_str) / 1000.0
                msg = Float64()
                msg.data = temperature
                self.publisher.publish(msg)
        except Exception as e:
                self.get_logger().error(f"Unexpected error: {e}")
                
                
                
def main(args=None):
    rclpy.init(args=args)
    try:
        node = TemperaturePublisher()
        rclpy.spin(node)
    except RuntimeError as e:
        print(f"Node failed to start: {e}")
    finally:
        rclpy.shutdown()
        
if __name__ == "__main__":
    main()
      
