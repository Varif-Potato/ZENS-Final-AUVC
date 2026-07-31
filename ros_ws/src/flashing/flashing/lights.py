import rclpy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn
from std_msgs.msg import Int16

class Lights(Node):
    def __init__(self):
        super().__init__("lights")
        self.lightPWM = 0
        self.light_pub = self.create_publisher(
            OverrideRCIn,
            "/override_rc",
            10
        )
        self.light_sub = self.create_subscription(
            Int16,
            "/target_lights",
            self.set_lights,
            10
        )
        self.timer = self.create_timer(
            0.1,
            self.turn_lights
        )
        
    def turn_lights(self):
        msg = OverrideRCIn()
        
        msg.channels = [OverrideRCIn.CHAN_NOCHANGE] * 18
        msg.channels[8] = self.lightPWM 
        
        self.light_pub.publish(msg)


    def set_lights(self, msg):
        self.lightPWM = msg.data
        if self.lightPWM == 0:
            self.lightPWM = OverrideRCIn.CHAN_RELEASE
        self.get_logger().info(f"Light PWM of {msg.data} given")
    
def main(args=None):
    rclpy.init(args=args)
    node = Lights()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
