import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl
from std_msgs.msg import Int16



class rotatehold(Node):
    def __init__(self):
        super().__init__("rotatehold")
        self.currentAngle = 0.0
        self.dt = 0.1
        self.targetAngle = 0.0
        self.errorAcu = 0.0
        self.errorPre = 0.0
        self.meter = 0.0
        self.processVar = 0.0
        self.manual_pub = self.create_publisher(
            ManualControl,
            "/manual_control",
            10
        )
        self.target_sub = self.create_subscription(
            Int16,
            "/target_angle",
            self.targetSetting, 
            10
        )
        self.sub = self.create_subscription(
            Int16,
            "/heading",
            self.angleCal,
            10
        )
        self.timer = self.create_timer(
            self.dt,
            self.anglePublish
        )
        
    def angleCal(self, msg):
        self.currentAngle = msg.data
        self.get_logger().info(f"Heading: {self.currentAngle}, ProcessVar: {self.processVar}")
        
    def targetSetting(self, msg):
        self.targetAngle = msg.data
        # if msg.data < 0:
        #     self.targetAngle = 360.0 - msg.data
        # elif msg.data > 360:
        #     self.targetAngle = msg.data - 360.0
        self.targetAngle = msg.data % 360.0
        self.get_logger().info(f"New Target Depth: {self.targetAngle:.2f} m")
    def anglePublish(self):
    
        self.PIDController(self.targetAngle, 1.4, 0.0, 0.65)
        self.get_logger().info(f"Published PID, angle {self.currentAngle}")
        
    def PIDController(self, target, kP, kI, kD):
        msg = ManualControl()
        error = (target - self.currentAngle + 180) % 360 - 180
        self.errorAcu += error * self.dt
        self.currentkP = kP * error
        self.currentkI = kI * self.errorAcu
        self.currentkD = kD * (error - self.errorPre) / self.dt
        self.errorPre = error
        

        self.processVar = (self.currentkP + self.currentkI + self.currentkD)
        
        msg.r = self.processVar
            
        
        self.manual_pub.publish(msg)
        self.get_logger().info(f"Process Var{self.processVar}")
        self.get_logger().info(
            f"Target={target:.2f}, Current={self.currentAngle:.2f}, "
            f"Error={error:.2f}, Output={self.processVar:.2f}"
        )
def main(args=None):
    rclpy.init(args=args)
    node = rotatehold()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()