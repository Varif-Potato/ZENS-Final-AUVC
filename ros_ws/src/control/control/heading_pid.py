import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl
from std_msgs.msg import Int16
from std_msgs.msg import Float32



class rotatehold(Node):
    def __init__(self):
        super().__init__("rotatehold")
        self.declare_parameter("min_thrust", -300.0)
        self.declare_parameter("max_thrust", 300.0)
        self.declare_parameter("speed", 300.0)
        self.declare_parameter("kP", 1.4)
        self.declare_parameter("kI", 0.0)
        self.declare_parameter("kD", 0.05)
        self.currentAngle = 0.0
        self.dt = 0.1
        self.targetAngle = 0.0
        self.errorAcu = 0.0
        self.errorPre = 0.0
        self.targetSpeed = 0.0
        self.processVar = 0.0
        self.manual_pub = self.create_publisher(
            ManualControl,
            "/manual_control",
            10
        )
        self.for_pub = self.create_publisher(
            Float32,
            "/current_for",
            10
        )
        self.for_sub = self.create_subscription(
            Float32,
            "/target_speed",
            self.speedCallback,
            10
        )
        self.target_sub = self.create_subscription(
            Float32,
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
        self.anglePub = self.create_publisher(
            Float32,
            "/current_heading",
            10
        )
    def angleCal(self, msg):
        self.currentAngle = float((msg.data % 360))
    
        self.get_logger().info(f"Heading: {self.currentAngle}, ProcessVar: {self.processVar}")
    def speedCallback(self, msg):
        self.targetSpeed = float(msg.data)
    def targetSetting(self, msg):
        self.targetAngle = msg.data
        self.targetAngle = msg.data % 360.0
        self.errorAcu = 0.0
        self.errorPre = 0.0
        self.get_logger().info(f"New Target Depth: {self.targetAngle:.2f} m")
    def anglePublish(self):
        self.PIDController(self.targetAngle, self.get_parameter("kP").value, self.get_parameter("kI").value, self.get_parameter("kD").value)
        heading_msg = Float32()
        heading_msg.data = self.currentAngle
        self.anglePub.publish(heading_msg)
        self.publishSpeed()
        self.get_logger().info(f"Published PID, angle {self.currentAngle}")
    def publishSpeed(self):
        speed_msg = Float32()
        speed_msg.data = max(-self.get_parameter("speed").value, min(self.get_parameter("speed").value, self.targetSpeed))
        self.for_pub.publish(speed_msg)
    def PIDController(self, target, kP, kI, kD):
        msg = ManualControl()
        error = (target - self.currentAngle + 180) % 360 - 180
        self.errorAcu += error * self.dt
        self.currentkP = kP * error
        self.currentkI = kI * max(min(self.errorAcu, 20.0), -20.0)
        self.currentkD = kD * (error - self.errorPre) / self.dt
        self.errorPre = error
        msg.x = max(self.get_parameter("min_thrust").value, min(self.get_parameter("max_thrust").value, self.targetSpeed))
                
        speed_fraction = abs(msg.x) / self.get_parameter("max_thrust").value
        scalar = max(0.6, 1.0 - 0.4 * speed_fraction)
        self.processVar = (self.currentkP + self.currentkI + self.currentkD) * scalar
        
        
        
        
        
        msg.r = max(self.get_parameter("min_thrust").value, min(self.get_parameter("max_thrust").value, self.processVar))
            
            
        
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
