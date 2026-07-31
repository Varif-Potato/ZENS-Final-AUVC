import rclpy
from rclpy.node import Node
from mavros_msgs.msg import ManualControl
from sensor_msgs.msg import FluidPressure
from std_msgs.msg import Float32


class depthhold(Node):
    def __init__(self):
        super().__init__("depthhold")
        self.declare_parameter("kP", 50.0)
        self.declare_parameter("kI", 2.0)
        self.declare_parameter("kD", 0.75)
        self.declare_parameter("max_depth", -3.00)
        self.declare_parameter("min_depth", 0.00)
        self.declare_parameter("speed", 300.0)
        self.declare_parameter("target_depth", -2.0)
        self.currentPos = 0
        self.dt = 0.1
        self.targetPos = -0.0
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
            Float32,
            "/target_depth",
            self.targetSetting, 
            10
        )
        self.sub = self.create_subscription(
            FluidPressure,
            "/pressure",
            self.depthMeters,
            10
        )
        self.timer = self.create_timer(
            self.dt,
            self.depthPublish
        )
        
    def depthMeters(self, msg):
        self.meter = -((msg.fluid_pressure - 101325) / (1000.0 * 9.81))
        self.currentPos = 0.5 * self.meter + (0.5) * self.currentPos
        self.get_logger().info(f"Depth: {self.meter}, ProcessVar: {self.processVar}")
    def targetFor(self, msg):
        self.targetSpeed = msg.data
        self.get_logger().info(f"Target Speed {self.targetSpeed} %")
    def targetSetting(self, msg):
        self.targetPos = msg.data
        self.errorAcu = 0.0
        self.errorPre = 0.0
        self.get_logger().info(f"New Target Depth: {self.targetPos:.2f} m")
    def depthPublish(self):
        target_param = self.get_parameter("target_depth").value
        self.PIDController(
            max(min(target_param, self.get_parameter("min_depth").value), self.get_parameter("max_depth").value),
            self.get_parameter("kP").value,
            self.get_parameter("kI").value,
            self.get_parameter("kD").value
        )
        self.get_logger().info(f"Published PID, depth {self.meter} m")
        
    def PIDController(self, target, kP, kI, kD):
        msg = ManualControl()
        error = target - self.currentPos
        if abs(error) < 1.0:
            self.errorAcu += error * self.dt
        else:
            self.errorAcu = 0.0
        self.currentkP = kP * error
        self.currentkI = kI * max(min(self.errorAcu, 400.0), -400.0)
        self.currentkD = kD * (error - self.errorPre) / self.dt
        self.errorPre = error
        

        self.processVar = (self.currentkP + self.currentkI + self.currentkD) - 20.0
        
        msg.z = max(-self.get_parameter("speed").value, min(self.get_parameter("speed").value, self.processVar))
            
        
        self.manual_pub.publish(msg)
        self.get_logger().info(f"Process Var{self.processVar}")
        self.get_logger().info(
            f"Target={target:.2f}, Current={self.currentPos:.2f}, "
            f"Error={error:.2f}, Output={self.processVar:.2f}"
        )
def main(args=None):
    rclpy.init(args=args)
    node = depthhold()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nKeyboardInterrupt received, shutting down...")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()