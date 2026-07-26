import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class Armed(Node):
    def __init__(self):
        super().__init__("arming_service")

        self.arm_client = self.create_client(
            SetBool,
            "/arming"
        )

        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for arming service...")

    def respond(self):
        request = SetBool.Request()
        request.data = True

        future = self.arm_client.call_async(request)

        rclpy.spin_until_future_complete(self, future)

        self.response = future.result()

        if self.response.success:
            self.get_logger().info("Vehicle armed!")
        else:
            self.get_logger().error(self.response.message)


def main(args=None):
    rclpy.init(args=args)

    node = Armed()

    node.respond()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
