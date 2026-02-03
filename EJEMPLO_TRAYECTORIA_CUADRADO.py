import rclpy
from rclpy.node import Node
from frhal_msgs.srv import ROSCmdInterface
import time

class CommandClient(Node):
    def __init__(self):
        super().__init__("command_client")
        self.client = self.create_client(ROSCmdInterface, "/FR_ROS_API_service")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting again...")

    def send_request_sync(self, command):
        req = ROSCmdInterface.Request()
        req.cmd_str = command
        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.done():
            try:
                response = future.result()
                return response.cmd_res
            except Exception as e:
                return None
        else:
            return None

def main(args=None):
    rclpy.init(args=args)
    command_client = CommandClient()
    commands = [
        'ResetAllError()',
        'CARTPoint(1,-149.000,476.000,530.000,167.000,54.000,-101.000)',
        'CARTPoint(2,-89.000,476.000,530.000,167.000,54.000,-101.000)',
        'CARTPoint(3,-89.000,506.000,530.000,167.000,54.000,-101.000)',
        'CARTPoint(4,-149.000,506.000,530.000,167.000,54.000,-101.000)',
        'CARTPoint(5,-149.000,476.000,530.000,167.000,54.000,-101.000)',
        'MoveL(CART1,8)',
        'MoveL(CART2,8)',
        'MoveL(CART3,8)',
        'MoveL(CART4,8)',
        'MoveL(CART5,8)',
    ]

    success_count = 0
    error_count = 0

    for i, command in enumerate(commands, 1):
        result = command_client.send_request_sync(command)
        if result is not None:
            if result == 1 or result == 0:
                success_count += 1
            else:
                error_count += 1
        else:
            error_count += 1
        time.sleep(0.1)

    command_client.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
