import sys

from example_interfaces.srv import SetJointAngles
import rclpy
from rclpy.node import Node

class MinimalSetJointAnglesClient(Node):

    def __init__(self):
        super().__init__('minimal_set_joint_angles_client')
        self.cli = self.create_client(SetJointAngles, 'set_joint_angles')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetJointAngles.Request()

    def send_request(self, joint_angles):
        self.req.joint_angles = joint_angles
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalSetJointAnglesClient()

    # Example: Send joint angles [0.0, 1.57, -1.57] for shoulder, elbow, etc.
    # In a real AI agent, these would come from an AI planning module.
    joint_angles_to_send = [float(arg) for arg in sys.argv[1:]]
    if not joint_angles_to_send:
        minimal_client.get_logger().error('Please provide joint angles as arguments (e.g., 0.0 1.57 -1.57)')
        rclpy.shutdown()
        return

    response = minimal_client.send_request(joint_angles_to_send)
    if response.success:
        minimal_client.get_logger().info(
            'Service call successful: %s' % response.message)
    else:
        minimal_client.get_logger().error(
            'Service call failed: %s' % response.message)

    minimal_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
