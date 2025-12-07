from example_interfaces.srv import SetJointAngles
import rclpy
from rclpy.node import Node

class MinimalSetJointAnglesService(Node):

    def __init__(self):
        super().__init__('minimal_set_joint_angles_service')
        self.srv = self.create_service(SetJointAngles, 'set_joint_angles', self.set_joint_angles_callback)
        self.get_logger().info('SetJointAngles service is ready.')

    def set_joint_angles_callback(self, request, response):
        self.get_logger().info('Incoming request: joint_angles: %s' % request.joint_angles)

        # In a real robot, this is where you would interface with the robot's hardware
        # to set the joint angles. For this example, we'll just simulate success/failure.
        if len(request.joint_angles) > 0: # Basic validation
            response.success = True
            response.message = 'Joint angles received and processed successfully.'
            self.get_logger().info('Joint angles set: %s' % request.joint_angles)
        else:
            response.success = False
            response.message = 'No joint angles provided.'
            self.get_logger().warn('No joint angles received.')

        return response

def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalSetJointAnglesService()

    rclpy.spin(minimal_service)

    minimal_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
