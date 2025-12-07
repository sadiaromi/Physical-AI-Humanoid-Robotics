# action_server_manip.py

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

# Mock Action Interface
class ManipulateObjectAction:
    class Goal:
        def __init__(self, action_type, object_name):
            self.action_type = action_type
            self.object_name = object_name
    class Result:
        def __init__(self):
            self.success = False

class ManipActionServer(Node):

    def __init__(self):
        super().__init__('manip_action_server')
        self._action_server = ActionServer(
            self,
            ManipulateObjectAction,
            'manipulate_object',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing manipulation goal: {goal_handle.request.action_type} {goal_handle.request.object_name}')
        
        # Simulate work
        import time
        time.sleep(1)

        goal_handle.succeed()

        result = ManipulateObjectAction.Result()
        result.success = True
        return result

def main(args=None):
    rclpy.init(args=args)
    node = ManipActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
