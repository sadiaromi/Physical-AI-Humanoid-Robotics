# action_server_nav.py

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

# Mock Action Interface
class NavigateToAction:
    class Goal:
        def __init__(self, location):
            self.location = location
    class Result:
        def __init__(self):
            self.success = False

class NavActionServer(Node):

    def __init__(self):
        super().__init__('nav_action_server')
        self._action_server = ActionServer(
            self,
            NavigateToAction,
            'navigate_to',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info(f'Executing navigation goal: Go to {goal_handle.request.location}')
        
        # Simulate work
        import time
        time.sleep(2)

        goal_handle.succeed()

        result = NavigateToAction.Result()
        result.success = True
        return result

def main(args=None):
    rclpy.init(args=args)
    node = NavActionServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
