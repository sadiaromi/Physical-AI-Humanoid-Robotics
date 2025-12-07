# simple_bipedal_nav_example.py
#
# Description:
# This script is a conceptual example demonstrating how a bipedal robot
# might interact with the ROS 2 Navigation2 stack in simulation. It outlines
# sending navigation goals and receiving feedback, without implementing
# the full complexity of bipedal locomotion control.
# It serves as the foundational code for Chapter 11 of the Physical AI & Humanoid Robotics book.
#
# A full implementation would involve complex gait generators, balance control,
# and specialized plugins for Navigation2 that understand bipedal dynamics.
# This example focuses on the high-level ROS 2 communication.
#
# Dependencies:
# - ROS 2 Humble Hawksbill
# - rclpy
# - nav2_msgs (conceptual usage for action types)
#
# Installation:
# Ensure ROS 2 Humble is installed and sourced. This script is meant to illustrate
# concepts; actual execution with a full Navigation2 stack would require additional setup.

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.action.client import ClientGoalHandle

# Conceptual import for Navigation2's NavigateToPose Action
# In a real setup, you would have:
# from nav2_msgs.action import NavigateToPose
class NavigateToPose:
    """Conceptual NavigateToPose Action Interface."""
    class Goal:
        def __init__(self):
            # Target pose for the robot to reach
            from geometry_msgs.msg import PoseStamped
            self.pose = PoseStamped()
            self.pose.header.frame_id = 'map' # Assuming navigation in a map frame
            self.pose.pose.position.x = 0.0
            self.pose.pose.position.y = 0.0
            self.pose.pose.orientation.w = 1.0
            
    class Result:
        def __init__(self):
            self.result_code = 0 # 0 for success, non-zero for failure
            
    class Feedback:
        def __init__(self):
            from geometry_msgs.msg import PoseStamped
            self.current_pose = PoseStamped()
            self.current_pose.header.frame_id = 'map'
            self.navigation_time = rclpy.time.Time().to_msg()
            self.number_of_recoveries = 0

class BipedalNavigationClient(Node):
    def __init__(self):
        super().__init__('bipedal_navigation_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Bipedal Navigation Client Node initialized.")

    def send_goal(self, x, y, yaw):
        self.get_logger().info(f"Waiting for action server: 'navigate_to_pose'")
        self._action_client.wait_for_server()

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        # Convert yaw to quaternion (simple 2D orientation)
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, yaw)
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        self.get_logger().info(f"Sending goal to ({x}, {y}, {yaw})")
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle: ClientGoalHandle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Goal finished with result: {result.result_code}')
        # In a real application, you might process the result code more
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: Current Pose (x: {feedback_msg.feedback.current_pose.pose.position.x:.2f}, y: {feedback_msg.feedback.current_pose.pose.position.y:.2f})')
        # Here, a bipedal robot would update its gait or balance based on feedback


def main(args=None):
    rclpy.init(args=args)

    node = BipedalNavigationClient()
    
    # Example: send a conceptual navigation goal
    # A full Navigation2 stack would be running with a planner and controller
    node.send_goal(x=5.0, y=0.0, yaw=0.0)

    rclpy.spin(node) # Keeps the node alive to process callbacks

if __name__ == '__main__':
    main()
