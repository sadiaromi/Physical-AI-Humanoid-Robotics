# main_orchestrator.py
#
# Description:
# The central orchestrator for the Capstone Project. This script ties together
# voice recognition, LLM-based planning, and ROS 2 action execution.

import os
import sys
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

# Assume the code from previous chapters is available in the python path
# In a real project, these would be installed as a library or part of the same package
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'vla')))
from whisper_integration.simple_whisper_integration import record_audio, save_audio_to_wav, transcribe_audio
from llm_planning.simple_llm_planner import LLMPlanner, Robot

# --- Mock Action Interfaces ---
# In a real system, these would be proper ROS 2 Action interfaces
class NavigateToAction:
    class Goal:
        def __init__(self, location):
            self.location = location

class ManipulateObjectAction:
    class Goal:
        def __init__(self, action_type, object_name):
            self.action_type = action_type
            self.object_name = object_name

# --- Main Orchestrator Node ---
class OrchestratorNode(Node):
    def __init__(self):
        super().__init__('capstone_orchestrator')
        self._nav_action_client = ActionClient(self, NavigateToAction, 'navigate_to')
        self._manip_action_client = ActionClient(self, ManipulateObjectAction, 'manipulate_object')

    def execute_plan(self, plan):
        self.get_logger().info("--- Starting Plan Execution ---")
        for step in plan:
            action_name = step.get("action")
            args = step.get("arguments", [])
            
            self.get_logger().info(f"Executing step: {action_name} with args {args}")

            if action_name == "navigate_to":
                self.send_nav_goal(args[0])
            elif action_name in ["pick_up", "open", "close", "place_object"]:
                self.send_manip_goal(action_name, args[0])
            else:
                self.get_logger().error(f"Unknown action: {action_name}")
        
        self.get_logger().info("--- Plan Execution Finished ---")

    def send_nav_goal(self, location):
        goal_msg = NavigateToAction.Goal(location=location)
        self._nav_action_client.wait_for_server()
        self.get_logger().info(f"Sending navigation goal: {location}")
        # In a real app, you would handle the future and result
        self._nav_action_client.send_goal_async(goal_msg)

    def send_manip_goal(self, action_type, object_name):
        goal_msg = ManipulateObjectAction.Goal(action_type=action_type, object_name=object_name)
        self._manip_action_client.wait_for_server()
        self.get_logger().info(f"Sending manipulation goal: {action_type} {object_name}")
        # In a real app, you would handle the future and result
        self._manip_action_client.send_goal_async(goal_msg)

def get_voice_command():
    """Records and transcribes a voice command."""
    # This is a simplified version of the Chapter 12 script
    # It assumes a Whisper model is available
    # For simplicity, we're returning a hardcoded command
    print("\n--- Voice Recognition ---")
    print("In a real run, this would record your voice.")
    # command = transcribe_audio(model, audio_file)
    command = "Bring me the water bottle from the desk"
    print(f"Recognized command: '{command}'")
    return command

def get_plan_from_llm(command):
    """Gets a plan from the LLM based on the command."""
    print("\n--- LLM Planning ---")
    robot = Robot() # Uses the mock robot from Chapter 13
    planner = LLMPlanner(robot)
    prompt = planner.construct_prompt(command)
    llm_response = planner.mock_llm_call(prompt) # Uses the mock LLM call
    plan = planner.parse_plan(llm_response)
    return plan

def main(args=None):
    rclpy.init(args=args)

    # 1. Get voice command
    command = get_voice_command()

    # 2. Get plan from LLM
    plan = get_plan_from_llm(command)

    if not plan:
        print("Failed to generate a plan. Exiting.")
        return

    # 3. Execute the plan with ROS 2
    orchestrator = OrchestratorNode()
    orchestrator.execute_plan(plan)

    # For this simulation, we don't spin the node.
    # In a real application, the orchestrator would likely be a long-running node.
    orchestrator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
