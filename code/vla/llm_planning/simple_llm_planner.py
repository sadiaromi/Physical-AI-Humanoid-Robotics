# simple_llm_planner.py
#
# Description:
# This script demonstrates a simplified approach to LLM-based task planning
# for a robot. It serves as the foundational code for Chapter 13 of the
# Physical AI & Humanoid Robotics book.
#
# The script simulates the following process:
# 1. A user gives a high-level command (e.g., "Get me a snack").
# 2. The system defines the robot's current state and available actions.
# 3. A prompt is constructed for an LLM, asking it to create a plan.
# 4. A mock LLM response is used to simulate the output of a real LLM.
# 5. The plan is parsed from the LLM's response.
# 6. The robot simulates the execution of the plan, step by step.
#
# Dependencies:
# - openai: This script is structured to use the OpenAI API, though the
#           actual API call is mocked out for demonstration purposes.
#
# Installation:
# pip install openai
#
# To run with a real LLM, you would need to:
# 1. Have an OpenAI API key.
# 2. Set the API key in your environment variables.
# 3. Replace the `mock_llm_call` function with a real API call.

import json
import re

# --- Mock Robot and Environment ---
class Robot:
    """A mock robot class to simulate state and actions."""
    def __init__(self):
        self.state = {
            "location": "living_room",
            "hands_free": True,
            "inspected_objects": {}
        }
        print(f"Robot initialized. Current state: {self.state}")

    def navigate_to(self, location):
        print(f"Executing: navigate_to('{location}')...")
        self.state["location"] = location
        print(f"  Success. Robot is now in the '{location}'.")
        return f"Successfully navigated to {location}."

    def find_object(self, obj_description):
        print(f"Executing: find_object('{obj_description}')...")
        # Simulate finding an object based on location
        if self.state["location"] == "kitchen" and "snack" in obj_description:
            self.state["inspected_objects"][obj_description] = {"location": "pantry"}
            print(f"  Success. Found '{obj_description}' in the 'pantry'.")
            return f"Found {obj_description} in pantry."
        print(f"  Failed. Could not find '{obj_description}'.")
        return f"Could not find {obj_description}."
    
    def pick_up(self, obj):
        print(f"Executing: pick_up('{obj}')...")
        if self.state["hands_free"]:
            self.state["hands_free"] = False
            print(f"  Success. Robot is now holding '{obj}'.")
            return f"Successfully picked up {obj}."
        print(f"  Failed. Robot's hands are not free.")
        return "Hands are not free."

# --- LLM Planner ---
class LLMPlanner:
    """A class to handle LLM-based planning."""
    def __init__(self, robot):
        self.robot = robot
        self.available_actions = {
            "navigate_to": self.robot.navigate_to,
            "find_object": self.robot.find_object,
            "pick_up": self.robot.pick_up
        }

    def construct_prompt(self, command):
        """Constructs a prompt for the LLM."""
        prompt = f"""
        You are a helpful robot assistant. Your goal is to create a plan to fulfill the user's command.
        
        User Command: "{command}"
        
        Current Robot State:
        {json.dumps(self.robot.state, indent=2)}
        
        Available Actions (Functions):
        - navigate_to(location: str): Moves the robot to a specified location.
        - find_object(object_description: str): Locates an object in the current area.
        - pick_up(object: str): Picks up a specified object.
        
        Instructions:
        Generate a JSON array of steps to accomplish the command. Each step should be a dictionary with "action" and "arguments".
        Use ONLY the available actions.
        
        Example Plan:
        [
            {{"action": "navigate_to", "arguments": ["kitchen"]}},
            {{"action": "find_object", "arguments": ["apple"]}},
            {{"action": "pick_up", "arguments": ["apple"]}}
        ]
        
        Generate the plan now.
        """
        return prompt

    def mock_llm_call(self, prompt):
        """Mocks the response from an LLM API call."""
        print("\n--- Sending Prompt to Mock LLM ---")
        print(prompt)
        print("--- Mock LLM Response ---")
        
        # This is a hardcoded response for the command "Get me a snack"
        mock_response = """
        [
            {"action": "navigate_to", "arguments": ["kitchen"]},
            {"action": "find_object", "arguments": ["a snack"]},
            {"action": "pick_up", "arguments": ["a snack"]}
        ]
        """
        print(mock_response)
        return mock_response

    def parse_plan(self, response_text):
        """Parses the JSON plan from the LLM's response."""
        try:
            # Use regex to find the JSON part of the response, in case the LLM adds extra text
            json_match = re.search(r'[[].*[]]', response_text, re.DOTALL)
            if json_match:
                plan = json.loads(json_match.group(0))
                print("\nPlan parsed successfully.")
                return plan
            return []
        except json.JSONDecodeError:
            print("Error: Failed to decode JSON from LLM response.")
            return []

    def execute_plan(self, plan):
        """Executes the parsed plan step by step."""
        print("\n--- Executing Plan ---")
        if not plan:
            print("Cannot execute an empty or invalid plan.")
            return
            
        for i, step in enumerate(plan):
            action_name = step.get("action")
            args = step.get("arguments", [])
            
            print(f"\nStep {i+1}: {action_name}({', '.join(map(str, args))})")
            
            if action_name in self.available_actions:
                action_function = self.available_actions[action_name]
                # Call the corresponding robot function
                result = action_function(*args)
                print(f"  Result: {result}")
            else:
                print(f"  Error: Action '{action_name}' is not a known robot action.")
                break # Stop execution if an action is invalid
        print("\n--- Plan Execution Finished ---")
        print(f"Robot's final state: {self.robot.state}")

# --- Main Execution Block ---
def main():
    """
    Main function to run the LLM-based planning simulation.
    """
    # 1. Initialize the robot and planner
    robot = Robot()
    planner = LLMPlanner(robot)
    
    # 2. Get user command
    user_command = "Get me a snack from the kitchen"
    print(f"\nUser command: '{user_command}'")
    
    # 3. Construct the prompt
    prompt = planner.construct_prompt(user_command)
    
    # 4. Get response from (mock) LLM
    llm_response = planner.mock_llm_call(prompt)
    
    # 5. Parse the plan from the response
    plan = planner.parse_plan(llm_response)
    
    # 6. Execute the plan
    if plan:
        planner.execute_plan(plan)
    else:
        print("Could not generate or parse a valid plan.")

if __name__ == "__main__":
    main()
