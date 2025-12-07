# System Architecture

This document details the architecture of the Autonomous Humanoid Capstone Project.

## Component Overview

The system is composed of three main parts:
1.  **The Orchestrator (`main_orchestrator.py`)**: The central "brain" of the operation.
2.  **The Perception & Planning Pipeline (Python Modules)**: The voice recognition and LLM planning scripts.
3.  **The Action Execution Backend (ROS 2)**: The ROS 2 nodes that control the robot in the simulation.

## Data Flow Diagram

```
[User Voice] -> [1. Voice Recognition] -> "Transcribed Text"
                                              |
                                              v
"Transcribed Text" -> [2. LLM Planner] -> [Structured Plan (JSON)]
                                              |
                                              v
[Structured Plan] -> [3. Orchestrator] -> [ROS 2 Action Goals]
                                              |
                                              v
[ROS 2 Action Goals] -> [4. ROS 2 Action Servers] -> [Robot Actions in Sim]
```

## 1. The Orchestrator

The `main_orchestrator.py` script is the entry point. It is responsible for:
-   Initializing the ROS 2 client node.
-   Calling the voice recognition module to get the user's command.
-   Passing the command to the LLM planner.
-   Receiving the structured plan.
-   Iterating through the plan and sending ROS 2 action goals to the appropriate servers.

## 2. Perception & Planning Pipeline

This pipeline consists of two main Python modules that are called by the orchestrator:

-   **`simple_whisper_integration.py`**:
    -   Records audio from the user.
    -   Uses the Whisper model to transcribe the audio into text.
    -   Returns the transcribed text.

-   **`simple_llm_planner.py`**:
    -   Takes the transcribed text as input.
    -   Constructs a detailed prompt for an LLM, including the command, the robot's state, and a list of its capabilities.
    -   Sends the prompt to the LLM and receives a plan.
    -   Parses the plan into a machine-readable format (JSON).
    -   Returns the structured plan.

## 3. Action Execution Backend (ROS 2)

This is a ROS 2 workspace containing the nodes that execute the robot's physical actions.

-   **`robot_actions` package**:
    -   `nav_action_server`: A ROS 2 node that provides a `NavigateTo` action. It takes a location (e.g., "kitchen") and simulates the robot moving to that location.
    -   `manip_action_server`: A ROS 2 node that provides a `ManipulateObject` action. It can perform various manipulations, such as "pick_up", "open", etc.

-   **`capstone_bringup` package**:
    -   Contains a launch file (`capstone_launch.py`) that starts up both the navigation and manipulation action servers, so they are ready to receive goals from the orchestrator.
