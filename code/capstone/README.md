# Capstone Project: The Autonomous Humanoid

This directory contains the full codebase for the Chapter 14 Capstone Project. It integrates all the concepts from the book into a single, functional system.

## Project Overview

The system is designed to take a spoken command, generate a plan using an LLM, and execute that plan in a ROS 2-based simulation.

The main components are:
-   **Main Orchestrator**: A Python script that controls the overall flow of the application.
-   **Voice Recognition**: The Whisper integration from Chapter 12.
-   **LLM Planner**: The LLM-based planner from Chapter 13.
-   **ROS 2 Actions**: A ROS 2 workspace containing action servers for robot navigation and manipulation.

## Directory Structure

```
code/capstone/
├── README.md
├── main_orchestrator.py
├── config/
│   └── params.yaml
├── scripts/
│   └── run_project.sh
└── ros2_ws/
    └── src/
        ├── capstone_bringup/
        │   ├── package.xml
        │   └── launch/
        │       └── capstone_launch.py
        └── robot_actions/
            ├── package.xml
            └── robot_actions/
                ├── __init__.py
                ├── action_server_nav.py
                └── action_server_manip.py
```

## How to Run

1.  **Build the ROS 2 Workspace**:
    -   Navigate to `code/capstone/ros2_ws`.
    -   Build the workspace: `colcon build`
    -   Source the workspace: `source install/setup.bash`

2.  **Launch the ROS 2 Action Servers**:
    -   In a new terminal, launch the action servers:
        ```bash
        ros2 launch capstone_bringup capstone_launch.py
        ```

3.  **Run the Main Orchestrator**:
    -   In another terminal, ensure your Python environment with `openai`, `sounddevice`, etc., is activated.
    -   Ensure your `OPENAI_API_KEY` is set as an environment variable.
    -   Run the main script:
        ```bash
        python main_orchestrator.py
        ```

4.  **Follow the Prompts**:
    -   The script will prompt you to speak a command.
    -   Watch as the system transcribes your speech, generates a plan, and executes it by calling the ROS 2 actions.
