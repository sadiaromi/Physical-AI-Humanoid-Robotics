# Detailed Setup Guide

This guide provides detailed step-by-step instructions to get the capstone project running.

## Prerequisites

-   Ubuntu 22.04 with ROS 2 Humble installed.
-   Python 3.8+
-   A working microphone.
-   An API key from an LLM provider like OpenAI (optional but recommended).

## Step 1: Set Up the Python Environment

1.  **Navigate to the project root**:
    ```bash
    cd /path/to/your/humanoid-robotics-book
    ```

2.  **Create and activate a Python virtual environment**:
    ```bash
    python3 -m venv venv
    source venv/bin/activate
    ```

3.  **Install Python dependencies**:
    ```bash
    pip install openai sounddevice scipy numpy
    ```

4.  **Set your API Key (Optional)**:
    -   Set your `OPENAI_API_KEY` as an environment variable to use a live LLM.
    ```bash
    export OPENAI_API_KEY='your-api-key-here'
    ```

## Step 2: Build the ROS 2 Workspace

1.  **Navigate to the capstone ROS 2 workspace**:
    ```bash
    cd code/capstone/ros2_ws
    ```

2.  **Build the workspace with colcon**:
    -   `colcon` is the standard build tool for ROS 2.
    ```bash
    colcon build
    ```

3.  **Source the workspace**:
    -   After building, you need to source the `setup.bash` file to make the ROS 2 packages available in your terminal.
    ```bash
    source install/setup.bash
    ```
    *Note: You will need to do this in every new terminal you open to interact with the ROS 2 nodes.*

## Step 3: Run the Project

You will need two terminals for this.

### Terminal 1: Launch the ROS 2 Action Servers

1.  **Navigate to the project root.**
2.  **Source your ROS 2 and workspace setup files**:
    ```bash
    source /opt/ros/humble/setup.bash
    source code/capstone/ros2_ws/install/setup.bash
    ```
3.  **Launch the action servers**:
    ```bash
    ros2 launch capstone_bringup capstone_launch.py
    ```
-   You should see output indicating that the `nav_action_server` and `manip_action_server` have started and are waiting for goals.

### Terminal 2: Run the Main Orchestrator

1.  **Navigate to the project root.**
2.  **Activate your Python virtual environment**:
    ```bash
    source venv/bin/activate
    ```
3.  **Navigate to the capstone code directory**:
    ```bash
    cd code/capstone
    ```
4.  **Run the main script**:
    ```bash
    python main_orchestrator.py
    ```

The script will now run through the full loop:
-   It will use the mock voice recognition to get a command.
-   It will use the mock LLM planner to generate a plan.
-   It will then connect to the ROS 2 action servers you started in Terminal 1 and send them goals.
-   You will see the log output from both the orchestrator and the action servers as the plan is executed.
