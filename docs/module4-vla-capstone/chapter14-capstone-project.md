---
id: chapter14-capstone-project
title: "Chapter 14: Capstone Project - The Autonomous Humanoid"
sidebar_label: "Chapter 14: Capstone Project"
---

## Introduction

This is it—the culmination of everything you have learned. In this capstone project, you will integrate the concepts and code from every previous module to build an end-to-end autonomous humanoid robot system. Your mission is to create a robot that can respond to a spoken command, understand its environment, formulate a plan, and execute it. This chapter will guide you through the architecture, integration, and evaluation of your final project.

## Learning Outcomes

By the end of this chapter, you will be able to:

*   Integrate multiple Python and ROS 2 components (voice recognition, LLM planning, action servers) into a single, cohesive application.
*   Design and implement a system architecture for an autonomous robot that follows the "listen, understand, plan, execute" paradigm.
*   Develop a ROS 2 launch file to start and manage multiple nodes simultaneously.
*   Create a central orchestrator node that can call other Python modules and send goals to ROS 2 action servers.
*   Demonstrate an end-to-end understanding of the course material by successfully running a complex, multi-step robotics task in simulation.
*   Critically evaluate a robotics project based on a comprehensive set of criteria, including functionality, code quality, design, and documentation.

## Required Skills and Tools

### Prerequisite Skills

*   **All skills from previous chapters are required.** This project is cumulative.
*   **Strong Debugging Skills**: You will need to troubleshoot issues that span multiple, interconnected systems (Python scripts, ROS 2 nodes, simulation).
*   **Code Integration**: The ability to read and understand code from different modules and integrate them into a coherent whole.

### Tools & Software

*   A complete **ROS 2 Humble** installation.
*   A working **Gazebo** or **Unity** simulation environment, configured to work with ROS 2.
*   A Python environment with all libraries from previous chapters installed:
    *   `openai`
    *   `sounddevice`
    *   `scipy`
    *   `numpy`
*   The `colcon` build tool for compiling ROS 2 packages.
*   A working microphone.
*   Access to an LLM API (e.g., OpenAI).

## Weekly Breakdown

This project is larger than a single chapter and is expected to take 1-2 weeks.

*   **Week 1: System Integration and ROS 2 Development**
    *   **Activity**: Focus on the ROS 2 backend. Create the `robot_actions` and `capstone_bringup` packages. Implement the action servers for navigation and manipulation. Build and test the ROS 2 workspace to ensure you can launch the servers and call them with simple test scripts.
    *   **Assessment**: Successfully launch your action servers using a ROS 2 launch file. Manually send a goal to each action server using the `ros2 action send_goal` command-line tool and verify that they receive the goal and execute their (simulated) task.

*   **Week 2, Day 1-4: Orchestrator and End-to-End Testing**
    *   **Activity**: Implement the `main_orchestrator.py` script. Integrate the voice recognition and LLM planning modules from the previous chapters. Write the client logic for the orchestrator to parse the plan and send goals to your ROS 2 action servers.
    *   **Assessment**: Run the entire system end-to-end. Give a voice command and watch the logs to see the robot's orchestrator successfully call the action servers in the correct sequence to complete the task.

*   **Week 2, Day 5-7: Documentation and Submission**
    *   **Activity**: Write the final project documentation, including the `README.md` and a short report summarizing your project's design, challenges, and results. Clean up your code and ensure it is well-commented.
    *   **Assessment**: Submit your complete `code/capstone` directory, including the working code, ROS 2 workspace, and all documentation. The project will be graded based on the evaluation criteria outlined in this chapter.

## Lab Setup Requirements

This capstone project requires a comprehensive setup that combines the environments from all previous modules. A detailed, step-by-step guide can be found in the project's code directory at `code/capstone/docs/setup_guide.md`. The key requirements are summarized below.

1.  **Simulation Environment**:
    *   You must have a working **Gazebo** or **Unity** simulation set up, as covered in Module 2.
    *   Your simulation world must contain a humanoid robot model that can be controlled via ROS 2.
    *   The world should be populated with the locations and objects required for your chosen task (e.g., a "desk", a "kitchen", a "water bottle" object).

2.  **ROS 2 Humble Environment**:
    *   Ensure your ROS 2 Humble installation is active (`source /opt/ros/humble/setup.bash`).
    *   You will need at least two terminals to run the project. It is recommended to use a terminal multiplexer like `tmux` or `screen` for convenience.

3.  **Python Virtual Environment**:
    *   A single Python virtual environment should be created for the project.
    *   This environment must have all the required packages installed:
        ```bash
        pip install openai sounddevice scipy numpy
        ```
    *   Remember to activate this environment (`source venv/bin/activate`) in the terminal where you run the main orchestrator.

4.  **API Keys**:
    *   Your `OPENAI_API_KEY` (or key for another LLM provider) must be set as an environment variable for the terminal running the orchestrator.

5.  **Final Checklist before Running**:
    *   [ ] Is your Gazebo/Unity simulation launched and running?
    *   [ ] Have you built the capstone's ROS 2 workspace (`code/capstone/ros2_ws`) with `colcon build`?
    *   [ ] In Terminal 1, have you sourced both your main ROS 2 setup and the capstone workspace (`install/setup.bash`)?
    *   [ ] In Terminal 2, have you activated your Python virtual environment?
    *   [ ] In Terminal 2, is your `OPENAI_API_KEY` environment variable exported?

## Capstone Project: The Autonomous Humanoid

### Project Goal

The goal of this project is to build and demonstrate a robotic system that can perform the following sequence:

1.  **Listen**: Receive a spoken command from a user (e.g., "Bring me the water bottle from the desk").
2.  **Understand**: Transcribe the voice command to text and parse it to determine the user's intent and the key entities (e.g., action: `FETCH`, object: `water bottle`, source: `desk`).
3.  **Plan**: Generate a multi-step plan to achieve the goal, using knowledge of the robot's capabilities and its environment.
4.  **Execute**: Carry out the plan by calling the appropriate ROS 2 actions for navigation and manipulation in a simulated environment.
5.  **Adapt (Optional)**: For a more advanced implementation, the robot should be able to react to its environment, such as navigating around an unexpected obstacle.

### System Architecture: Integrating the Modules

Your capstone project will be a master application that orchestrates the components you built in previous chapters. The data will flow through the system as follows:

1.  **Voice Command Input (Chapter 12)**:
    *   The `simple_whisper_integration.py` script captures and transcribes your voice command.
    *   The transcribed text is passed to the planning module.

2.  **LLM-based Planning (Chapter 13)**:
    *   The `simple_llm_planner.py` script takes the transcribed text as its input command.
    *   It constructs a prompt, queries the LLM (mock or real), and parses the response to get a structured plan (a list of actions).

3.  **ROS 2 Action Execution (Chapter 4 & Others)**:
    *   A new "Orchestrator" node in your ROS 2 system will be responsible for executing the plan.
    *   For each step in the plan (e.g., `{"action": "navigate_to", "arguments": ["desk"]}`), the orchestrator will call the corresponding ROS 2 Action Client.
    *   You will need to have ROS 2 Action Servers running for each capability (e.g., a navigation server, a manipulation server). These servers will perform the actual work in the simulation.

4.  **Simulation Environment (Module 2)**:
    *   The entire task will be performed within a Gazebo or Unity simulation environment that you design.
    *   This environment should contain the necessary objects and locations for your chosen task (e.g., a desk, a kitchen, a water bottle).
    *   The ROS 2 servers will interact with the simulated robot and environment.

### Guidance for Optional Physical Deployment

For those with access to physical hardware, deploying this system outside of a simulation is the ultimate challenge. Key considerations include:

*   **Hardware Abstraction**: Your ROS 2 Action Servers should be written in a way that they can either control a simulated robot or a real one. The "Orchestrator" node should not need to know the difference.
*   **Sim-to-Real Transfer**: This is a major field of robotics. You will encounter challenges with sensor noise, motor inaccuracies, and environmental differences. Start simple and iterate.
*   **Safety**: Implement safety stops and be prepared to take manual control at any time. Real robots can be dangerous if they behave unexpectedly.

### Evaluation Criteria

Your capstone project will be evaluated based on the following criteria:

*   **Functionality (60%)**:
    *   Does the system successfully execute the full listen-understand-plan-execute loop?
    *   Can it handle a variety of commands within its known capabilities?
    *   Is the plan generated by the LLM logical and efficient?
    *   Does the robot successfully complete the task in the simulation?
*   **Code Quality & Design (20%)**:
    *   Is the code well-structured, readable, and commented?
    *   Is the system modular (i.e., are the different components loosely coupled)?
    *   Is the ROS 2 architecture (nodes, topics, actions) well-designed?
*   **Documentation (10%)**:
    *   A `README.md` file that clearly explains how to set up and run your project.
    *   A brief report summarizing your design choices, the challenges you faced, and the final outcome.
*   **Robustness (10%)**:
    *   How well does the system handle minor variations in commands?
    *   (Advanced) Does the system have any error handling or recovery mechanisms?

Good luck! This project is your opportunity to demonstrate a comprehensive understanding of modern, AI-driven humanoid robotics.
