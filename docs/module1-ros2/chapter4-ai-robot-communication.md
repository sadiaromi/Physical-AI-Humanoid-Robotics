---
id: chapter4-ai-robot-communication
title: "Chapter 4: Communication between AI Agents and Robot Controllers"
sidebar_label: "Chapter 4: AI-Robot Comm"
---

## Introduction

In the previous chapters, we established our ROS 2 environment, understood its core communication primitives, and learned how to describe a robot's physical structure using URDF. Now, we bring these concepts together to tackle a crucial aspect of Physical AI: enabling intelligent AI agents to communicate effectively with the robot's low-level controllers.

This chapter focuses on designing and implementing robust ROS 2 interfaces that allow high-level AI commands (such as "move arm to target") to be translated into specific actions executed by the robot. We will explore how to create custom ROS 2 message and service types tailored for AI-robot interaction and provide a concrete example of an AI agent sending target joint angles to a robot controller via a ROS 2 service. This forms a vital bridge between the "brain" (AI) and the "body" (robot).

## Learning Outcomes

By the end of this chapter, you will be able to:

*   Understand the principles of abstract AI commands and how they translate to low-level robot control through ROS 2 interfaces.
*   Design custom ROS 2 message and service types using `.srv` files to facilitate specific AI-robot interactions.
*   Implement a ROS 2 service server to receive and process AI commands (e.g., setting joint angles) from an AI agent.
*   Implement a ROS 2 service client to send AI commands and handle responses from robot controllers.
*   Explain how Topics, Services, and Actions can be leveraged to create a robust and layered control architecture for intelligent robots.

## Required Skills and Tools

### Prerequisite Skills

*   **Solid Python 3 Programming**: You should be comfortable with Python syntax, functions, classes, and basic data structures for implementing ROS 2 nodes.
*   **Familiarity with ROS 2 Core Concepts**: A strong understanding of Nodes, Topics, Services, and Actions from Chapter 2 is essential.
*   **Basic ROS 2 Package Development**: You should know how to create new ROS 2 packages and build them using `colcon`.
*   **Basic Git Usage**: Familiarity with basic `git` commands for managing code.

### Tools & Software

*   **ROS 2 Humble Hawksbill**: A fully functional installation on Ubuntu 22.04.
*   **Code Editor**: A code editor (e.g., VS Code, Sublime Text, PyCharm) for creating and editing Python and `.srv` files.
*   **ROS 2 Workspace**: A configured ROS 2 workspace where you can create and build custom packages.
*   **Terminal**: A terminal with your ROS 2 environment and workspace sourced.

## Weekly Breakdown

This chapter is designed to be completed within one week.

*   **Day 1-2: Understanding AI-Robot Communication & Custom Services**
    *   **Activity**: Read the sections "ROS 2 Interfaces for High-Level AI Commands to Low-Level Robot Controllers" and "Designing Custom ROS 2 Messages/Services for AI-Robot Interaction." Focus on the `SetJointAngles.srv` definition and how it bridges the AI command to robot action.
    *   **Assessment**: In a new ROS 2 package (which you should create), define a custom service (e.g., `SetGripperState.srv` with a boolean request for `open/close` and a boolean response for `success/message`). Ensure the package builds correctly.

*   **Day 3-5: Implementing Service Server and Client**
    *   **Activity**: Implement the `set_joint_angles_server.py` and `set_joint_angles_client.py` examples. If you created your own custom service, implement a server and client for that. Build your package containing these nodes.
    *   **Assessment**: Successfully run the service server and use the client to send joint angle commands (or your custom service commands). Verify the server logs show the incoming request and the client logs show the response, indicating successful communication.

*   **Day 6-7: Experimentation with ROS 2 CLI and Communication Patterns**
    *   **Activity**: Use ROS 2 CLI tools (e.g., `ros2 service list`, `ros2 service type`, `ros2 service call`) to interact with your running `SetJointAngles` service (or your custom service). Experiment with sending different joint angle values or gripper states.
    *   **Assessment**: Document a brief scenario where a ROS 2 Action (as introduced in Chapter 2) would be a more suitable communication pattern than a Service for a different AI-robot interaction, explaining your reasoning with respect to feedback and preemption.

## Lab Setup Requirements

The lab setup for this chapter continues to use the ROS 2 Humble environment configured in Chapter 1 and builds upon the ROS 2 workspace from Chapter 2.

1.  **ROS 2 Humble Environment**:
    *   Ensure your ROS 2 Humble Hawksbill environment is fully installed and verified.
    *   Make sure you have sourced the ROS 2 setup file in each terminal you intend to use for running ROS 2 commands.

2.  **ROS 2 Workspace**:
    *   You will need a ROS 2 workspace (e.g., `~/ros2_ws`) where you have created your packages in previous chapters. Ensure this workspace is sourced in all your development terminals.

3.  **Custom Message/Service Package**:
    *   You will need to create a new ROS 2 package (e.g., `ai_interface_msgs`) to define your custom `.srv` file. Use `ros2 pkg create --build-type ament_cmake ai_interface_msgs`.
    *   Remember to add `build_type ament_cmake` to the `package.xml` and include the service definition in `CMakeLists.txt` for your package.

4.  **Verification**:
    *   The successful creation of a custom `.srv` file within a ROS 2 package, followed by the ability to build that ROS 2 package (`colcon build`), is the first verification step.
    *   The successful running of the service server and client, verifying communication via your custom service type, confirms the lab setup.

## ROS 2 Interfaces for High-Level AI Commands to Low-Level Robot Controllers

The challenge in AI-robot communication lies in bridging the gap between abstract AI goals and concrete robot movements.

*   **High-Level AI Commands**: These are typically human-understandable instructions or high-level decisions from an AI planner (e.g., "pick up the red ball", "navigate to the charging station", "wave hello").
*   **Low-Level Robot Control**: This involves sending precise commands to individual actuators (e.g., joint positions, motor velocities, gripper force).

ROS 2 provides an excellent framework for defining these interfaces. We can use Topics, Services, and Actions to create a layered control architecture:

*   **Topics**: Suitable for streaming frequent, continuous data (e.g., AI agent publishing desired end-effector pose continuously).
*   **Services**: Ideal for single-shot, blocking requests (e.g., AI agent commanding "open gripper" and waiting for confirmation).
*   **Actions**: Best for long-running, goal-oriented tasks that require feedback and potentially preemption (e.g., AI agent sending a "move_to_pose" action goal and receiving feedback on current pose).

For AI-robot communication, Services and Actions are often preferred when the AI needs to ensure a specific task is completed before proceeding.

## Designing Custom ROS 2 Messages/Services for AI-Robot Interaction

While ROS 2 provides many standard message types (`std_msgs`, `geometry_msgs`, etc.), complex AI-robot interactions often require custom data structures. ROS 2 allows you to define your own message, service, and action types.

### Defining a Custom Service Example: `SetJointAngles.srv`

Let's consider an AI agent that wants to command a robot's arm to move to specific joint angles. We can define a custom service for this.

Create a `.srv` file (e.g., `SetJointAngles.srv`) in a ROS 2 package:

```
# Request
float64[] joint_angles
---
# Response
bool success
string message
```

This service definition specifies:
*   **Request**: An array of `float64` values representing the target joint angles.
*   **Response**: A boolean `success` indicating if the command was accepted/executed, and a `string` message for additional details.

Once defined, ROS 2 will automatically generate code (in Python, C++, etc.) that allows nodes to use this custom service type.

## Example: AI Agent Sending Target Joint Angles via a ROS 2 Service

We will implement a simple example where an AI agent (represented by a ROS 2 client node) sends a `SetJointAngles` request to a robot controller (represented by a ROS 2 service server node).

### Service Server (`set_joint_angles_server.py`)

The service server will:
1.  Listen for `SetJointAngles` requests.
2.  Receive the `joint_angles` array.
3.  Simulate the movement to these angles (e.g., with a `time.sleep`).
4.  Send back a `success` and `message` in the response.

### Service Client (`set_joint_angles_client.py`)

The service client will:
1.  Create a `SetJointAngles` request with target angles.
2.  Send the request to the server.
3.  Wait for and process the response.

This interaction demonstrates a clean, ROS 2-native way for an AI agent to issue high-level commands to a robot's actuators. In future modules, this basic communication will be expanded to incorporate more sophisticated planning and perception.