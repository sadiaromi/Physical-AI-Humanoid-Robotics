# Chapter 4: Communication between AI Agents and Robot Controllers

## ROS 2 Interfaces for High-Level AI Commands to Low-Level Robot Controllers

Integrating AI agents with robotic systems requires robust communication interfaces. ROS 2 provides a flexible framework for defining these interfaces, allowing high-level AI decision-making to translate into low-level robot actions. This typically involves a hierarchical control architecture where AI agents operate at a higher level of abstraction, sending commands or goals, while robot controllers handle the precise execution.

Key ROS 2 communication mechanisms for this integration include:

-   **Topics**: For streaming continuous data from sensors (e.g., AI vision output, state estimations) and for broadcasting simple, frequent commands (e.g., motor speed adjustments).
-   **Services**: For synchronous, request-response interactions. Ideal for AI agents requesting specific actions (e.g., "move arm to joint angles X, Y, Z", "pick up object A") or querying the robot's current status.
-   **Actions**: For long-running, preemptable tasks that require feedback (e.g., "navigate to room B", "perform complex manipulation sequence"). Actions allow AI agents to monitor progress and adjust goals dynamically.

## Designing Custom ROS 2 Messages/Services for AI-Robot Interaction

For effective AI-robot communication, it is often necessary to define custom ROS 2 message (`.msg`), service (`.srv`), and action (`.action`) types. These custom interfaces ensure that the data exchanged between AI agents and robot controllers precisely matches their requirements, encapsulating specific command formats, state information, or sensor data structures.

When designing custom interfaces, consider:

1.  **Granularity**: How detailed should the commands be? Should the AI send raw joint torques or high-level navigation goals?
2.  **Data Types**: Use appropriate primitive types (integers, floats, booleans) and arrays. Leverage existing ROS 2 standard messages (`std_msgs`, `geometry_msgs`, `sensor_msgs`) where applicable.
3.  **Semantics**: Clearly define the meaning and expected behavior for each field in your custom message/service/action.
4.  **Error Handling**: How will the robot controller report failures or exceptions back to the AI agent?

## Example: AI Agent Sending Target Joint Angles via a ROS 2 Service

Let's consider an example where an AI agent (e.g., an LLM-based planner) determines a sequence of target joint angles for a robot arm to reach a specific pose. This can be implemented using a custom ROS 2 service.

**1. Define a Custom Service (`SetJointAngles.srv`)**:

```
# Request
float64[] joint_angles
---
# Response
bool success
string message
```

This service would take an array of `float64` values representing the target joint angles as a request, and return a `boolean` indicating success and a `string` message for feedback.

**2. Robot Controller (Service Server)**:

The robot controller node would implement the `SetJointAngles` service. It would receive the `joint_angles` request, validate them, generate a motion plan (e.g., using inverse kinematics), and execute the movement. Upon completion (or failure), it would send back a `success` flag and a `message`.

**3. AI Agent (Service Client)**:

The AI agent would act as a client, constructing a `SetJointAngles.Request` with the desired joint angles and calling the service. It would then asynchronously wait for the response to determine if the robot successfully executed the command.

This example demonstrates a clean, synchronous interaction suitable for discrete, critical commands where the AI agent needs immediate confirmation of the robot's action.

## Learning Outcomes

Upon completing this chapter, you should be able to:

-   Explain the different ROS 2 communication interfaces (Topics, Services, Actions) and their applicability for AI-robot integration.
-   Design custom ROS 2 message and service types to facilitate tailored communication between AI agents and robot controllers.
-   Implement basic ROS 2 service clients and servers for high-level AI commands, such as sending target joint angles to a robot.
-   Understand the considerations for granularity and error handling in AI-robot communication interfaces.

## Required Skills

To get the most out of this chapter, you should have:

-   Familiarity with ROS 2 Nodes, Topics, Services, and Actions (Chapter 2).
-   Intermediate Python programming skills.
-   Basic understanding of robot kinematics (joint angles, poses).

## Tools & Software

The primary tools and software required for this chapter are:

-   **Operating System**: Ubuntu 22.04 LTS with ROS 2 Humble Hawksbill installed.
-   **ROS 2 Packages**: `rclpy`, `rosidl_default_generators`, `example_interfaces` (for custom message examples).
-   **Text Editor/IDE**: VS Code (recommended).

## Weekly Breakdown

**Week 4: AI-Robot Communication**

-   **Learning Objectives**:
    -   Demonstrate the use of custom ROS 2 messages and services for AI-robot interaction.
    -   Implement a ROS 2 service for an AI agent to send high-level commands to a robot.
    -   Debug communication issues between AI agents and robot controllers.

-   **Activities**:
    -   Read Chapter 4 content.
    -   Define a custom ROS 2 service (`.srv`) for sending joint angles.
    -   Implement a ROS 2 service server (robot controller) that responds to joint angle commands.
    -   Implement a ROS 2 service client (AI agent) that sends commands to the robot controller.
    -   Test the communication using `ros2 service call` and custom client.

## Assessments

-   **Quiz 4**: Short quiz on custom ROS 2 interfaces and AI-robot communication patterns.
-   **Lab Assignment 4**: Extend the joint angle service to include more complex commands (e.g., target Cartesian pose, specific manipulation tasks) and demonstrate feedback mechanisms.

## Lab Setup Requirements

### On-Premise Lab

-   A computer with ROS 2 Humble Hawksbill installed on Ubuntu 22.04.
-   Access to a terminal for executing ROS 2 commands.
-   Python development environment set up.
-   ROS 2 workspace configured for custom message/service development.

### Cloud-Native Lab (Conceptual)

-   ROS 2 Humble Docker container running on a cloud VM.
-   VS Code Remote - Containers connection to the Docker environment.
