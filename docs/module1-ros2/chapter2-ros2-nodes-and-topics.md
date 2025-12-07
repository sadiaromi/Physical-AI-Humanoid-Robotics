# Chapter 2: ROS 2 Architecture: Nodes, Topics, Services, Actions

## Detailed Explanation of Nodes, Topics, Services, and Actions

ROS 2 is built upon a distributed architecture where independent executable processes, called **Nodes**, communicate with each other using various communication mechanisms. Understanding these mechanisms is crucial for developing robust and modular robotic applications.

### Nodes

A Node is a fundamental unit of computation in ROS 2. Each node should be responsible for a single, well-defined task (e.g., a camera driver, a motor controller, a planning algorithm). This modularity allows for easier debugging, testing, and reuse of components.

-   **Example**: `rclpy` provides `Node` class for creating ROS 2 nodes in Python.

### Topics (Asynchronous Data Streaming)

Topics are a publish/subscribe communication model used for streaming continuous data. A node that produces data (e.g., sensor readings) acts as a **publisher**, and nodes that consume this data (e.g., visualization tools, control algorithms) act as **subscribers**. This is a one-to-many or many-to-many communication pattern.

-   **Key Characteristics**:
    -   Asynchronous: Publishers don't wait for subscribers.
    -   Decoupled: Publishers and subscribers don't need to know about each other's existence directly.
    -   Real-time data: Suitable for sensor streams, odometry, joint states.

-   **Example**: A LiDAR sensor node publishes `sensor_msgs/LaserScan` messages on a `/scan` topic, and a mapping node subscribes to it.

### Services (Synchronous Request/Reply)

Services are a request/reply communication model used for single, synchronous operations. A client node sends a **request** to a server node, which performs a computation and sends back a **response**. This is a one-to-one communication pattern.

-   **Key Characteristics**:
    -   Synchronous: The client waits for the server's response.
    -   Reliable: Guarantees delivery of request and response.
    -   Discrete operations: Suitable for triggering actions (e.g., "move robot to X, Y"), querying information (e.g., "get current robot pose").

-   **Example**: A navigation client requests a `nav_msgs/GetMap` service from a map server.

### Actions (Long-running, Preemptable Tasks with Feedback)

Actions are designed for long-running tasks that can be preempted and provide periodic feedback. They extend the service concept by adding preemption and continuous feedback. An action client sends a **goal** to an action server, which executes the task, provides **feedback** on its progress, and eventually returns a **result**. The client can also **cancel** the goal at any time.

-   **Key Characteristics**:
    -   Asynchronous goal execution.
    -   Preemptable: Goals can be canceled.
    -   Feedback: Provides continuous progress updates.
    -   Terminal result: Returns a final outcome.

-   **Example**: An action client commands a robot arm to pick up an object (goal), receives feedback on arm position, and gets a success/failure result.

## Python (rclpy) Examples for Each Concept

This section will provide practical Python code examples using `rclpy` to demonstrate the implementation of ROS 2 nodes, publishers, subscribers, service clients, service servers, action clients, and action servers. These examples will be designed to be runnable within your ROS 2 environment.

## CLI Tools for Inspecting ROS 2 Graphs

ROS 2 provides a rich set of command-line interface (CLI) tools to inspect and interact with a running ROS 2 system. These tools are invaluable for debugging and understanding the communication graph.

-   `ros2 node list`: Lists all active nodes.
-   `ros2 topic list`: Lists all active topics.
-   `ros2 topic echo <topic_name>`: Displays messages being published on a topic.
-   `ros2 service list`: Lists all active services.
-   `ros2 service call <service_name> <service_type> <arguments>`: Calls a service.
-   `ros2 action list`: Lists all active actions.
-   `ros2 action send_goal <action_name> <action_type> <arguments>`: Sends a goal to an action server.

## Learning Outcomes

Upon completing this chapter, you should be able to:

-   Explain the core architectural components of ROS 2 (nodes, topics, services, actions).
-   Differentiate between asynchronous (topics, actions) and synchronous (services) communication patterns.
-   Implement basic ROS 2 nodes, publishers, subscribers, service clients/servers, and action clients/servers in Python using `rclpy`.
-   Utilize ROS 2 CLI tools to inspect and debug a running ROS 2 graph.

## Required Skills

To get the most out of this chapter, you should have:

-   Familiarity with the ROS 2 environment setup (Chapter 1).
-   Intermediate Python programming skills.
-   Basic understanding of distributed systems concepts.

## Tools & Software

The primary tools and software required for this chapter are:

-   **Operating System**: Ubuntu 22.04 LTS with ROS 2 Humble Hawksbill installed.
-   **Python**: Version 3.8+.
-   **ROS 2 Packages**: `rclpy`, `std_msgs`, `example_interfaces` (for services and actions).
-   **Text Editor/IDE**: VS Code (recommended).

## Weekly Breakdown

**Week 2: ROS 2 Communication Patterns**

-   **Learning Objectives**:
    -   Demonstrate the use of ROS 2 nodes, topics, services, and actions through practical examples.
    -   Effectively use ROS 2 CLI tools for introspection and debugging.
    -   Understand the trade-offs between different ROS 2 communication methods.

-   **Activities**:
    -   Read Chapter 2 content.
    -   Run provided ROS 2 Python examples (publisher, subscriber, service client/server, action client/server).
    -   Experiment with `ros2` CLI tools (e.g., `ros2 node info`, `ros2 topic hz`).
    -   Modify a simple publisher/subscriber example.

## Assessments

-   **Quiz 2**: Short quiz on ROS 2 communication patterns and CLI tools.
-   **Lab Assignment 2**: Implement a simple ROS 2 publisher/subscriber system with custom messages, and demonstrate its functionality using CLI tools.

## Lab Setup Requirements

### On-Premise Lab

-   A computer with ROS 2 Humble Hawksbill installed on Ubuntu 22.04.
-   Access to a terminal for executing ROS 2 commands.
-   Python development environment set up.

### Cloud-Native Lab (Conceptual)

-   ROS 2 Humble Docker container running on a cloud VM.
-   VS Code Remote - Containers connection to the Docker environment.
