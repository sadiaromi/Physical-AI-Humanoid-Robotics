---
id: chapter2-ros2-nodes-and-topics
title: "Chapter 2: ROS 2 Architecture: Nodes, Topics, Services, Actions"
sidebar_label: "Chapter 2: ROS 2 Arch"
---

## Introduction

In the previous chapter, we established ROS 2 as the "nervous system" for our humanoid robots and successfully set up our development environment. Now, it's time to dive deeper into the core architectural components that make ROS 2 so powerful and flexible. Understanding these primitives—Nodes, Topics, Services, and Actions—is crucial for building any non-trivial robotic application.

This chapter will provide a detailed explanation of each of these concepts, illustrating them with practical Python (using `rclpy`) code examples. We will also explore the command-line interface (CLI) tools that come with ROS 2, which are invaluable for inspecting, debugging, and understanding the communication patterns within your robot's software system.

## Learning Outcomes

By the end of this chapter, you will be able to:

*   Differentiate between the four core communication primitives in ROS 2: Nodes, Topics, Services, and Actions, understanding their unique purposes and interaction patterns.
*   Explain the key characteristics of each primitive, including their synchronous/asynchronous nature, one-to-many or one-to-one communication models, and typical use cases in robotic applications.
*   Develop basic ROS 2 Python nodes using `rclpy` to implement publishers, subscribers, service servers, service clients, action servers, and action clients.
*   Utilize ROS 2 command-line interface (CLI) tools (e.g., `ros2 node list`, `ros2 topic echo`, `ros2 service call`, `ros2 action send_goal`) to inspect, monitor, and interact with a running ROS 2 system.

## Required Skills and Tools

### Prerequisite Skills

*   **Solid Python 3 Programming**: This chapter will heavily involve writing Python code using `rclpy`. You should be comfortable with Python syntax, functions, classes, and basic data structures.
*   **Linux Command-Line Interface**: Continued proficiency with the Linux terminal is essential for running ROS 2 commands and scripts.
*   **Basic Git Usage**: Familiarity with basic `git` commands (clone, add, commit) for managing code.
*   **Completed Chapter 1 Setup**: Your ROS 2 Humble environment should be fully installed and verified as per Chapter 1's lab.

### Tools & Software

*   **ROS 2 Humble Hawksbill**: A fully functional installation on Ubuntu 22.04.
*   **Code Editor**: A code editor capable of editing Python files (e.g., VS Code, Sublime Text, PyCharm).
*   **Terminal**: A terminal with your ROS 2 environment sourced.

## Weekly Breakdown

This chapter is designed to be completed within one week.

*   **Day 1-2: Understanding ROS 2 Primitives**
    *   **Activity**: Read the "Detailed Explanation of Nodes, Topics, Services, and Actions" sections. Focus on understanding the conceptual differences and appropriate use cases for each communication primitive.
    *   **Assessment**: In your own words, describe a simple robotic task and break down how you would use Nodes, Topics, Services, and Actions to implement its various components.

*   **Day 3-5: Implementing ROS 2 `rclpy` Examples**
    *   **Activity**: Review and understand the provided Python (`rclpy`) examples for publishers, subscribers, service servers, service clients, action clients, and action servers located in `code/ros2/basics/`. Run these examples to observe their behavior.
    *   **Assessment**: Successfully run the publisher/subscriber pair to verify topic communication. Run the service client/server pair to verify synchronous communication. Run the action client/server pair to verify goal-oriented communication with feedback.

*   **Day 6-7: Mastering ROS 2 CLI Tools**
    *   **Activity**: Experiment with all the `ros2` command-line tools listed in the chapter. Use them to inspect your running ROS 2 examples from Day 3-5. Pay attention to how the information displayed by the CLI tools corresponds to the concepts learned.
    *   **Assessment**: Use CLI tools to list all active nodes, topics, services, and actions. Echo a topic, call a service, and send a goal to an action server purely from the command line, verifying successful interaction and information retrieval.

## Lab Setup Requirements

The lab setup for this chapter continues to use the ROS 2 Humble environment configured in Chapter 1.

1.  **ROS 2 Humble Environment**:
    *   Ensure your ROS 2 Humble Hawksbill environment is fully installed and verified (as per Chapter 1's lab setup).
    *   Make sure you have sourced the ROS 2 setup file in each terminal you intend to use for running ROS 2 nodes.

2.  **Code Editor**:
    *   A code editor (e.g., VS Code, Sublime Text, PyCharm) is required for writing and editing Python scripts.

3.  **ROS 2 Workspace**:
    *   You will need a ROS 2 workspace (e.g., `~/ros2_ws`) to create and build your Python packages. If you don't have one, create it by following these steps:
        ```bash
        mkdir -p ~/ros2_ws/src
        cd ~/ros2_ws
        colcon build
        source install/setup.bash
        ```
    *   Remember to source this workspace setup file in every new terminal where you will be working on your ROS 2 packages.

4.  **Python Packages**:
    *   No additional Python packages beyond the standard ROS 2 Python dependencies (`rclpy`) are strictly required for the basic examples in this chapter, as these are typically installed with the ROS 2 desktop version.

5.  **Verification**:
    *   The successful execution of the publisher/subscriber, service client/server, and action client/server examples (as described in the Weekly Breakdown) will serve as verification that your lab setup is correct and ready.

## Detailed Explanation of Nodes, Topics, Services, and Actions

### Nodes: The Computational Units

In ROS 2, a **Node** is essentially an executable process that performs computation. A robotic system is typically composed of many nodes, each responsible for a specific task. For example, one node might control the robot's motors, another might process camera images, and a third might implement a high-level planning algorithm.

**Key characteristics of Nodes:**
*   **Modular**: Each node focuses on a single, well-defined task. This promotes code reusability and simplifies debugging.
*   **Distributed**: Nodes can run on different machines, different processes, or even different threads within the same process. ROS 2's underlying Data Distribution Service (DDS) handles the inter-process communication seamlessly.
*   **Decentralized**: There is no central "master" node in ROS 2. Nodes discover each other dynamically.

### Topics: Asynchronous Data Streaming

**Topics** are the most common way for nodes to exchange data asynchronously and in a many-to-many fashion. They implement a publish-subscribe model:

*   A node that wants to share data **publishes** messages to a named topic.
*   Any other node that is interested in that data **subscribes** to the same topic.

Messages published to a topic are typically small, streamable pieces of data (e.g., sensor readings, joint states, velocity commands).

**Example Use Cases:**
*   A camera node publishes images to `/camera/image_raw`.
*   A motor controller node subscribes to `/cmd_vel` to receive velocity commands.
*   A LiDAR node publishes point clouds to `/scan`.

### Services: Synchronous Request-Reply

**Services** are used for synchronous, request-reply communication between nodes. When a client node needs to request a specific operation from a server node and expects an immediate response, it uses a service.

**Key characteristics of Services:**
*   **Request-Reply**: A client sends a request and waits for a response.
*   **Synchronous**: The client blocks until it receives a response (or a timeout occurs).
*   **One-to-one**: Typically involves one client making a request to one server.

**Example Use Cases:**
*   A navigation node requests a map-building service to "save current map".
*   A UI node requests a robot configuration service to "get current joint limits".

### Actions: Long-Running Goal-Oriented Tasks

**Actions** are designed for long-running, goal-oriented tasks that may be preempted and provide periodic feedback. They are essentially an extension of services, adding the ability to monitor progress and cancel the goal during execution.

**Key characteristics of Actions:**
*   **Goal**: The request to perform a task (e.g., "Navigate to the kitchen").
*   **Feedback**: Periodic updates on the progress of the task (e.g., "Robot is 50% to the kitchen").
*   **Result**: The outcome of the task (e.g., "Navigation successful").
*   **Preemptable**: The client can cancel the goal while it is still executing.

**Example Use Cases:**
*   A high-level AI sends a goal to "pick_up_object(red_cube)" and receives feedback on gripper progress.
*   A control system sends a goal to "walk_to(waypoint_A)" and gets updates on the robot's current position.

## Python (rclpy) Examples for Each Concept

(These code examples will be implemented in T013)

### Node Example (Minimal)

```python
# minimal_publisher.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS 2: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Topic Example (Minimal Subscriber)

```python
# minimal_subscriber.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Service Example

(Service server and client examples will be detailed and implemented in T013)

### Action Example

(Action server and client examples will be detailed and implemented in T013)

## CLI Tools for Inspecting ROS 2 Graphs

ROS 2 provides a powerful set of command-line tools to interact with and inspect your running system. These are indispensable for development and debugging.

*   **`ros2 node list`**: Lists all currently active ROS 2 nodes.
*   **`ros2 topic list`**: Lists all active topics. Add `-t` to show topic types.
*   **`ros2 topic echo <topic_name>`**: Displays messages being published on a specified topic.
*   **`ros2 topic info <topic_name>`**: Shows information about a topic, including its type and publishers/subscribers.
*   **`ros2 service list`**: Lists all available services. Add `-t` to show service types.
*   **`ros2 service type <service_name>`**: Shows the type of a service.
*   **`ros2 service call <service_name> <service_type> <arguments>`**: Calls a service.
*   **`ros2 action list`**: Lists all active actions. Add `-t` to show action types.
*   **`ros2 action info <action_name>`**: Shows information about an action.
*   **`ros2 action send_goal <action_name> <action_type> <goal_arguments>`**: Sends a goal to an action server.

By mastering these architectural concepts and CLI tools, you will be well-equipped to design, implement, and debug sophisticated robotic behaviors.