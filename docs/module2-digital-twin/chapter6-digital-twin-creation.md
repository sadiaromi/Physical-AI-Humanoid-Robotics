---
id: chapter6-digital-twin-creation
title: "Chapter 6: Gazebo + Unity for Digital Twin Creation"
sidebar_label: "Chapter 6: Digital Twin Creation"
---

## Introduction

In the previous chapter, we explored the fundamental principles of physics simulation and sensor modeling. Now, it's time to put those principles into practice by building and interacting with digital twins. A **digital twin** is a virtual replica of a physical system, in our case, a humanoid robot and its environment. It allows for realistic simulation, testing, and training in a risk-free and cost-effective manner.

This chapter will guide you through the process of creating digital twins using two prominent simulation platforms: **Gazebo** and **Unity**. We will cover how to create and import robot models, set up diverse virtual environments, and crucially, how to bridge these simulations with our ROS 2 control framework. This integration enables our AI agents to control and perceive the digital twin as if it were a real robot.

## Learning Outcomes

By the end of this chapter, you will be able to:

*   Explain the concept of a digital twin and articulate its critical role in the development, testing, and training of humanoid robots.
*   Compare and contrast Gazebo and Unity as simulation platforms, identifying their strengths and weaknesses for robotics applications.
*   Create and import robot models into both Gazebo (leveraging URDF conversion to SDF) and Unity environments.
*   Set up comprehensive virtual environments within Gazebo (using `.world` files) and Unity (using Scenes), including static obstacles and dynamic objects.
*   Understand the mechanisms for bridging ROS 2 communication with both Gazebo (using `ros_gz_bridge`) and Unity (using Unity Robotics Hub), enabling real-time control and data exchange.

## Required Skills and Tools

### Prerequisite Skills

*   **Completion of all previous chapters**: A solid understanding of ROS 2 primitives (Nodes, Topics, Services, Actions) and URDF is essential.
*   **Basic familiarity with Linux command-line operations**: For launching simulators and ROS 2 nodes.
*   **Familiarity with 3D concepts**: Understanding how objects are represented and manipulated in a 3D space.

### Tools & Software

*   **ROS 2 Humble Hawksbill**: A fully functional installation on Ubuntu 22.04.
*   **Gazebo**: The primary 3D robot simulator we will use. Install Gazebo Fortress (the version compatible with ROS 2 Humble):
    ```bash
    sudo apt install ros-humble-gazebo-ros-pkgs
    ```
*   **(Optional) Unity Hub and Unity Editor**: Version 2022.3 LTS or newer. If you choose to explore Unity, you will also need to install the Unity Robotics Hub package from the Unity Asset Store within your Unity project.
*   **Text Editor**: For editing `.world`, URDF, and potentially Unity-related configuration files.
*   **`ros_gz_bridge`**: This package facilitates communication between ROS 2 and Gazebo. It is typically installed as part of `ros-humble-gazebo-ros-pkgs`.

## Weekly Breakdown

This chapter is designed to be completed within one week.

*   **Day 1-2: Gazebo Setup and Robot Import**
    *   **Activity**: Install Gazebo and its ROS 2 packages using the instructions in the "Required Skills and Tools" section. Launch an empty Gazebo world. Then, launch your `simple_humanoid.urdf` from Chapter 3 into Gazebo using appropriate ROS 2 launch commands (e.g., `ros2 launch urdf_tutorial display.launch.py model:=$(ros2 pkg prefix urdf_tutorial)/share/urdf_tutorial/urdf/01-myfirst.urdf`).
    *   **Assessment**: Successfully spawn your humanoid robot model into Gazebo. Observe it interacting with gravity and the ground plane.

*   **Day 3-4: Virtual Environment Design in Gazebo**
    *   **Activity**: Create a new `.world` file in `code/simulation/digital_twin/` (e.g., `my_room.world`). Add static models like walls, a table, and a simple obstacle. You can use Gazebo's built-in models (e.g., `model://table`, `model://bookshelf`) or define simple geometries directly in the `.world` file.
    *   **Assessment**: Launch your custom world in Gazebo and verify that all static objects and obstacles are present and correctly positioned.

*   **Day 5-6: Bridging ROS 2 and Gazebo**
    *   **Activity**: Experiment with `ros_gz_bridge`. Set up a simple ROS 2 publisher that publishes to a topic that `ros_gz_bridge` can forward to Gazebo (e.g., a joint command or an object pose). Conversely, try to subscribe to a basic sensor topic from Gazebo (e.g., `model/humanoid_robot/joint_state`) in ROS 2.
    *   **Assessment**: Demonstrate a successful one-way communication bridge (e.g., sending a command from ROS 2 to move a joint in Gazebo, or reading a sensor value from Gazebo in a ROS 2 node) and describe the configuration used.

*   **Day 7: (Optional) Unity Exploration or Review**
    *   **Activity**: (Optional) If you are interested in Unity, follow the instructions to install Unity Hub and the Unity Editor. Explore the Unity Robotics Hub and try importing a simple URDF model into a Unity scene.
    *   **Assessment**: (Optional) Successfully import a basic URDF into a Unity scene and get it to display correctly. Otherwise, conduct a thorough review of the concepts of digital twin creation, URDF import, and ROS 2 bridging in both Gazebo and Unity.

## Lab Setup Requirements

The lab setup for this chapter requires a properly installed and configured Gazebo simulation environment, optionally with Unity.

1.  **ROS 2 Humble Environment**:
    *   Ensure your ROS 2 Humble Hawksbill environment is fully installed and verified (as per Chapter 1).
    *   Make sure you have sourced the ROS 2 setup file in each terminal you intend to use.

2.  **Gazebo Simulation Environment**:
    *   **Gazebo Fortress** (or newer, compatible with ROS 2 Humble) must be installed. This includes the `ros-humble-gazebo-ros-pkgs` package, which provides the necessary ROS 2 integration.
    *   Familiarity with launching Gazebo from the command line (`gazebo --verbose`).

3.  **(Optional) Unity Development Environment**:
    *   **Unity Hub and Unity Editor (version 2022.3 LTS or newer)** installed.
    *   **Unity Robotics Hub** package installed within a Unity project to facilitate ROS 2 integration.

4.  **URDF Model**:
    *   Your `simple_humanoid.urdf` file from Chapter 3 should be available and correctly configured.

5.  **Verification**:
    *   The successful spawning of your URDF model into Gazebo and/or Unity.
    *   Demonstrating one-way communication between ROS 2 and Gazebo (or Unity) using `ros_gz_bridge` (or Unity Robotics Hub), as outlined in the Weekly Breakdown.

## Creating and Importing Robot Models into Gazebo and Unity

To bring our humanoid robots into the digital realm, we need to create or import their 3D models and define their physical properties within the simulation environment.

### Using URDF with Gazebo

Gazebo primarily uses the Simulation Description Format (SDF) for world and robot descriptions. However, it can directly parse and convert URDF files (which we learned about in Chapter 3) into its internal SDF representation. This means you can often use your existing URDF models in Gazebo.

**Key steps for importing URDF into Gazebo:**

1.  **Ensure URDF is complete**: Your URDF must define visual, collision, and inertial properties for all links and specify all joints.
2.  **Launch Gazebo with URDF**: You typically launch Gazebo and then spawn your robot model. ROS 2 provides tools for this, often using a launch file that includes `spawn_entity.py` from `ros_gz_sim_demos`.

### Importing Models into Unity

Unity, being a game engine, has its own asset pipeline. Robot models are often imported as standard 3D file formats (e.g., FBX, OBJ, URDF) and then configured using Unity's physics engine and scripting capabilities.

**Key steps for importing models into Unity:**

1.  **3D Model Import**: Import your robot's 3D assets (meshes, textures) into your Unity project.
2.  **Configure Physics**: Add `Rigidbody` components to links that need to interact physically and configure colliders.
3.  **Unity Robotics Hub**: This package from Unity provides tools and tutorials for integrating robotics simulations with ROS. It simplifies the process of importing URDFs and setting up ROS communication.

## Setting Up Virtual Environments (Rooms, Obstacles)

A digital twin is more than just a robot model; it's a robot operating within a defined environment. These virtual environments are crucial for testing navigation, manipulation, and perception algorithms.

### Gazebo Worlds

Gazebo uses `.world` files (SDF format) to describe entire environments, including:

*   **Models**: Static models (e.g., walls, furniture, obstacles) and dynamic models (other robots, objects to be manipulated).
*   **Lights**: Directional, point, and spot lights.
*   **Sensors**: Virtual cameras, LiDARs, IMUs attached to models or the world.
*   **Physics Properties**: Global gravity, physics engine settings.

### Unity Scenes

In Unity, environments are created within Scenes. You can:

*   **Design with ProBuilder/ProGrids**: Create geometric primitives for walls, floors, and furniture.
*   **Import 3D Assets**: Use assets from the Unity Asset Store or external 3D modeling software.
*   **Configure Physics**: Ensure all static and dynamic objects have appropriate colliders and rigidbodies.

## Bridging ROS 2 with Gazebo/Unity

The critical link between our AI control system (ROS 2) and the digital twin (Gazebo/Unity) is the communication bridge.

### `ros_gz_bridge` for Gazebo

For Gazebo, the `ros_gz_bridge` package is the official way to establish communication between ROS 2 topics/services/actions and Gazebo's native communication system (Ignition Transport).

*   **Purpose**: It allows ROS 2 nodes to publish commands to Gazebo (e.g., joint commands, spawn models) and subscribe to sensor data from Gazebo (e.g., camera images, LiDAR scans).
*   **Configuration**: The bridge maps topics between ROS 2 and Ignition Transport. This is typically configured in YAML files or directly via launch files.

### Unity Robotics Hub for Unity

The Unity Robotics Hub provides robust tools for integrating Unity with ROS 2.

*   **ROS TCP Connector**: Enables bi-directional communication between Unity and ROS 2 over TCP.
*   **ROS Message Generation**: Automatically generates C# message types from ROS 2 `.msg`, `.srv`, and `.action` definitions.
*   **URDF Importer**: Simplifies importing and configuring URDF robot models directly into Unity scenes.

By mastering these techniques, you'll be able to create rich, interactive digital twins that faithfully represent your physical robotics systems, providing an invaluable platform for development and experimentation.