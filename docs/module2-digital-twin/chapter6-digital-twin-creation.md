# Chapter 6: Gazebo + Unity for Digital Twin Creation

## Creating and Importing Robot Models into Gazebo and Unity

Digital twins are virtual representations of physical systems, crucial for simulating, testing, and optimizing robotic behaviors. Gazebo and Unity are two powerful simulation environments widely used for creating digital twins. Both allow for the import of robot models, typically defined using URDF (Unified Robot Description Format) or SDF (Simulation Description Format) for Gazebo, and custom 3D models (e.g., FBX, OBJ) for Unity.

### Gazebo

-   **URDF/SDF Import**: Gazebo natively supports URDF and SDF. A URDF file can be converted to SDF for Gazebo, which often provides more features for simulation aspects like sensors, plugins, and physics properties. You can directly load these models into a Gazebo world.
-   **Model Database**: Gazebo has a rich online model database, allowing users to easily import common objects and robots.
-   **Custom Models**: For custom robots, you typically create a URDF/SDF file describing the robot's links, joints, and visual/collision geometries, and place it within a ROS 2 package or Gazebo model path.

### Unity

-   **3D Model Import**: Unity is a game engine, so it excels at importing various 3D model formats. Robot models can be imported as FBX, OBJ, or other formats. These models are then composed into a GameObject hierarchy, with each part representing a robot link.
-   **Articulations**: Unity's Articulation Body system is designed for realistic robot simulation, allowing you to define joints and control them with physics-based or kinematic methods.
-   **ROS-Unity Bridge**: The Unity Robotics Hub provides tools and packages to facilitate communication between Unity and ROS 2, enabling the control of Unity-simulated robots from ROS 2.

## Setting Up Virtual Environments (Rooms, Obstacles)

Realistic and diverse virtual environments are essential for comprehensive robot testing and training. Both Gazebo and Unity offer tools to create detailed environments, including rooms, obstacles, varying textures, and lighting conditions.

### Gazebo

-   **World Files (`.world`)**: Gazebo environments are defined in `.world` files (SDF format). These files specify the ground plane, lights, static obstacles (e.g., walls, furniture), and dynamic objects.
-   **Plugins**: Gazebo plugins allow for dynamic environment manipulation, such as spawning/despawning objects, changing properties, or creating interactive elements.
-   **Terrain Generation**: You can create complex terrains using heightmaps and textures.

### Unity

-   **Scene Creation**: Unity provides a visual editor to build environments by dragging and dropping 3D assets, creating terrains, and arranging lighting.
-   **Prefabs**: Reusable environmental components (e.g., a specific type of chair, a modular wall section) can be saved as prefabs for efficient environment construction.
-   **Procedural Generation**: Scripting in C# allows for procedural generation of environments, enabling infinite variations for training or testing.

## Bridging ROS 2 with Gazebo/Unity (e.g., `ros_gz_bridge`, Unity Robotics Hub)

To effectively integrate AI agents and controllers (often running in ROS 2) with simulation environments, a robust bridge is required. This bridge enables the flow of sensor data from the simulator to ROS 2, and control commands from ROS 2 to the simulator.

### ROS 2 and Gazebo

-   **`ros_gz_bridge`**: This package provides a bidirectional bridge between ROS 2 and Gazebo (specifically Gazebo Garden/Fortress, the new version). It translates Gazebo topics (e.g., sensor data, joint states) into ROS 2 topics, and vice-versa, allowing seamless communication.
-   **Gazebo ROS 2 Control**: For older Gazebo versions or direct control, `gazebo_ros2_control` provides a direct interface between `ros2_control` (ROS 2's hardware interface) and Gazebo, enabling realistic joint control.

### ROS 2 and Unity

-   **Unity Robotics Hub**: This is a comprehensive collection of Unity packages that facilitate robotics development, including `ROS-TCP-Connector` and `ROS-Unity-Message-Generation`.
-   **`ROS-TCP-Connector`**: Enables communication between Unity and ROS 2 over TCP, allowing ROS 2 nodes to send commands to a Unity simulation and receive sensor data.
-   **`ROS-Unity-Message-Generation`**: Helps generate C# equivalents of ROS 2 messages, making it easy to use custom ROS 2 messages within Unity.

## Learning Outcomes

Upon completing this chapter, you should be able to:

-   Understand the concepts of digital twins and their importance in robotics.
-   Create and import basic robot models into both Gazebo and Unity.
-   Design and set up virtual environments with obstacles and diverse elements in simulation.
-   Implement basic communication bridges between ROS 2 and Gazebo/Unity to enable control and sensor data flow.
-   Compare and contrast the approaches to digital twin creation in Gazebo and Unity.

## Required Skills

To get the most out of this chapter, you should have:

-   Familiarity with URDF/SDF for robot description (Chapter 3).
-   Basic understanding of ROS 2 communication (Chapter 2, 4).
-   Familiarity with 3D modeling concepts (optional, but helpful).
-   Basic knowledge of game engine concepts (for Unity).

## Tools & Software

The primary tools and software required for this chapter are:

-   **Operating System**: Ubuntu 22.04 LTS with ROS 2 Humble Hawksbill installed.
-   **Simulation Environments**: Gazebo (latest version, e.g., Garden/Fortress) and/or Unity (latest LTS version) with Unity Robotics Hub.
-   **ROS 2 Packages**: `ros_gz_bridge` (for Gazebo), Unity Robotics Hub packages (for Unity).
-   **Text Editor/IDE**: VS Code (recommended).

## Weekly Breakdown

**Week 5: Digital Twin Creation**

-   **Learning Objectives**:
    -   Demonstrate importing a URDF robot model into Gazebo.
    -   Demonstrate importing a 3D model into Unity and configuring its articulations.
    -   Create a simple virtual room with obstacles in both Gazebo and Unity.
    -   Set up a basic ROS 2-Gazebo bridge to control a simulated robot.
    -   Set up a basic ROS 2-Unity bridge to control a simulated robot.

-   **Activities**:
    -   Read Chapter 6 content.
    -   Practice importing the `simple_humanoid.urdf` (from Chapter 3) into Gazebo.
    -   Explore creating simple environments in Gazebo using SDF.
    -   Install Unity, Unity Robotics Hub, and import a simple humanoid 3D model (e.g., from Unity Asset Store).
    -   Set up basic articulations for the humanoid in Unity.
    -   Experiment with `ros_gz_bridge` to send commands to a Gazebo robot.
    -   Experiment with `ROS-TCP-Connector` to send commands to a Unity robot.

## Assessments

-   **Quiz 5**: Short quiz on digital twin concepts, Gazebo/Unity environments, and ROS 2 bridging.
-   **Lab Assignment 5**: Create a more complex environment in either Gazebo or Unity (user's choice) with dynamic obstacles. Demonstrate control of a humanoid robot in this environment via ROS 2 commands.

## Lab Setup Requirements

### On-Premise Lab

-   A computer with ROS 2 Humble Hawksbill installed on Ubuntu 22.04.
-   Gazebo (latest) installed.
-   Unity (latest LTS) with Unity Hub and Unity Robotics Hub packages installed.
-   ROS 2 workspace with `ros_gz_bridge` built (if using Gazebo).
-   Python and C# development environments.

### Cloud-Native Lab (Conceptual)

-   ROS 2 Humble Docker container with Gazebo or Unity instance (e.g., via Cloud-based streaming for Unity) running on a cloud VM.
-   VS Code Remote - Containers connection.
-   Considerations for GPU access for Unity/Gazebo rendering in the cloud.
