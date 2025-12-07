# Unity Digital Twin Setup Guide

This directory contains conceptual guidance and a placeholder for a Unity-based digital twin project. Direct creation of Unity project files is not feasible through this interface.

## To set up a Unity digital twin for the humanoid robot:

1.  **Create a New Unity Project**: Open Unity Hub and create a new 3D (URP or HDRP recommended) project.

2.  **Install Unity Robotics Hub**: Follow the official Unity Robotics Hub documentation to install the necessary packages (e.g., `ROS-TCP-Connector`, `URDF-Importer`).

3.  **Import Robot Model**:
    *   If using the `simple_humanoid.urdf` from Chapter 3, use the URDF Importer package to import it into Unity.
    *   Alternatively, import a 3D model (FBX, OBJ) of a humanoid robot and manually configure its Articulation Bodies to match the desired joint structure.

4.  **Create Virtual Environment**: Design a simple room with obstacles using Unity's built-in tools or by importing 3D assets.

5.  **Configure ROS 2 Communication**:
    *   Add the `ROS-TCP-Connector` components to your Unity scene.
    *   Create C# scripts to subscribe to ROS 2 topics (e.g., `/joint_commands`) and publish sensor data (e.g., `/camera/image_raw`).
    *   Ensure the robot's Articulation Bodies are controlled by the incoming ROS 2 commands.

## Example Structure (Conceptual)

```
UnityProjectRoot/
├── Assets/
│   ├── Scenes/
│   │   └── HumanoidScene.unity
│   ├── Models/
│   │   └── HumanoidRobot/
│   │       ├── Humanoid.fbx
│   │       └── Humanoid_articulation.prefab
│   ├── Scripts/
│   │   ├── RosJointController.cs
│   │   └── RosCameraPublisher.cs
│   └── ... (other assets)
└── Packages/
    ├── com.unity.robotics.ros-tcp-connector/
    ├── com.unity.robotics.urdf-importer/
    └── ...
```

This setup allows for a Unity instance to act as a digital twin, receiving commands and sending sensor data to/from a ROS 2 environment.
