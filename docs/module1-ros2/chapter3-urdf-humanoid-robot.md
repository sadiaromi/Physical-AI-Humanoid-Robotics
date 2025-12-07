---
id: chapter3-urdf-humanoid-robot
title: "Chapter 3: URDF for Humanoid Robot Structure"
sidebar_label: "Chapter 3: URDF"
---

## Introduction

To interact with and simulate a robot, we first need a way to describe its physical characteristics. This is where the Unified Robot Description Format (URDF) comes in. URDF is an XML-based file format used in ROS (and by extension, ROS 2) to describe all aspects of a robot's physical model, including its kinematic and dynamic properties, visual appearance, and collision geometry.

In this chapter, we will delve into the fundamentals of URDF. We will learn how to define a robot's links (rigid bodies) and joints (connections between links) to create a basic kinematic chain. Our focus will be on constructing a simplified humanoid robot model, providing a foundation for more complex designs. Finally, we will cover how to visualize your URDF models in `rviz2`, a powerful 3D visualization tool for ROS 2.

## Learning Outcomes

By the end of this chapter, you will be able to:

*   Understand the purpose and fundamental concepts of the Unified Robot Description Format (URDF) for describing robotic systems.
*   Identify and differentiate between URDF links (rigid bodies) and joints (connections between links), understanding their respective properties (visual, collision, inertial, type, limits).
*   Create a basic URDF file to describe a simple humanoid robot, including defining its links (e.g., `base_link`, `head_link`) and connecting them with appropriate joints (e.g., `neck_joint`).
*   Utilize `rviz2`, the ROS 2 visualization tool, to load and display your URDF models, configuring the `RobotModel` display and understanding the role of `robot_state_publisher`.

## Required Skills and Tools

### Prerequisite Skills

*   **Basic Understanding of XML**: URDF files are written in XML, so familiarity with XML syntax is helpful.
*   **Familiarity with 3D Coordinate Systems**: Understanding concepts like origin, XYZ, and RPY (Roll, Pitch, Yaw) for defining poses and orientations in 3D space.
*   **Completed Chapter 1 Setup**: Your ROS 2 Humble environment should be fully installed and verified as per Chapter 1's lab.

### Tools & Software

*   **ROS 2 Humble Hawksbill**: A fully functional installation on Ubuntu 22.04.
*   **Code Editor**: A text editor (e.g., VS Code, Sublime Text, PyCharm) for creating and editing URDF files.
*   **`rviz2`**: The ROS 2 visualization tool, which should be installed as part of the ROS 2 desktop installation.
*   **`urdf_tutorial` package**: A helpful ROS 2 package that provides tools for visualizing URDFs. You may need to install it:
    ```bash
    sudo apt install ros-humble-urdf-tutorial
    ```
*   **`robot_state_publisher` package**: Essential for publishing the robot's state to `rviz2`. This is typically installed with the ROS 2 desktop version.

## Weekly Breakdown

This chapter is designed to be completed within one week.

*   **Day 1-2: URDF Fundamentals and Links**
    *   **Activity**: Read the "Fundamentals of URDF for Robot Description" section, focusing on the properties of links (visual, collision, inertial). Create a copy of the `simple_humanoid.urdf` and start modifying it.
    *   **Assessment**: In your modified URDF, add an additional link (e.g., a "left_arm_link" as a simple box or cylinder) with appropriate visual, collision, and inertial properties.

*   **Day 3-4: Joints and Kinematic Chains**
    *   **Activity**: Read the section on joints. Connect your newly created "left_arm_link" to the "base_link" using a revolute joint. Experiment with different `origin` and `axis` values to understand their effect on the joint's position and rotation.
    *   **Assessment**: Successfully connect two new links with a joint in your URDF file, ensuring it forms a valid kinematic chain relative to your `base_link`.

*   **Day 5-6: Visualization with `rviz2`**
    *   **Activity**: Follow the instructions to load your `simple_humanoid.urdf` into `rviz2`. Experiment with changing joint values interactively (e.g., if you install `joint_state_publisher_gui` using `sudo apt install ros-humble-joint-state-publisher-gui`) to see the robot move.
    *   **Assessment**: Successfully visualize your `simple_humanoid.urdf` in `rviz2` and demonstrate the ability to manipulate its `neck_joint` (and any other joints you added) to observe its movement.

*   **Day 7: Review and Extension**
    *   **Activity**: Review the entire URDF file for your simple humanoid robot. Consider how you would extend this model to represent a more complete humanoid (e.g., adding legs, more complex arms, end-effectors).
    *   **Assessment**: Propose a design (links and joints) for a new major limb (e.g., a "left_leg_link" with a "hip_joint" and "knee_joint") that would connect to your existing humanoid model, including the type, origin, and axis for each joint.

## Lab Setup Requirements

The lab setup for this chapter requires a properly configured ROS 2 environment with visualization tools.

1.  **ROS 2 Humble Environment**:
    *   Ensure your ROS 2 Humble Hawksbill environment is fully installed and verified (as per Chapter 1's lab setup).
    *   Make sure you have sourced the ROS 2 setup file in each terminal you intend to use for running ROS 2 commands.

2.  **Code Editor**:
    *   A text editor (e.g., VS Code, Sublime Text, PyCharm) is required for creating and editing URDF (XML) files.

3.  **Required ROS 2 Packages**:
    *   **`rviz2`**: The primary 3D visualization tool for ROS 2. This should be installed as part of the ROS 2 desktop version.
    *   **`robot_state_publisher`**: Essential for publishing the robot's state to `rviz2`. This is typically installed with the ROS 2 desktop version.
    *   **`urdf_tutorial`**: This package is very useful for quick URDF visualization examples. Install it if you haven't already:
        ```bash
        sudo apt install ros-humble-urdf-tutorial
        ```
    *   **`joint_state_publisher_gui`**: (Optional but highly recommended for interactive joint manipulation)
        ```bash
        sudo apt install ros-humble-joint-state-publisher-gui
        ```

4.  **Verification**:
    *   The successful loading and visualization of your `simple_humanoid.urdf` in `rviz2` (as described in the Weekly Breakdown) will serve as verification that your lab setup is correct and ready.

## Fundamentals of URDF for Robot Description

URDF uses an XML schema to represent a robot as a tree of links and joints.

*   **Links**: Represent the rigid bodies of the robot (e.g., torso, head, upper arm, forearm). Each link has:
    *   **Inertial Properties**: Mass, center of mass, and inertia matrix (for dynamic simulation).
    *   **Visual Properties**: The 3D model (e.g., `.dae`, `.stl`, `.obj`) and color used for visualization.
    *   **Collision Properties**: The 3D model used for collision detection in simulations. This is often a simplified version of the visual model for computational efficiency.

*   **Joints**: Represent the connections between two links, defining their relative motion. Each joint has:
    *   **Parent and Child Links**: Specifies which links the joint connects.
    *   **Type**: Defines the degrees of freedom of the joint (e.g., `revolute` for rotational, `prismatic` for linear, `fixed` for rigid connection).
    *   **Origin**: The 3D pose of the joint frame relative to the parent link.
    *   **Axis**: The axis of motion for revolute and prismatic joints.
    *   **Limits**: Upper and lower bounds for the joint's position, velocity, and effort.

### Basic Structure of a URDF File

A URDF file starts with a `<robot>` tag, which contains one or more `<link>` and `<joint>` tags.

```xml
<?xml version="1.0"?>
<robot name="simple_humanoid">

  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.4"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 0.8 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Add more links and joints here -->

</robot>
```

## Creating a Basic URDF for a Simple Humanoid Link Structure

Let's build a very simple humanoid, starting with a base link and adding a head.

### Defining the `base_link` (Torso)

As shown in the example above, `base_link` can be a box. We'll give it a mass and inertia for physical simulation.

### Adding a `head_link`

The head will be another link, connected to the `base_link` by a joint.

```xml
  <link name="head_link">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
      <material name="red">
        <color rgba="0.8 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.5"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0" iyy="0.001" iyz="0.0" izz="0.001"/>
    </inertial>
  </link>
```

### Connecting with a `neck_joint`

Now we connect `base_link` (parent) and `head_link` (child) with a revolute joint.

```xml
  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head_link"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/> <!-- Offset from base_link to position head above torso -->
    <axis xyz="0 0 1"/> <!-- Rotation around Z-axis -->
    <limit lower="-1.57" upper="1.57" effort="100.0" velocity="0.5"/>
  </joint>
```

(The full `simple_humanoid.urdf` will be created in T019)

## Loading URDFs into `rviz2`

`rviz2` is the primary 3D visualization tool for ROS 2. It allows you to display a robot's model, sensor data, and other critical information in a dynamic 3D environment.

To load and visualize your URDF:

1.  **Ensure ROS 2 is sourced**: Open a terminal and source your ROS 2 setup (`source /opt/ros/humble/setup.bash`).

2.  **Launch `rviz2`**:
    ```bash
    ros2 run rviz2 rviz2
    ```

3.  **Add RobotModel Display**:
    *   In the `rviz2` interface, click "Add" in the "Displays" panel.
    *   Select "RobotModel" and click "OK".

4.  **Configure RobotModel**:
    *   In the "RobotModel" properties, under "Description File", you need to specify the path to your URDF file.
    *   Set the "Fixed Frame" to `base_link` (or the first link in your robot's URDF).

5.  **Publish Robot State**:
    `rviz2` needs a `robot_state_publisher` to read the joint states from your URDF and publish the full transformation tree.
    ```bash
    ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="<robot_description_content_from_urdf>"
    ```
    Alternatively, for simple visualization, you can use `urdf_tutorial` which launches a dummy joint state publisher:
    ```bash
    ros2 launch urdf_tutorial display.launch.py model:=$(ros2 pkg prefix urdf_tutorial)/share/urdf_tutorial/urdf/01-myfirst.urdf
    ```
    (Note: The `urdf_tutorial` is a convenient way to quickly visualize URDFs without setting up a full `robot_state_publisher` configuration yourself).

By the end of this chapter, you'll have a fundamental understanding of how to physically describe your robot and bring it to life in a 3D visualization environment.