---
id: chapter1-introduction-to-ros2
title: "Chapter 1: Introduction to ROS 2"
sidebar_label: "Chapter 1: Intro to ROS 2"
---

## Introduction: The Dawn of Physical AI

Welcome to the cutting edge of robotics and artificial intelligence! This textbook, "Physical AI & Humanoid Robotics," is your guide to understanding and building the intelligent machines that are rapidly becoming a part of our world. We stand at the precipice of a new era where AI is not just confined to screens but is embodied in physical forms, interacting with the real world—this is the essence of Physical AI.

Humanoid robots, in particular, represent a grand challenge and a fascinating application of Physical AI. They are designed to operate in human-centric environments, requiring sophisticated perception, decision-making, and control capabilities. To achieve this, a robust and flexible software framework is essential, and that's where ROS 2 comes in.

This chapter will lay the groundwork for our journey. We will introduce the core concepts of Physical AI and humanoid robotics, provide an overview of ROS 2 as the central nervous system for our robots, and guide you through setting up your development environment.

## Learning Outcomes

By the end of this chapter, you will be able to:

*   Understand the fundamental concepts of Physical AI and its unique challenges and applications in humanoid robotics.
*   Describe the role and architectural principles of ROS 2 as a foundational framework for developing complex robotic systems.
*   Identify and differentiate between the core components of ROS 2, including Nodes, Topics, Services, and Actions.
*   Successfully set up a ROS 2 Humble Hawksbill development environment on Ubuntu 22.04, choosing between on-premise installation or a cloud-native (Docker) approach.
*   Explain the purpose and usage of essential ROS 2 development tools like `colcon`.

## Required Skills and Tools

### Prerequisite Skills

*   **Basic Linux Command-Line Interface (CLI)**: You should be comfortable navigating directories, executing commands, and managing files in a Linux terminal.
*   **Text Editor Familiarity**: Ability to use a text editor (e.g., `nano`, `vi`, VS Code, gedit) to modify configuration files.
*   **(Optional) Basic Programming Concepts**: While not strictly required for this chapter, a basic understanding of variables, data types, and functions will be beneficial for later chapters.

### Tools & Software

*   **Operating System**: A computer running **Ubuntu 22.04 LTS (Jammy Jellyfish)**. A virtual machine or dual-boot setup is acceptable.
*   **Internet Connection**: Required for downloading ROS 2 packages and dependencies.
*   **(Optional) Docker**: If you plan to use the cloud-native (containerized) installation method.

## Weekly Breakdown

This chapter is designed to be completed within one week.

*   **Day 1-2: Introduction to Physical AI and ROS 2 Concepts**
    *   **Activity**: Read the "Introduction: The Dawn of Physical AI" and "Overview of ROS 2 as the Robotic Nervous System" sections. Focus on understanding the core definitions and the fundamental role of ROS 2 components (Nodes, Topics, Services, Actions).
    *   **Assessment**: Be able to verbally explain the concept of Physical AI and the purpose of each core ROS 2 component.

*   **Day 3-5: ROS 2 Environment Setup**
    *   **Activity**: Follow the "Installation Guide for ROS 2 Humble Hawksbill on Ubuntu 22.04" section. Carefully choose either the on-premise or Docker installation method and meticulously follow all steps to set up your development environment.
    *   **Assessment**: Successfully complete the ROS 2 installation. Verify your setup by running a simple ROS 2 publisher-subscriber example (e.g., `ros2 run demo_nodes_cpp talker` and `ros2 run demo_nodes_py listener` in separate terminals) to confirm basic communication functionality.

*   **Day 6-7: Exploration and Troubleshooting**
    *   **Activity**: Spend time exploring your new ROS 2 environment. Experiment with different `ros2` command-line tools (e.g., `ros2 node list`, `ros2 topic list`). Review any official ROS 2 documentation for Humble Hawksbill that elaborates on the concepts introduced. Troubleshoot any installation or setup issues you encountered.
    *   **Assessment**: Document any significant challenges you faced during installation and how you resolved them. Ensure your ROS 2 environment is fully functional and ready for development in subsequent chapters.

## Lab Setup Requirements

The primary lab setup requirement for this chapter is a correctly installed and configured ROS 2 Humble Hawksbill environment.

1.  **Operating System**:
    *   **Ubuntu 22.04 LTS (Jammy Jellyfish)** is required. This can be a native installation, a dual-boot setup, or a virtual machine (e.g., using VirtualBox or VMware).

2.  **ROS 2 Installation**:
    *   Either a full **on-premise ROS 2 Humble Desktop installation** (Option 1 in the chapter) or a functional **Docker installation with ROS 2 Humble Desktop Full image** (Option 2) is necessary.
    *   Ensure all post-installation steps, particularly sourcing the setup scripts and installing `colcon`, are completed.

3.  **Basic Tools**:
    *   Access to a **terminal/command line interface**.
    *   A **text editor** for modifying configuration files (if necessary during troubleshooting).

4.  **Internet Connectivity**:
    *   Stable internet access is needed during the installation process to download packages and updates.

5.  **Verification**:
    *   The successful execution of the `ros2 run demo_nodes_cpp talker` and `ros2 run demo_nodes_py listener` example, as described in the "Weekly Breakdown," confirms the lab setup is correct.

## Overview of ROS 2 as the Robotic Nervous System

The Robot Operating System (ROS) is a flexible framework for writing robot software. It's a collection of tools, libraries, and conventions that aim to simplify the task of creating complex and robust robot behavior across a wide variety of robotic platforms. ROS 2 is the latest iteration of this framework, re-architected to address the demands of modern robotics applications, including real-time control, multi-robot systems, and embedded platforms.

Think of ROS 2 as the central nervous system for your humanoid robot:

*   **Nodes**: Like neurons, these are individual processes that perform computation (e.g., a node to read sensor data, a node to control motors, a node for AI processing).
*   **Topics**: Like axons and dendrites, these provide a publish-subscribe mechanism for nodes to exchange data asynchronously (e.g., a sensor node publishes "camera_feed" data, and a perception node subscribes to it).
*   **Services**: For synchronous request-reply interactions (e.g., a high-level AI node requests a low-level motor controller to "move_arm_to_position").
*   **Actions**: For long-running, goal-oriented tasks that provide feedback during execution (e.g., an AI agent sends a goal to "navigate_to_kitchen" and receives updates on progress).

ROS 2 provides the communication backbone, allowing disparate software components (written in different programming languages by different teams) to work together seamlessly. This modularity is crucial for complex systems like humanoid robots, where many different functions—from perception and planning to manipulation and locomotion—must coexist and cooperate.

## Installation Guide for ROS 2 Humble Hawksbill on Ubuntu 22.04

Setting up your ROS 2 development environment is a critical first step. While ROS 2 can run on various operating systems, **Ubuntu 22.04 LTS (Jammy Jellyfish)** is the recommended and most widely supported platform for ROS 2 Humble Hawksbill. We will focus on this setup.

### Option 1: On-Premise Installation (Recommended for Desktop/Laptop)

This option involves installing ROS 2 directly on your physical machine.

1.  **Configure Locale**:
    Ensure you have a "UTF-8" locale.
    ```bash
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    ```
    Verify with `locale`.

2.  **Add ROS 2 Repository**:
    ```bash
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```

3.  **Install ROS 2 Packages**:
    We recommend the "desktop" install, which includes ROS, RViz, and demos.
    ```bash
    sudo apt update
    sudo apt upgrade
    sudo apt install ros-humble-desktop
    ```
    For development tools:
    ```bash
    sudo apt install ros-dev-tools
    ```

4.  **Source the Setup Script**:
    Each time you open a new terminal, you need to source the ROS 2 setup script:
    ```bash
    source /opt/ros/humble/setup.bash
    ```
    To automatically source it in new terminals, add the above line to your `~/.bashrc` file.

5.  **Install colcon (ROS 2 Build Tool)**:
    ```bash
    sudo apt install python3-colcon-common-extensions
    ```

### Option 2: Cloud-Native Installation (e.g., via Docker)

For those who prefer a containerized environment or do not have Ubuntu 22.04, Docker offers a convenient way to run ROS 2.

1.  **Install Docker**:
    Follow the official Docker installation guide for your operating system: [https://docs.docker.com/engine/install/](https://docs.docker.com/engine/install/)

2.  **Pull ROS 2 Humble Docker Image**:
    ```bash
    docker pull ros:humble-ros-base # For a minimal setup
    docker pull ros:humble-desktop-full # For desktop environment with tools
    ```

3.  **Run ROS 2 Container**:
    To run an interactive container with GUI support (for RViz, etc.), you'll need X server forwarding.
    ```bash
    xhost +local:docker
    docker run -it --rm \
        --net=host \
        --env="DISPLAY" \
        --env="QT_X11_NO_MITSHM=1" \
        -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        ros:humble-desktop-full
    ```
    Inside the container, ROS 2 is already sourced.

This chapter provides the essential foundation. With ROS 2 installed, you are ready to begin exploring the world of physical AI and humanoid robotics.