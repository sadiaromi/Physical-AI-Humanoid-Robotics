# Chapter 1: Introduction to ROS 2

## Purpose of Physical AI & Humanoid Robotics

Physical AI is an exciting field that bridges artificial intelligence with the physical world through robotic systems. Humanoid robots, with their human-like form and capabilities, offer a unique platform for exploring complex interactions, learning, and autonomous decision-making in real-world environments. This book aims to provide a comprehensive guide for university-level students and researchers to understand, design, and implement Physical AI solutions for humanoid robots.

## Overview of ROS 2 as the Robotic Nervous System

ROS (Robot Operating System) has become the de-facto standard framework for robotics development. ROS 2, the successor to ROS 1, offers significant improvements in performance, security, and real-time capabilities, making it ideal for advanced robotic applications, including humanoid control. We will explore how ROS 2 acts as the "nervous system" of our robotic systems, enabling seamless communication, data exchange, and modular development across various hardware and software components.

### Key Concepts of ROS 2

-   **Nodes**: Executable processes that perform computation.
-   **Topics**: A publish/subscribe mechanism for asynchronous data streaming.
-   **Services**: A request/reply mechanism for synchronous communication.
-   **Actions**: For long-running, preemptable tasks with feedback.
-   **Parameters**: Dynamic configuration of nodes.

## Installation Guide for ROS 2 Humble Hawksbill on Ubuntu 22.04

This section provides a step-by-step guide to setting up your ROS 2 development environment. We will focus on ROS 2 Humble Hawksbill, which is a Long Term Support (LTS) release, on Ubuntu 22.04.

### On-Premise Lab Setup

1.  **Configure Locale:**
    ```bash
    locale  # check for UTF-8
    sudo apt update && sudo apt install locales
    sudo locale-gen en_US en_US.UTF-8
    sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
    export LANG=en_US.UTF-8
    locale  # verify settings
    ```

2.  **Add ROS 2 Repository:**
    ```bash
    sudo apt install software-properties-common
    sudo add-apt-repository universe
    sudo apt update && sudo apt install curl
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    ```

3.  **Install ROS 2 Packages:**
    ```bash
    sudo apt update
    sudo apt upgrade
    sudo apt install ros-humble-desktop
    ```

4.  **Source ROS 2 Setup Script:**
    ```bash
    source /opt/ros/humble/setup.bash
    ```

    To automatically source ROS 2 every time you open a new terminal, add the above line to your `~/.bashrc` file:

    ```bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    ```

5.  **Install ROS 2 Development Tools:**
    ```bash
    sudo apt install python3-colcon-common-extensions python3-rosdep2
    sudo rosdep init  # if not already initialized
    rosdep update
    ```

### Cloud-Native Lab Setup (Conceptual Outline)

For cloud-native deployments, the general approach involves leveraging containerization (Docker) and cloud-based virtual machines (e.g., AWS EC2, Google Cloud Compute Engine) or specialized robotics platforms (e.g., AWS RoboMaker, NVIDIA Omniverse Cloud). A typical workflow would be:

1.  **Provision a VM:** Launch an Ubuntu 22.04 VM with sufficient resources (CPU, RAM, GPU if needed).
2.  **Install Docker:** Set up Docker on the VM for containerized development.
3.  **ROS 2 Docker Image:** Pull or build a Docker image with ROS 2 Humble pre-installed.
4.  **VS Code Remote Development:** Use VS Code's Remote - SSH or Remote - Containers extension to connect to your cloud VM and develop within the Docker container.

Detailed instructions for specific cloud providers or NVIDIA Omniverse Cloud will be provided in a separate supplementary guide.

## Learning Outcomes

Upon completing this chapter, you should be able to:

-   Understand the core purpose and application domains of Physical AI and Humanoid Robotics.
-   Recognize ROS 2 as a foundational framework for robotic systems.
-   Identify the key communication concepts in ROS 2 (Nodes, Topics, Services, Actions, Parameters).
-   Successfully install and configure ROS 2 Humble Hawksbill on an Ubuntu 22.04 system for on-premise development.
-   Outline the conceptual steps for setting up a cloud-native ROS 2 development environment.

## Required Skills

To get the most out of this chapter, you should have:

-   Basic command-line proficiency in Linux (e.g., navigating directories, executing commands).
-   Familiarity with Python programming fundamentals (variables, data types, control flow).

## Tools & Software

The primary tools and software required for this chapter are:

-   **Operating System**: Ubuntu 22.04 LTS (recommended)
-   **ROS 2 Distribution**: Humble Hawksbill
-   **Python**: Version 3.8+
-   **Text Editor/IDE**: VS Code (recommended) or any other preferred editor.

## Weekly Breakdown

**Week 1: Introduction to Physical AI and ROS 2 Fundamentals**

-   **Learning Objectives**:
    -   Grasp the foundational concepts of Physical AI and its relevance in humanoid robotics.
    -   Understand the role of ROS 2 as a robust robotic framework.
    -   Successfully set up a ROS 2 Humble Hawksbill environment on Ubuntu 22.04.
    -   Familiarize with core ROS 2 communication concepts.

-   **Activities**:
    -   Read Chapter 1 content.
    -   Perform ROS 2 installation steps (On-Premise or begin Cloud-Native setup).
    -   Execute basic ROS 2 commands (e.g., `ros2 run demo_nodes_cpp talker`).
    -   Initial exploration of ROS 2 documentation.

## Assessments

-   **Quiz 1**: Short multiple-choice quiz on Physical AI concepts and ROS 2 fundamentals.
-   **Lab Assignment 1**: Submit a screenshot/log of a successful ROS 2 Humble installation and a simple `talker`/`listener` demonstration.

## Lab Setup Requirements

### On-Premise Lab

-   A computer running Ubuntu 22.04 LTS (preferably a fresh installation).
-   Minimum 8GB RAM, 50GB free disk space.
-   Internet connection for downloading packages.

### Cloud-Native Lab (Conceptual)

-   Access to a cloud provider (e.g., AWS, Google Cloud) or NVIDIA Omniverse Cloud.
-   Familiarity with cloud VM provisioning and Docker basics.
-   VS Code with Remote Development extensions (Remote - SSH, Remote - Containers) for an optimal development experience.
