# Chapter 5: Physics Simulation – Gravity, Collisions, Sensors

## 5.1 Principles of Rigid Body Dynamics in Simulation

Physics simulation is a cornerstone of modern robotics, especially in the development of humanoid robots and AI systems. It allows engineers and researchers to test algorithms, design hardware, and train AI agents in a safe, cost-effective, and repeatable virtual environment. At its heart, physics simulation relies on the principles of rigid body dynamics.

A **rigid body** is an idealized solid body where deformation is neglected. In other words, the distance between any two given points of a rigid body remains constant in time regardless of external forces exerted on it. While real-world robots are flexible, modeling them as rigid bodies simplifies complex calculations and provides a good approximation for many applications.

Simulators like Gazebo and Unity leverage physics engines (e.g., ODE - Open Dynamics Engine, Bullet, PhysX) to solve the equations of motion for rigid bodies. These engines typically work by:

1.  **Representing Objects**: Each object (robot link, environment obstacle) is defined with physical properties such as mass, inertia, position, and orientation.
2.  **Applying Forces and Torques**: External forces (like gravity, motor torques, contact forces) and torques are applied to these bodies.
3.  **Solving Equations of Motion**: The physics engine continuously solves Newton's laws of motion for all interacting bodies. For a rigid body, this involves calculating its linear acceleration (from net force) and angular acceleration (from net torque), then integrating these to update linear and angular velocities, and finally positions and orientations over small time steps.
    *   **Newton's Second Law (Linear)**: \( \vec{F} = m\vec{a} \)
    *   **Newton's Second Law (Angular)**: \( \vec{\tau} = I\vec{\alpha} \), where \( \vec{\tau} \) is torque, \( I \) is the inertia tensor, and \( \vec{\alpha} \) is angular acceleration.
4.  **Collision Detection and Resolution**: Identifying when objects intersect and calculating appropriate contact forces to prevent interpenetration and simulate realistic interactions.

The accuracy and computational cost of a simulation depend heavily on the complexity of the models, the integration methods used (e.g., explicit vs. implicit Euler, Runge-Kutta), and the size of the simulation time steps. Smaller time steps generally lead to higher accuracy but also higher computational cost.

## 5.2 How Gravity, Friction, and Collisions Work in Simulation

These fundamental physical phenomena are crucial for realistic robotic simulations.

### 5.2.1 Gravity

Gravity is a constant force applied to every object with mass in the simulation. In most simulators, it's configured as a vector, typically pointing downwards along the Z-axis (e.g., `0 0 -9.81` m/s²).

-   **Configuration in Gazebo (SDF)**: Gravity is defined within the `<world>` element of an SDF file.
    ```xml
    <world name="default">
      <gravity>0 0 -9.81</gravity>
      <!-- ... other world elements ... -->
    </world>
    ```
-   **Configuration in Unity**: Gravity is a property of the `Physics` settings. You can modify the `Physics.gravity` vector in your scripts or through the Unity Editor (`Edit > Project Settings > Physics`).

### 5.2.2 Collisions

Collision detection is the process of determining if two or more objects in the simulation are interpenetrating or touching. Collision *resolution* is the subsequent process of calculating and applying forces to prevent interpenetration and simulate a physical response (e.g., bouncing, resting contact).

-   **Collision Geometries**: For computational efficiency, collision detection often uses simplified geometric primitives (boxes, spheres, cylinders, capsules) or convex hulls, which are simpler than the detailed visual meshes.
-   **Contact Points and Normals**: When a collision is detected, the physics engine identifies contact points, contact normals (the direction of force), and penetration depth.
-   **Impulse-based vs. Force-based**: Physics engines typically use either impulse-based or force-based methods for collision resolution. Impulse-based methods apply instantaneous changes in momentum at the moment of contact, while force-based methods calculate continuous forces over the contact duration.

### 5.2.3 Friction and Restitution

-   **Friction**: A force that opposes relative motion or attempted motion between two surfaces in contact.
    -   **Static Friction**: Prevents objects from sliding when a tangential force is applied, up to a certain limit.
    -   **Kinetic Friction**: Acts when objects are already sliding, generally with a lower magnitude than static friction.
    -   **Friction Coefficient**: A dimensionless scalar (often denoted \( \mu \)) that characterizes the friction between two surfaces. Higher values mean more friction.
-   **Restitution (Bounciness)**: Determines how "bouncy" a collision is. It's defined by the **coefficient of restitution (COR)**, a value between 0 and 1.
    -   **COR = 0**: A perfectly inelastic collision (objects stick together).
    -   **COR = 1**: A perfectly elastic collision (objects bounce off with no energy loss).
    -   **Configuration**: Both friction and restitution are typically defined as properties of the collision geometry or material.

-   **Configuration in Gazebo (SDF)**: Within the `<collision>` element, `<surface>` properties are used.
    ```xml
    <collision name="my_collision">
      <geometry>...</geometry>
      <surface>
        <friction>
          <ode>
            <mu>0.7</mu>    <!-- Coefficient of friction -->
            <mu2>0.7</mu2>  <!-- Second coefficient of friction -->
          </ode>
        </friction>
        <bounce>
          <restitution_coefficient>0.2</restitution_coefficient> <!-- COR -->
        </bounce>
      </surface>
    </collision>
    ```
-   **Configuration in Unity**: These properties are set on `Physics Material` assets, which can then be assigned to `Collider` components.

## 5.3 Configuring Physics Parameters

Beyond gravity, friction, and restitution, various other parameters can be configured to fine-tune the simulation.

### 5.3.1 General Physics Parameters

-   **Update Rate/Time Step**: How frequently the physics engine calculates updates. A smaller time step (higher update rate) generally improves stability and accuracy but increases computational cost.
-   **Solver Iterations**: The number of iterations the physics solver performs to resolve contacts and joints. More iterations lead to better accuracy in contact resolution but also higher computational cost.
-   **Global Damping**: A parameter that gradually reduces the velocity of all objects in the simulation, helping to stabilize it and prevent perpetual motion.

-   **Configuration in Gazebo (SDF)**:
    ```xml
    <world name="default">
      <physics name="default_physics" default="true" type="ode">
        <max_step_size>0.001</max_step_size> <!-- Time step -->
        <real_time_factor>1.0</real_time_factor> <!-- Simulation speed multiplier -->
        <ode>
          <solver>
            <iterations>50</iterations> <!-- Solver iterations -->
          </solver>
          <constraints>
            <contact_surface_layer>0.001</contact_surface_layer>
            <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
          </constraints>
        </ode>
      </physics>
      <!-- ... -->
    </world>
    ```
-   **Configuration in Unity**: Configured in `Edit > Project Settings > Physics`. Key settings include `Fixed Timestep` (inverse of update rate), `Solver Iterations`, and various `Bounce Threshold` and `Sleep Threshold` parameters.

### 5.3.2 Humanoid Robot Specific Physics Parameters

For humanoid robots, accurate physics parameters are critical for realistic motion and stable control.

-   **Mass and Inertia**: Each link (body part) of the robot needs correctly specified mass and inertia properties. These are often calculated from the CAD models of the robot.
-   **Joint Limits and Dynamics**:
    -   **Joint Limits**: Maximum and minimum angular positions for each joint.
    -   **Joint Dynamics**: Properties like friction (damping) and stiffness (spring constant) at the joints, which affect how smoothly and responsively joints move.
-   **Actuator Properties**: Torque limits, velocity limits, and control gains for the motors driving each joint.

-   **Configuration in URDF/SDF**: These properties are defined within the `<link>` and `<joint>` elements of a robot description file.
    ```xml
    <link name="upper_arm">
      <inertial>
        <mass value="2.5"/>
        <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
      </inertial>
      <!-- ... -->
    </link>

    <joint name="shoulder_joint" type="revolute">
      <parent link="torso"/>
      <child link="upper_arm"/>
      <limit lower="-1.57" upper="1.57" effort="100" velocity="10"/>
      <dynamics damping="0.1" friction="0.01"/>
      <!-- ... -->
    </joint>
    ```

## 5.4 Basics of Sensor Simulation

Simulating sensors is vital for training AI agents that rely on perception to interact with their environment. Simulators provide virtual representations of common sensors.

### 5.4.1 LiDAR (Light Detection and Ranging)

-   **Principle**: Emits laser pulses and measures the time it takes for them to return, creating a point cloud representing the distance to surrounding objects.
-   **Simulation**: In Gazebo, a `gpu_ray` or `ray` sensor type is used, configured with parameters like horizontal/vertical resolution, angle ranges, and scan rate. It essentially performs a series of ray casts into the environment.
-   **ROS 2 Integration**: Simulated LiDAR data is typically published as `sensor_msgs/msg/LaserScan` messages on a ROS 2 topic.

### 5.4.2 Depth Cameras (RGB-D)

-   **Principle**: Provides both a color image (RGB) and a depth map (distance to objects from the camera's perspective).
-   **Simulation**: Simulated as a `depth_camera` or `camera` sensor type with specific image resolution, field of view, and depth sensing capabilities. The depth information is generated by calculating the distance of each pixel to the nearest surface.
-   **ROS 2 Integration**: Publishes `sensor_msgs/msg/Image` for RGB and depth, often compressed, and `sensor_msgs/msg/CameraInfo` for camera calibration parameters.

### 5.4.3 IMU (Inertial Measurement Unit)

-   **Principle**: Measures linear acceleration and angular velocity (gyroscopic data), often including orientation estimates (roll, pitch, yaw) derived from fusing accelerometer and gyroscope data. Some IMUs also include magnetometers.
-   **Simulation**: Simulated by tracking the actual rigid body motion of the link it's attached to. It reports the body's accelerations and angular velocities in its own local frame.
-   **ROS 2 Integration**: Publishes `sensor_msgs/msg/Imu` messages, containing orientation (quaternion), angular velocity, and linear acceleration.

### 5.4.4 Noise Modeling

Realistic sensor simulation often includes noise to mimic real-world sensor imperfections. This can involve adding Gaussian noise, quantization noise, or drift to the simulated data. Simulators often provide parameters to configure different types of noise.

## 5.5 Gazebo vs. Unity for Physics Simulation

Both Gazebo and Unity are powerful platforms for robotics simulation, each with strengths and weaknesses.

### Gazebo

-   **Strengths**:
    -   **Robotics-focused**: Designed specifically for robotics, with deep integration with ROS 2.
    -   **Rich sensor suite**: Extensive, highly configurable, and realistic sensor models (LiDAR, cameras, IMU, contact sensors, etc.).
    -   **Strong physics engine**: Default ODE, with support for Bullet and DART. Good for complex rigid-body interactions.
    -   **Large community & resources**: Many pre-built robot models, environments, and tutorials for ROS users.
-   **Weaknesses**:
    -   **Graphics**: Historically less visually appealing than Unity, though improving.
    -   **Learning Curve**: Can be challenging for users without a strong ROS/Linux background.
    -   **C++ heavy**: Many plugins and core components are in C++, though Python interfaces exist.

### Unity

-   **Strengths**:
    -   **High-fidelity graphics**: Excellent for creating visually rich and immersive environments.
    -   **Broad ecosystem**: Powerful game engine features, asset store, and strong developer tools.
    -   **C# scripting**: More accessible for many developers compared to C++.
    -   **Unity Robotics Hub**: Provides tools and packages (e.g., URDF importer, ROS TCP Connector, ML-Agents) to bridge Unity with ROS 2 and robotics research.
-   **Weaknesses**:
    -   **Less robotics-native**: Requires more setup and specific packages (like Robotics Hub) to integrate fully with ROS 2.
    -   **Sensor models**: While flexible, creating highly realistic physics-based sensor models may require more custom development compared to Gazebo's out-of-the-box offerings.
    -   **Licensing**: Commercial aspects for larger projects.

**Choice**: The choice between Gazebo and Unity often depends on the project's priorities: Gazebo for strong ROS 2 integration and realistic physics/sensor models, Unity for visual fidelity, broader game engine features, and C# development.

## 5.6 Hands-on Examples

### 5.6.1 Gazebo Example: Falling Box (SDF)

This example demonstrates a basic falling box in Gazebo, illustrating gravity and simple collision.

**File**: `code/simulation/physics/falling_box.world`

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="default">
    <!-- A global light source -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- A ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- A simple box model -->
    <model name="falling_box">
      <pose>0 0 5 0 0 0</pose> <!-- Initial position: 5m above ground -->
      <link name="box_link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.083</ixx>
            <iyy>0.083</iyy>
            <izz>0.083</izz>
            <ixy>0.0</ixy>
            <ixz>0.0</ixz>
            <iyz>0.0</iyz>
          </inertia>
        </inertial>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <ambient>0.0 0.0 1.0 1</ambient>
            <diffuse>0.0 0.0 1.0 1</diffuse>
            <specular>0.0 0.0 1.0 1</specular>
          </material>
        </visual>
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.7</mu>
                <mu2>0.7</mu2>
              </ode>
            </friction>
            <bounce>
              <restitution_coefficient>0.2</restitution_coefficient>
            </bounce>
          </surface>
        </collision>
      </link>
    </model>
  </world>
</sdf>
```
To run this in Gazebo, save it as `falling_box.world` in your Gazebo worlds directory or specify its path when launching Gazebo:
```bash
gazebo falling_box.world
```

### 5.6.2 Unity Example: Basic Cube with Physics (C#)

This example shows how to create a simple cube in Unity and attach a Rigidbody component for physics simulation.

**Script**: `Assets/Scripts/SimplePhysicsCube.cs` (or via Unity Editor)

```csharp
using UnityEngine;

public class SimplePhysicsCube : MonoBehaviour
{
    // Start is called before the first frame update
    void Start()
    {
        // Create a new GameObject for the cube
        GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
        cube.name = "DynamicCube";

        // Position the cube
        cube.transform.position = new Vector3(0, 5, 0);

        // Add a Rigidbody component to make it affected by physics
        // This will make it fall due to gravity and interact with other colliders
        Rigidbody rb = cube.AddComponent<Rigidbody>();

        // You can set physics properties through the Rigidbody component
        rb.mass = 1.0f;
        rb.drag = 0.0f;
        rb.angularDrag = 0.05f;
        rb.useGravity = true; // Use global gravity setting

        // Access and set material for collision properties (friction, bounciness)
        // This usually involves creating a Physics Material asset and assigning it
        // For demonstration, we'll just log its presence
        if (cube.GetComponent<Collider>().material == null)
        {
            Debug.Log("Assign a Physics Material to DynamicCube's Collider for custom friction/bounciness.");
        }
    }

    // Update is called once per frame
    void Update()
    {
        // You can apply forces or modify physics in Update/FixedUpdate
        // For example, a continuous upward force:
        // if (GetComponent<Rigidbody>() != null)
        // {
        //     GetComponent<Rigidbody>().AddForce(Vector3.up * 10 * Time.deltaTime);
        // }
    }
}
```
To use this in Unity:
1.  Create a new C# script named `SimplePhysicsCube` in your `Assets/Scripts` folder.
2.  Copy the code above into the script.
3.  Create an empty GameObject in your scene (`GameObject > Create Empty`).
4.  Drag the `SimplePhysicsCube` script onto this empty GameObject in the Inspector.
5.  Run the scene. A cube will appear at `(0, 5, 0)` and fall due to gravity. You can add a `Plane` (`GameObject > 3D Object > Plane`) for it to collide with.
