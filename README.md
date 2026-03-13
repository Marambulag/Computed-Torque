# 📝 Recipe: 1-DOF Pendulum Simulation & Control
**Author:** Professor Abdiel Mercado  
**Platform:** ROS 2 Jazzy + Gazebo Harmonic

This guide provides a complete, step-by-step walkthrough to build, simulate, and control a simple pendulum from scratch using effort (torque) control and IMU feedback.

---

## 🏗️ Project Architecture


The system consists of a physical plant (Gazebo) and a control node (Python). Communication is handled via `ros2_control` for actuators and a ROS 2 bridge for sensors.

---

## 🛠️ Phase 1: Creating the Simulation Package
First, we build the "body" of the robot and its physical environment.

1. **Create the Workspace and Package:**
   ```bash
   mkdir -p ~/robot_1dof_ws/src
   cd ~/robot_1dof_ws/src
   ros2 pkg create --build-type ament_cmake robot_1dof_sim
   ```
2. **Setup Internal Folders:**
    ```bash
    mkdir -p robot_1dof_sim/urdf robot_1dof_sim/launch robot_1dof_sim/config robot_1dof_sim/rviz
    ```
3. **Define the Robot (URDF):** 
Create the file ``robot_1dof_sim/urdf/1dof_robot.urdf``.
    - **Links:** Define mass, inertia, and visual shapes.
    - **Joints:** Define the rotation axis (Z-axis).
    - **ros2_control:** Maps the virtual motor to the hardware plugin.
    - **IMU Sensor:** Attached to link1 to measure angular velocity.
4. **Configure Controllers (YAML):**
Create `robot_1dof_sim/config/controllers.yaml`. This file defines the `torque_controller` (ForwardCommandController) acting on `joint1` with an `effort` interface.

---

## 🧠 Phase 2: Creating the Control Package
Now, we create the "brain" using Python.
1. **Generate the Python Package:**
    ```bash
    cd ~/robot_1dof_ws/src
    ros2 pkg create --build-type ament_python robot_1dof_control --dependencies rclpy std_msgs sensor_msgs
    ```
2. **Program the Control Node:**
    Create ``robot_1dof_control/robot_1dof_control/control_senoidal_imu.py``. This script:
    - **Publishes:** A sine wave signal $\tau(t) = A \sin(\omega t)$ to ``/torque_controller/commands``.
    - **Subscribes:** To the IMU topic ``/link1/ruido_imu`` to monitor system response.

---

## 🏗️ Phase 3: Compilation & Setup
We must install the files into the ``install`` directory so ROS 2 can locate them.
1. **Update CMakeLists.txt:**
Ensure ``robot_1dof_sim/CMakeLists.txt`` includes the installation of ``urdf``, ``launch``, ``config``, and ``rviz`` directories.
2. **Build the Workspace:**
    ```bash
    cd ~/robot_1dof_ws
    colcon build --symlink-install
    source install/setup.bash
    ```

---

## 🚀 Phase 4: Launching (Serving the Dish)
You will need two separate terminals.
- **Terminal 1: The Physical Plant (Gazebo + Robot)**
        ```bash
        source ~/robot_1dof_ws/install/setup.bash
        ros2 launch robot_1dof_sim spawn_robot.launch.py
        ```
    *The pendulum will spawn and drop due to gravity.*
- **Terminal 2: The Controller (The Brain)**
        ```bash
        source ~/robot_1dof_ws/install/setup.bash
        ros2 run robot_1dof_control control_senoidal_imu
        ```
    *The pendulum will start swinging based on the torque wave.*

---

## 📊 Phase 5: Visualization
To see real-time graphs of **Torque Input vs. IMU Velocity Output**, open a third terminal:
```bash
ros2 run rqt_plot rqt_plot
```
- **Topic 1:** ``/torque_controller/commands/data[0]`` (Applied Torque)
- **Topic 2:** ``/link1/ruido_imu/angular_velocity/z`` (System Response)