ROS2 2-DOF Arm Sim
A lightweight ROS2/Gazebo simulation of a 2-joint planar arm featuring Computed Torque Control and EKF state estimation.

Quick Start
Bash
# Build
colcon build --packages-select robotic_arm_pkg && source install/setup.bash

# Run Simulation
ros2 launch robotic_arm_pkg robotic_arm.launch.py

# Move to Point (x, y)
ros2 service call /set_target_position robotic_arm_interfaces/srv/SetTarget "{x: 0.5, y: 0.4}"
Key Components
Control: Model-based PD (Computed Torque).

Estimation: EKF fusing IMU (accel/gyro).

Kinematics: Full Forward/Inverse solvers.

