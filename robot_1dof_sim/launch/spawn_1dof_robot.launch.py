import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # ==============================================================================
    # 1. DEFINICIÓN DEL PAQUETE Y RUTAS
    # ==============================================================================
    pkg_name = 'robot_1dof_sim' 
    pkg_path = os.path.join(get_package_share_directory(pkg_name))
    
    urdf_path = os.path.join(pkg_path, 'urdf', '1dof.urdf')
    # rviz_config_path = os.path.join(pkg_path, 'rviz', 'view_robot.rviz')
    
    # NUEVO: Calculamos la ruta absoluta del YAML aquí en Python
    yaml_path = os.path.join(pkg_path, 'config', 'controllers.yaml')

    # ==============================================================================
    # 2. PROCESAR XACRO (Inyección Dinámica)
    # ==============================================================================
    # Le pasamos la ruta completa del YAML a la variable 'yaml_file_path' del URDF
    doc = xacro.process_file(urdf_path, mappings={'yaml_file_path': yaml_path})
    robot_desc = doc.toprettyxml(indent='  ')

    # ==============================================================================
    # 3. PUBLICADOR DEL ESTADO DEL ROBOT
    # ==============================================================================
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # ==============================================================================
    # 4. GAZEBO HARMONIC Y SPAWN DEL ROBOT
    # ==============================================================================
    # Inicia el simulador con un mundo vacío
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Materializa el robot 1-DOF dentro del simulador
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'pendulum_1dof', '-z', '0.1']
    )

    # ==============================================================================
    # 5. PUENTE ROS 2 - GAZEBO (BRIDGE)
    # ==============================================================================
    # Ahora solo cruzamos el reloj general y la ÚNICA IMU que tiene el robot
    bridge_node = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=[
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/link1/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        '/link2/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
    ],
    output='screen'
)

    # ==============================================================================
    # 6. VISUALIZACIÓN EN RVIZ 2
    # ==============================================================================
    # node_rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     arguments=['-d', rviz_config_path],
    #     parameters=[{'use_sim_time': True}]
    # )

    # ==============================================================================
    # 7. CARGAR CONTROLADORES (CON RETARDO)
    # ==============================================================================
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    load_torque_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["torque_controller", "--controller-manager", "/controller_manager"],
    )

    # Retrasos estratégicos para evitar que ros2_control falle antes de que Gazebo esté listo
    delay_broadcaster = TimerAction(period=5.0, actions=[load_joint_state_broadcaster])
    delay_torque = TimerAction(period=6.0, actions=[load_torque_controller])

    # ==============================================================================
    # ORQUESTACIÓN FINAL
    # ==============================================================================
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        bridge_node,
        #node_rviz,
        delay_broadcaster,
        delay_torque
    ])
