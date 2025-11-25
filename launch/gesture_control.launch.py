import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    """
    Launch file that starts:
    1. Gazebo simulation
    2. Robot state publisher
    3. Spawn robot in Gazebo
    4. Joint state broadcaster
    5. Elbow position controller
    6. Gesture detector node
    """

    pkg_name = 'cv_arm' 
    pkg_share = get_package_share_directory(pkg_name)

    gazebo_ros_pkg = get_package_share_directory('ros_gz_sim')
    gazebo_launch_file = os.path.join(gazebo_ros_pkg, 'launch', 'gz_sim.launch.py')

    urdf_file_path = os.path.join(pkg_share, 'model', 'arm.urdf.xacro')
    controller_config = os.path.join(pkg_share, 'config', 'elbow_controllers.yaml')
    
    robot_description_config = xacro.process_file(urdf_file_path)
    robot_description_raw = robot_description_config.toxml()
    
    robot_description = robot_description_raw.replace(
        '$(find cv_arm)/config/elbow_controllers.yaml',
        controller_config
    )

    # Start Gazebo
    start_gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={'gz_args': '-r empty.sdf'}.items()
    )

    # Robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }]
    )

    # Spawn robot entity in Gazebo
    spawn_entity_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'elbow_robot',
            '-allow_renaming', 'true'
        ],
        output='screen'
    )

    # Delayed joint state broadcaster
    delayed_joint_state_broadcaster = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster'],
                output='screen'
            )
        ]
    )

    # Delayed elbow controller
    delayed_elbow_controller = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['elbow_position_controller'],
                output='screen'
            )
        ]
    )

    # Gesture detector node (start after controllers are loaded)
    delayed_gesture_detector = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='cv_arm',
                executable='gesture_detector',
                name='gesture_detector',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        start_gazebo_cmd,
        robot_state_publisher_node,
        spawn_entity_node,
        delayed_joint_state_broadcaster,
        delayed_elbow_controller,
        delayed_gesture_detector
    ])
