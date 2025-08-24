import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path

def generate_launch_description():

    # URDF/Xacro file path
    urdf_path = os.path.join(
        get_package_share_path('simulation_final_project'),
        'urdf',
        'differential_drive_robot.urdf.xacro'
    )

    # RViz configuration file path
    rviz_config_path = os.path.join(
        get_package_share_path('simulation_final_project'),
        'config',
        'differential_drive_arm_robot.rviz'   # also check spelling: "differentia_drive.rviz" looked like a typo
    )

    # Process the URDF/Xacro file
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    # joint_state_publisher_gui_node = Node(
    #     package="joint_state_publisher_gui",
    #     executable="joint_state_publisher_gui"
    # )

    # RViz Node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path]
    )

    # # Define paths
    gazebo_launch_path = os.path.join(
        get_package_share_path('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    )

    world_model_path = os.path.join(
        get_package_share_path('simulation_final_project'),
        'worlds',
        'sample_world.sdf'
    )

    gz_ros_bridge_file = os.path.join(
        get_package_share_path('simulation_final_project'),
        'config',
        'gz_ros_bridge.yaml'
    )

    # Gazebo (choose which world you want to load)
    gazebo_sim_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_path),
        launch_arguments={'gz_args': [world_model_path, ' -r']}.items()
    )

    # Spawn Entity Node with Delay
    spawn_entity_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_entity',
                arguments=['-string', Command(['xacro ', urdf_path]), '-name', 'diff_drive_arm_robot'],
                output='screen'
            )
        ]
    )
    
    # # ROS-GZ Bridge Node
    ros_gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': gz_ros_bridge_file}],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz2_node,
        # joint_state_publisher_gui_node,
        gazebo_sim_node,
        spawn_entity_node,
        ros_gz_bridge_node
    ])