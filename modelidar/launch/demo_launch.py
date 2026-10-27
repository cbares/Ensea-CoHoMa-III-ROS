"""Robot model in RViz, moved by mock_state_publisher (no hardware, no ros2_control)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


package_name = 'modelidar'
urdf_default_file_name = 'wild_thumper.urdf.xacro'
rviz_default_file_name = 'wild_thumper.rviz'


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true',
        ),
        DeclareLaunchArgument(
            'urdf_name',
            default_value=urdf_default_file_name,
            description='URDF model file name',
        ),
        DeclareLaunchArgument(
            'rviz_name',
            default_value=rviz_default_file_name,
            description='RVIZ configuration file name',
        ),
    ]

    use_sim_time = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    urdf = PathJoinSubstitution(
        [FindPackageShare(package_name), 'urdf', LaunchConfiguration('urdf_name')])
    rviz = PathJoinSubstitution(
        [FindPackageShare(package_name), 'urdf', LaunchConfiguration('rviz_name')])

    # xacro output is a string: without ParameterValue, launch tries to parse it as YAML.
    robot_description = ParameterValue(
        Command([PathJoinSubstitution([FindExecutable(name='xacro')]), ' ', urdf]),
        value_type=str)

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_description}],
        ),
        Node(
            package=package_name,
            executable='state_publisher',
            name='mock_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='log',
            arguments=['-d', rviz],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ]

    return LaunchDescription(declared_arguments + nodes)
