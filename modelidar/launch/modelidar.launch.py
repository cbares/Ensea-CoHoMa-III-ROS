# Copyright 2020 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression)

from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


package_name = 'modelidar'
urdf_default_file_name = 'wild_thumper.urdf.xacro'
rviz_default_file_name = 'view_robot.rviz'
controller_file = 'modelidar_controllers.yaml'
twist_mux_file = 'twist-mux.yaml'
# Name of the ros2_control tag in the URDF (urdf/wild_thumper.urdf.xacro).
hardware_name = 'Modelidar'
controllers = ['joint_state_broadcaster', f'{package_name}_base_controller']


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value="",
            description="Prefix for joint names, useful for multi-robot setup. If changed than also "
            "joint names in the controller configuration file must be updated accordingly.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "device",
            default_value="/dev/modelidar",
            description="Serial port of the board (/dev/modelidar: see udev/99-modelidar.rules).",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "imu",
            default_value="true",
            description="Start the MPU9250 IMU driver (I2C bus 1), not with the mock hardware.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "log_level",
            default_value="info",
            choices=["debug", "info", "warn", "error", "fatal"],
            description="Log level of all the nodes (debug shows each serial exchange).",
        )
    )

    # Initialize Arguments
    gui = LaunchConfiguration("gui")
    log_level = LaunchConfiguration("log_level")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    device = LaunchConfiguration("device")
    imu = LaunchConfiguration("imu")
    prefix = LaunchConfiguration("prefix")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(package_name), "urdf", urdf_default_file_name]
            ),
            " ",
            "use_mock_hardware:=",
            use_mock_hardware,
            " ",
            "prefix:=",
            prefix,
            " ",
            "device:=",
            device,
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    robot_controllers = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", controller_file]
    )
    
    twist_mux_config = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", twist_mux_file]
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "urdf", rviz_default_file_name]
    )

    # Nodes
    # Controller Manager. The hardware starts UNCONFIGURED, so that a missing board does
    # not abort it: board_supervisor activates it (and the controllers) once it is there.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_controllers,
            {"hardware_components_initial_state.unconfigured": [hardware_name]},
        ],
        output="both",
        arguments=['--ros-args', '--log-level', log_level]
    )
    board_supervisor = Node(
        package=package_name,
        executable="board_supervisor",
        output="both",
        parameters=[{
            "hardware": hardware_name,
            # Wait for the serial device, except with the mock hardware.
            "device": PythonExpression(
                ["'' if '", use_mock_hardware, "'.lower() == 'true' else '", device, "'"]),
            "controllers": controllers,
        }],
        arguments=['--ros-args', '--log-level', log_level]
    )
    # Robot State Publisher from URDF
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        arguments=['--ros-args', '--log-level', log_level]
    )
    # Load and configure the controllers; board_supervisor activates them.
    controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=controllers + [
            "--inactive",
            "--param-file",
            robot_controllers,
            '--ros-args', '--log-level', log_level
        ],
    )
    # Velocity commands go through twist_mux (config/twist-mux.yaml): joystick, then
    # keyboard, then /cmd_vel (navigation, any other tool). Only twist_mux talks to the
    # base controller.
    twist_mux_node = Node(
        package="twist_mux",
        executable="twist_mux",
        output="both",
        parameters=[twist_mux_config],
        remappings=[("cmd_vel_out", f"/{package_name}_base_controller/cmd_vel")],
        arguments=['--ros-args', '--log-level', log_level]
    )
    # IMU: publishes /imu/data_raw and /imu/mag. It calibrates at start (about 1000 samples):
    # the robot must stay still and flat. Settings: mpu9250driver/params/mpu9250.yaml.
    imu_node = Node(
        package="mpu9250driver",
        executable="mpu9250driver",
        name="mpu9250driver_node",
        output="both",
        parameters=[
            PathJoinSubstitution([FindPackageShare("mpu9250driver"), "params", "mpu9250.yaml"]),
            # link of the URDF (ParameterValue: one string, not a list)
            {"frame_id": ParameterValue([prefix, "base_imu_link"], value_type=str)},
        ],
        arguments=['--ros-args', '--log-level', log_level],
        condition=IfCondition(PythonExpression(
            ["'", imu, "'.lower() == 'true' and '", use_mock_hardware, "'.lower() != 'true'"])),
    )
    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file, '--ros-args', '--log-level', log_level],
        condition=IfCondition(gui),
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        controllers_spawner,
        board_supervisor,
        twist_mux_node,
        imu_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
