# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
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
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

package_name = 'modelidar'
urdf_default_file_name = 'wild_thumper.urdf.xacro'
rviz_default_file_name = "view_robot.rviz"


def generate_launch_description():
    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument(
            "description_package",
            default_value="modelidar",
            description="Description package with robot URDF/xacro files. Usually the argument " \
        "is not set, it enables use of a custom description.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value=urdf_default_file_name,
            description="URDF/XACRO description file with the robot.",
        ),
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start Rviz2 and Joint State Publisher gui automatically " \
        "with this launch file.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for " \
        "multi-robot setup. If changed than also joint names in the controllers' configuration " \
        "have to be updated.",
        ),
        DeclareLaunchArgument(
            "rviz_default_file",
            default_value=rviz_default_file_name,
            description="Default RViz config file.",
        ),
    ]

    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    gui = LaunchConfiguration("gui")
    prefix = LaunchConfiguration("prefix")
    rviz_default_file = LaunchConfiguration("rviz_default_file")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("modelidar"), "urdf", description_file]
            ),
            " ",
            "use_mock_hardware:=",
            "false",
            " ", 
            "prefix:=",
            prefix,

        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "urdf", rviz_default_file]
    )

    # Nodes
    # Joint State Publisher with GUI
    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        condition=IfCondition(gui),
    )
    # Robot State Publisher from URDF
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file, "-f", "base_footprint"],
        condition=IfCondition(gui),
    )

    nodes = [
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)