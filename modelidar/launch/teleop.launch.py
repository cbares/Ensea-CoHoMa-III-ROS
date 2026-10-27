r"""
Joystick teleoperation, on top of modelidar.launch.py (which starts twist_mux).

    ros2 launch modelidar teleop.launch.py joy_config:=xbox

Hold the enable button (deadman) to drive, and the turbo button for full speed. The
buttons and axes come from the teleop_twist_joy preset `joy_config` (xbox, ps3, ps5,
atk3...), the speeds from config/joystick.yaml.

Keyboard (needs its own terminal, so it is not started here):

    ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args \
      --params-file $(ros2 pkg prefix modelidar)/share/modelidar/config/teleop.yaml \
      -r cmd_vel:=/keyboard_teleop/cmd_vel
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    joy_config = LaunchConfiguration('joy_config')
    preset = PathJoinSubstitution(
        [FindPackageShare('teleop_twist_joy'), 'config', [joy_config, '.config.yaml']])
    speeds = PathJoinSubstitution(
        [FindPackageShare('modelidar'), 'config', 'joystick.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'joy_config', default_value='xbox',
            description='teleop_twist_joy button preset: xbox, ps3, ps5, atk3, xd3...'),
        DeclareLaunchArgument(
            'joy_dev', default_value='0', description='Joystick device id (SDL)'),
        Node(
            package='joy',
            executable='joy_node',
            parameters=[{'device_id': LaunchConfiguration('joy_dev'),
                         'deadzone': 0.1,
                         'autorepeat_rate': 20.0}],
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[preset, speeds],  # speeds override the preset
            remappings=[('cmd_vel', '/joy_teleop/cmd_vel')],
        ),
    ])
