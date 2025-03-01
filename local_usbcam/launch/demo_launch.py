from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


package_name = 'local_usbcam'

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time,
        description='Use simulation (Gazebo) clock if true',
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        Node(
            package=package_name,
            executable='local_usbcam_start',
            name='local_usbcam',
            output='screen'
            ),
        Node(
            package='rviz2',
            executable='rviz2',
            output="log",
            #arguments=['-d', rviz]
            ),
    ])