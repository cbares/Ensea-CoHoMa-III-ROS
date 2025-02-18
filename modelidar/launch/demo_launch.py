from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


package_name = 'modelidar'
urdf_default_file_name = 'wild_thumper.urdf.xacro'
rviz_default_file_name = 'wild_thumper.rviz'

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    urdf_file_name = LaunchConfiguration('urdf_name', default=urdf_default_file_name)
    rviz_file_name = LaunchConfiguration('rviz_name', default=rviz_default_file_name)

    urdf  = PathJoinSubstitution([
        FindPackageShare(package=package_name)                              ,
        'urdf',
        urdf_file_name])
    
    rviz = PathJoinSubstitution([
        FindPackageShare(package=package_name),
        'urdf',
        rviz_file_name])
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time,
        description='Use simulation (Gazebo) clock if true',
    )
    
    urdf_arg = DeclareLaunchArgument(
        'urdf_name',
        default_value=urdf_file_name,
        description='URDF model file name',
    )
    
    rviz_arg = DeclareLaunchArgument(
        'rviz_name',
        default_value=rviz_file_name,
        description='RVIZ configuration file name',
    )

    return LaunchDescription([
        use_sim_time_arg,
        urdf_arg,
        rviz_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': Command(['xacro', ' ', urdf])}],
            ),
        Node(
            package='modelidar',
            executable='state_publisher',
            name='mock_state_publisher',
            output='screen'
            ),
        Node(
            package='rviz2',
            executable='rviz2',
            output="log",
            arguments=['-d', rviz]
            ),
    ])