from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


package_name = 'local_usbcam'

def generate_launch_description():
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(package_name), "config", "view_robot.rviz"]
    )

    return LaunchDescription([
        Node(
            package="v4l2_camera",
            executable='v4l2_camera_node',
            name='video_usbcam',
            output='screen',
            parameters=[{
                # 'ffmpeg_image_transport.encoding': 'hevc_v4l2m2m',
                # 'ffmpeg_image_transport.encoding': 'hevc_nvec',
                'ffmpeg_image_transport.encoding': 'libx265',
                'ffmpeg_image_transport.profile': 'main',
                'ffmpeg_image_transport.preset': 'ultrafast',
                'ffmpeg_image_transport.tune': 'zerolatency',
                'ffmpeg_image_transport.gop_size': 15,
                'image_raw.enable_pub_plugins': ['image_transport/ffmpeg'],
                'video_device': '/dev/video0',
                }],
            ),
        Node(
            package='rviz2',
            executable='rviz2',
            output="log",
            additional_env={'ROS_SUPER_CLIENT': 'true'},
            arguments=['-d', rviz_config_file],
            ),
        #ros2 run image_transport republish ffmpeg in/ffmpeg:=image_raw/ffmpeg raw out:=image_raw/uncompressed --ros-args -p "ffmpeg_image_transport.map.hevc_nvenc:=hevc"
        Node(
            package='image_transport',
            executable='republish',
            name='republish_uncompressed',
            output='screen',
            arguments=['ffmpeg', 'raw'],
            parameters=[{
                'ffmpeg_image_transport.map.libx265': 'hevc',
                'in_transport': 'ffmpeg',
                'out_transport': 'raw',
                'out.enable_pub_plugins': ['image_transport/raw'],
                }],
            remappings=[
                ('in/ffmpeg', 'image_raw/ffmpeg'),
                ('out', 'uncompressed'),
            ],
            ),
    ])