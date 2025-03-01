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
                'ffmpeg_image_transport.encoding': 'hevc_v4l2m2m',
                # 'ffmpeg_image_transport.encoding': 'hevc_nvec',
                # 'ffmpeg_image_transport.encoding': 'libx265',
                'ffmpeg_image_transport.profile': 'main',
                'ffmpeg_image_transport.preset': 'ultrafast',
                'ffmpeg_image_transport.tune': 'zerolatency',
                'ffmpeg_image_transport.gop_size': 15,
                'image_raw.enable_pub_plugins': ['image_transport/ffmpeg'],
                'video_device': '/dev/video0',
                }],
            ),
    ])