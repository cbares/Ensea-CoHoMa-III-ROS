from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    sbus_port = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyAMA5',
        description='Serial port for SBUS communication'
    )
    sbus_baud = DeclareLaunchArgument(
        'baudrate',
        default_value='100000',
        description='Baudrate for SBUS communication'
    )
    
    sbus_node = Node(
        package='sbus_serial',
        executable='sbus_serial_node',
        name='sbus_serial_node',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate')
        }]
    )
    
    sbus_cmd_vel_node = Node(
        package='sbus_serial',
        executable='sbus_cmd_vel_node',
        name='sbus_cmd_vel_node',
        remappings=[('/output/sbus/cmd_vel','/bicycle_steering_controller/reference')],
        parameters=[{
            'useStamped': True,
        }]
    )

    send_udp = Node(
        package='send_udp',
        executable='send_udp',
        name='send_udp',
        parameters=[{
        }]
    )

    return LaunchDescription([
        sbus_port,
        sbus_baud,
        sbus_node, 
        sbus_cmd_vel_node,
        send_udp,   
    ])