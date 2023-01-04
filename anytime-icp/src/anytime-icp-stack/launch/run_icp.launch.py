from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import time

def generate_launch_description():
    joy_teleop_config = os.path.join(
        get_package_share_directory('anytime-icp-stack'),
        'config',
        'joy_teleop.yaml'
    )
    vesc_config = os.path.join(
        get_package_share_directory('anytime-icp-stack'),
        'config',
        'vesc.yaml'
    )
    sensors_config = os.path.join(
        get_package_share_directory('anytime-icp-stack'),
        'config',
        'sensors.yaml'
    )
    mux_config = os.path.join(
        get_package_share_directory('anytime-icp-stack'),
        'config',
        'mux.yaml'
    )
    icp_config = os.path.join(
        get_package_share_directory('anytime-icp-stack'),
        'config',
        'icp_timing.yaml'
    ) 
    stanley_config = os.path.join(
        get_package_share_directory('anytime-icp-stack'),
        'config',
        'stanley.yaml'
    ) 

    joy_la = DeclareLaunchArgument(
        'joy_config',
        default_value=joy_teleop_config,
        description='Descriptions for joy and joy_teleop configs')
    vesc_la = DeclareLaunchArgument(
        'vesc_config',
        default_value=vesc_config,
        description='Descriptions for vesc configs')
    sensors_la = DeclareLaunchArgument(
        'sensors_config',
        default_value=sensors_config,
        description='Descriptions for sensor configs')
    mux_la = DeclareLaunchArgument(
        'mux_config',
        default_value=mux_config,
        description='Descriptions for ackermann mux configs')
    icp_la = DeclareLaunchArgument(
        'icp_config',
        default_value=icp_config,
        description='ICP configs')
    stanley_la = DeclareLaunchArgument(
        'stanley_config',
        default_value=stanley_config,
        description='Stanley configs')

    ld = LaunchDescription([joy_la, vesc_la, sensors_la, mux_la, icp_la, stanley_la])

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        parameters=[LaunchConfiguration('joy_config')]
    )
    ackermann_to_vesc_node = Node(
        package='vesc_ackermann',
        executable='ackermann_to_vesc_node',
        name='ackermann_to_vesc_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_to_odom_node = Node(
        package='vesc_ackermann',
        executable='vesc_to_odom_node',
        name='vesc_to_odom_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    vesc_driver_node = Node(
        package='vesc_driver',
        executable='vesc_driver_node',
        name='vesc_driver_node',
        parameters=[LaunchConfiguration('vesc_config')]
    )
    urg_node = Node(
        package='urg_node',
        executable='urg_node_driver',
        name='urg_node',
        parameters=[LaunchConfiguration('sensors_config')]
    )
    ackermann_mux_node = Node(
        package='ackermann_mux',
        executable='ackermann_mux',
        name='ackermann_mux',
        parameters=[LaunchConfiguration('mux_config')],
        remappings=[('ackermann_cmd_out', 'ackermann_drive')]
    ) 
    icp_node = Node(
        package = 'anytime-icp-stack',
        executable='icp',
        name = 'icp',
        output = 'screen',
        parameters=[LaunchConfiguration('icp_config')]
    )
    controller_node = Node(
        package = 'anytime-icp-stack',
        executable='anytime_stanley',
        name = 'anytime_stanley',
        output = 'screen',
        parameters=[LaunchConfiguration('stanley_config')]
    ) 
    tf_publish_node = Node(
        package = 'anytime-icp-stack',
        executable = 'tf_publish',
        name = 'tf_publish_node',
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('anytime-icp-stack'), 'description', 'rviz_launch.rviz')]
    )

    # finalize
    ld.add_action(joy_node)
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    ld.add_action(urg_node)
    ld.add_action(ackermann_mux_node)
    ld.add_action(tf_publish_node)
    ld.add_action(icp_node)
    ld.add_action(controller_node)


    return ld
