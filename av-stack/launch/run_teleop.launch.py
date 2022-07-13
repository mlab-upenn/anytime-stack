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
        get_package_share_directory('av-stack'),
        'config',
        'joy_teleop.yaml'
    )
    vesc_config = os.path.join(
        get_package_share_directory('av-stack'),
        'config',
        'vesc.yaml'
    )
    ekf_config = os.path.join(
        get_package_share_directory('av-stack'),
        'config',
        'ekf.yaml'
    )
    sensors_config = os.path.join(
        get_package_share_directory('av-stack'),
        'config',
        'sensors.yaml'
    )
    mux_config = os.path.join(
        get_package_share_directory('av-stack'),
        'config',
        'mux.yaml'
    )
    localize_config = os.path.join(
        get_package_share_directory('av-stack'),
        'config',
        'amcl.yaml'
    )
    nav2_config = os.path.join(
        get_package_share_directory('av-stack'),
        'config',
        'nav2.yaml'
    ) 
    
    #localize_config_dict = yaml.safe_load(open(localize_config, 'r'))

    remappings = [('/tf', 'tf'),
                ('/tf_static', 'tf_static')]
    
    map_name = "map3"

    joy_la = DeclareLaunchArgument(
        'joy_config',
        default_value=joy_teleop_config,
        description='Descriptions for joy and joy_teleop configs')
    vesc_la = DeclareLaunchArgument(
        'vesc_config',
        default_value=vesc_config,
        description='Descriptions for vesc configs')
    ekf_la = DeclareLaunchArgument(
        'ekf_config',
        default_value=ekf_config,
        description='Descriptions for ekf configs')
    sensors_la = DeclareLaunchArgument(
        'sensors_config',
        default_value=sensors_config,
        description='Descriptions for sensor configs')
    mux_la = DeclareLaunchArgument(
        'mux_config',
        default_value=mux_config,
        description='Descriptions for ackermann mux configs')
    localize_la = DeclareLaunchArgument(
        'localize_config',
        default_value=localize_config,
        description='Localization configs')
    nav2_la = DeclareLaunchArgument(
        'nav2_la',
        default_value=nav2_config,
        description='Navigation 2 configs')

    ld = LaunchDescription([joy_la, vesc_la, sensors_la, mux_la, localize_la, nav2_la])

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
    joy_teleop_node = Node(
        package = 'av-stack',
        executable='joy_code',
        name = 'joy_teleop_node',
        output = 'screen'
    )
    laser_node = Node(
        package = 'av-stack',
        executable='laser_code',
        name = 'laser_node',
        output = 'screen'
    )  
    tf_publish_node = Node(
        package = 'av-stack',
        executable = 'tf_publish',
        name = 'tf_publish_node',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('av-stack'), 'description', 'rviz_launch.rviz')]
    )

    # finalize
    ld.add_action(joy_node)
    ld.add_action(ackermann_to_vesc_node)
    ld.add_action(vesc_to_odom_node)
    ld.add_action(vesc_driver_node)
    ld.add_action(urg_node)
    ld.add_action(ackermann_mux_node)
    ld.add_action(tf_publish_node)
    ld.add_action(joy_teleop_node)


    return ld
