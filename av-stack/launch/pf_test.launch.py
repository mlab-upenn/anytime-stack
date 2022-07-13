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
        get_package_share_directory('f1tenth_cmu_stack_cpp'),
        'config',
        'joy_teleop.yaml'
    )
    vesc_config = os.path.join(
        get_package_share_directory('f1tenth_cmu_stack_cpp'),
        'config',
        'vesc.yaml'
    )
    ekf_config = os.path.join(
        get_package_share_directory('f1tenth_cmu_stack_cpp'),
        'config',
        'ekf.yaml'
    )
    sensors_config = os.path.join(
        get_package_share_directory('f1tenth_cmu_stack_cpp'),
        'config',
        'sensors.yaml'
    )
    mux_config = os.path.join(
        get_package_share_directory('f1tenth_cmu_stack_cpp'),
        'config',
        'mux.yaml'
    )
    localize_config = os.path.join(
        get_package_share_directory('f1tenth_cmu_stack_cpp'),
        'config',
        'amcl.yaml'
    )
    nav2_config = os.path.join(
        get_package_share_directory('f1tenth_cmu_stack_cpp'),
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
    throttle_interpolator_node = Node(
        package='f1tenth_cmu_stack',
        executable='throttle_interpolator',
        name='throttle_interpolator',
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
    robot_localization_node = Node(
       package='robot_localization',
       executable='ekf_node',
       name='ekf_filter_node',
       parameters=[LaunchConfiguration('ekf_config')]
    )
    joy_teleop_node = Node(
        package = 'f1tenth_cmu_stack_cpp',
        executable='joy_code',
        name = 'joy_teleop_node',
        output = 'screen'
    )
    laser_node = Node(
        package = 'f1tenth_cmu_stack_cpp',
        executable='laser_code',
        name = 'laser_node',
        output = 'screen'
    )  
    tf_publish_node = Node(
        package = 'f1tenth_cmu_stack_cpp',
        executable = 'tf_publish',
        name = 'tf_publish_node',
    )
    ego_robot_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='ego_robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', os.path.join(get_package_share_directory('f1tenth_cmu_stack_cpp'), 'description', 'racecar.xacro')])}],
        remappings=[('/robot_description', 'ego_robot_description')]
    )
    nav_lifecycle_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': False},
                    {'autostart': True},
                    {'node_names': ['map_server']}]
    )
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        parameters=[{'yaml_filename': '/home/f1tenth2/f1tenth_ros2_ws/src/f1tenth_cmu_stack_cpp/maps/savedMap.yaml'},
                    {'topic': 'map'},
                    {'frame_id': 'map'},
                    {'output': 'screen'},
                    {'use_sim_time': False}]
    )
    recovery_server_node = Node(
            package='nav2_recoveries',
            executable='recoveries_server',
            name='recoveries_server',
            output='screen',
            parameters=[nav2_config, {'use_sim_time': False}],
            remappings=remappings
    )
    bt_navigator_node = Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_config, {'use_sim_time': False}],
            remappings=remappings
    )
    planner_node = Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_config, {'use_sim_time': False}],
            remappings=remappings
    )
    controller_node = Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[nav2_config, {'use_sim_time': False}],
            remappings=remappings
    )
    pf_node = Node(
        package='particle_filter',
        executable='particle_filter',
        name='particle_filter',
        parameters=[LaunchConfiguration('localize_config')]
    )
    pid_node = Node(
        package = 'f1tenth_cmu_stack_cpp',
        executable='pid',
        name = 'pid_wall',
    )
    jlb_pid_node = Node(
        package='jlb_pid',
        namespace='wallfollow/pid',
        executable='controller_node',
        name='distance_controller',
        parameters=[
            # Required
            {'kp': 0.40},
            {'ki': 0.15},
            {'kd': 0.30},
            # Optional
            {'upper_limit': 0.3},
            {'lower_limit': -0.3},
            {'windup_limit': 0.001},
            {'update_rate': 100.00}
        ]
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(get_package_share_directory('f1tenth_cmu_stack_cpp'), 'description', 'rviz_launch.rviz')]
    )

    # finalize
    #ld.add_action(joy_node)
    #ld.add_action(ackermann_to_vesc_node)
    #ld.add_action(vesc_to_odom_node)
    #ld.add_action(vesc_driver_node)
    #ld.add_action(robot_localization_node)
    #ld.add_action(throttle_interpolator_node)
    #ld.add_action(urg_node)
    #ld.add_action(ackermann_mux_node)
    ld.add_action(map_server_node)
    #ld.add_action(ego_robot_publisher)
    #ld.add_action(tf_publish_node)
    ld.add_action(nav_lifecycle_node)
    #ld.add_action(joy_teleop_node)
    ld.add_action(pf_node)
    #ld.add_action(recovery_server_node)
    #ld.add_action(bt_navigator_node)
    #ld.add_action(planner_node)
    #ld.add_action(controller_node)
    #ld.add_action(jlb_pid_node)
    #ld.add_action(pid_node)
    #ld.add_action(rviz_node)


    return ld
