from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
from launch_ros.actions import Node


def generate_launch_description():

    map_path = LaunchConfiguration('map_path')

    nodes = [
        DeclareLaunchArgument('split_y', default_value='0.0'),
        DeclareLaunchArgument('lower_robot', default_value='tb3_0'),
        DeclareLaunchArgument(
            'map_path',
            default_value=os.path.join(
                get_package_share_directory('multi_robot_challenge_23'),
                'maps', 'map_dat160_w1.yaml'
            )
        )
    ]

    # Start navigasjon, utforsking og markørdeteksjon
    # for begge robotene.
    for robot in ['tb3_0', 'tb3_1']:

        nodes.append(
            Node(
                package='rescue_navigation',
                executable='path_follower',
                name=f'path_follower_{robot}',
                parameters=[{'robot_name': robot, 'map_path': map_path}],
                output='screen'
            )
        )

        nodes.append(
            Node(
                package='rescue_navigation',
                executable='explorer',
                name=f'explorer_{robot}',
                parameters=[{'robot_name': robot, 'map_path': map_path,
                             'split_y': LaunchConfiguration('split_y'),
                             'lower_robot': LaunchConfiguration('lower_robot')}],
                output='screen'
            )
        )

        nodes.append(
            Node(
                package='rescue_navigation',
                executable='marker_detector',
                name=f'marker_detector_{robot}',
                parameters=[{'robot_name': robot}],
                output='screen'
            )
        )

    # Én koordinator for begge robotene
    nodes.append(
        Node(
            package='rescue_navigation',
            executable='robot_coordinator',
            name='robot_coordinator',
            parameters=[{'map_path': map_path}],
            output='screen'
        )
    )

    return LaunchDescription(nodes)