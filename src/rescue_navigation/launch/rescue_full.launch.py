from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def setup(context):

    scenario = LaunchConfiguration('scenario').perform(context)

    if scenario not in ['w1', 'w2', 'w3', 'w4', 'w5']:
        raise RuntimeError('Velg scenario w1, w2, w3, w4 eller w5')

    challenge = get_package_share_directory('multi_robot_challenge_23')
    rescue = get_package_share_directory('rescue_navigation')

    world_path = os.path.join(
        challenge, 'worlds', f'dat160_{scenario}.world'
    )

    map_path = os.path.join(
        challenge, 'maps', f'map_dat160_{scenario}.yaml'
    )

    positions = {'w1': {'first_tb3_pos': ['0.0', '-1.0', '0.0'], 'second_tb3_pos': ['0.0', '1.0', '0.0'], 'first_tb3_yaw': '0.0', 'second_tb3_yaw': '0.0'}, 'w2': {'first_tb3_pos': ['-1.0', '1.0', '0.0'], 'second_tb3_pos': ['-1.0', '-1.0', '0.0'], 'first_tb3_yaw': '0.0', 'second_tb3_yaw': '0.0'}, 'w3': {'first_tb3_pos': ['-1.0', '0.5', '0.0'], 'second_tb3_pos': ['-1.0', '-1.5', '0.0'], 'first_tb3_yaw': '0.0', 'second_tb3_yaw': '0.0'}, 'w4': {'first_tb3_pos': ['-0.5', '0.7', '0.0'], 'second_tb3_pos': ['-0.5', '-1.3', '0.0'], 'first_tb3_yaw': '0.0', 'second_tb3_yaw': '0.0'}, 'w5': {'first_tb3_pos': ['0.0', '-0.5', '0.0'], 'second_tb3_pos': ['0.0', '-2.5', '0.0'], 'first_tb3_yaw': '0.0', 'second_tb3_yaw': '0.0'}}
    p = positions[scenario]

    y0 = float(p['first_tb3_pos'][1])
    y1 = float(p['second_tb3_pos'][1])

    split_y = (y0 + y1) / 2
    lower_robot = 'tb3_0' if y0 < y1 else 'tb3_1'

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(challenge, 'launch', 'rescue_robots_w1.launch.py')
        ),
        launch_arguments={
            'world_file': world_path,
            'robot0_x': p['first_tb3_pos'][0],
            'robot0_y': p['first_tb3_pos'][1],
            'robot1_x': p['second_tb3_pos'][0],
            'robot1_y': p['second_tb3_pos'][1],
            'robot0_yaw': p['first_tb3_yaw'],
            'robot1_yaw': p['second_tb3_yaw'],
            'map_path': map_path
        }.items()
    )

    rescue_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rescue, 'launch', 'rescue_system.launch.py')
        ),
        launch_arguments={
            'map_path': map_path,
            'split_y': str(split_y),
            'lower_robot': lower_robot
        }.items()
    )

    return [
        rescue_system,
        TimerAction(period=10.0, actions=[simulation])
    ]


def generate_launch_description():

    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='w1'),
        OpaqueFunction(function=setup)
    ])
