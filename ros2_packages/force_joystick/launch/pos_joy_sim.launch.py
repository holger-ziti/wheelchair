import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    
    # if not using simulation, set to false for autorepeat etc.
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    joy_dev = LaunchConfiguration('joy_dev', default='/dev/input/js0')
    joy_config = LaunchConfiguration('joy_config', default='atk3')
    config_filepath = LaunchConfiguration(
        'config_filepath',
        default=[
            TextSubstitution(text=os.path.join(
                get_package_share_directory('teleop_twist_joy'), 'config', '')),
            joy_config,
            TextSubstitution(text='.config.yaml')])

    joy_node = Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[{
                'dev': joy_dev,
                'deadzone': 0.3,
                'autorepeat_rate': 1000.0, # rate (frequency), not time
                'use_sim_time': use_sim_time}],
        )

    joy_sim_node = Node(
            package='joy_animation',
            executable='animation_1',
            name='animation_1'
        )
        
    ld.add_action(joy_node)
    ld.add_action(joy_sim_node)

    return ld
