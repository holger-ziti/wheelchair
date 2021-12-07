import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    
    # if not using simulation, set to false for autorepeat etc.
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    package_dir = get_package_share_directory('force_joystick')

    included_launch = os.path.join(
        get_package_share_directory('force_joystick'), 
        'launch',
        'force2joy.launch.py')
           
    include1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            included_launch),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        )


    joy_sim_node = Node(
            package='joy_animation',
            executable='animation_1',
            name='animation_1'
        )
        
    ld.add_action(include1)
    ld.add_action(joy_sim_node)

    return ld
