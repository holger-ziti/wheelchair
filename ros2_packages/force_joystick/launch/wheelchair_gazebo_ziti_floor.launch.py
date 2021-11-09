#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    world = LaunchConfiguration(
        'world',
        default=os.path.join(
            get_package_share_directory('wheelchair_gazebo'),
            'worlds',
            'fourth_level.world'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='true') 
        
    world_launch = os.path.join(
        get_package_share_directory('wheelchair_gazebo'), 
        'launch',
        'world.launch.py')     

        
    os.environ['GAZEBO_MODEL_PATH'] = os.path.join(
        get_package_share_directory('wheelchair_gazebo'),
        'models')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(world_launch),
            launch_arguments={'world': world, 'use_sim_time': use_sim_time}.items(),
        ),

    ])
