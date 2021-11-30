from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    node1 = Node(
            package='force_joystick',
            executable='force2joy.py',
            name='force2joy',
            parameters=[
                {'raw_voltage_number_1': 1},
                {'raw_voltage_number_2': 0},
                {'voltage_offset_1': 742},
                {'voltage_offset_2': 742},
                {'voltage_to_force_factor_1': 0.015},
                {'voltage_to_force_factor_2': 0.015}
            ],
            output='screen',
            emulate_tty=True
        )
    ld.add_action(node1)

    return ld
