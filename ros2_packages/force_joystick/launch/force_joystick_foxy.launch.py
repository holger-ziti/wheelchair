from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    node1 = Node(
            package='force_joystick',
            executable='force_cmd.py',
            name='test3',
            parameters=[
                {'raw_voltage_number_1': 1},
                {'raw_voltage_number_2': 0},
                {'voltage_offset_1': 742},
                {'voltage_offset_2': 742},
                {'voltage_to_force_factor_1': 0.01},
                {'voltage_to_force_factor_2': 0.01},
                {'damping_param_1': 0.0},
                {'damping_param_2': 1.0}
            ],
            output='screen',
            emulate_tty=True
        )
    ld.add_action(node1)

    return ld
