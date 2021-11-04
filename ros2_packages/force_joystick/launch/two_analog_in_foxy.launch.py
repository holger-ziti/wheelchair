from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    node1 = Node(
            package='force_joystick',
            executable='voltage',
            name='test1',
            parameters=[
                {'analog_input_number': 0}
            ],
            output='screen',
            emulate_tty=True
        )
    node2 = Node(
            package='force_joystick',
            executable='voltage',
            name='test2',
            parameters=[
                {'analog_input_number': 1}
            ],
            output='screen',
            emulate_tty=True
        )
    node3 = Node(
            package='force_joystick',
            executable='force_cmd.py',
            name='test3',
            parameters=[
                {'raw_voltage_number_1': 0},
                {'raw_voltage_number_2': 1},
                {'voltage_offset_1': 742},
                {'voltage_offset_2': 742}
            ],
            output='screen',
            emulate_tty=True
        )
    ld.add_action(node1)
    ld.add_action(node2)
    ld.add_action(node3)
    return ld
