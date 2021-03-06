from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    node1 = Node(
            package='force_joystick',
            node_executable='voltage',
            node_name='analog_voltage_reader_1',
            parameters=[
                {'analog_input_number': 0}
            ],
            output='screen',
            emulate_tty=True
        )
    node2 = Node(
            package='force_joystick',
            node_executable='voltage',
            node_name='analog_voltage_reader_2',
            parameters=[
                {'analog_input_number': 1}
            ],
            output='screen',
            emulate_tty=True
        )

    ld.add_action(node1)
    ld.add_action(node2)
    return ld
