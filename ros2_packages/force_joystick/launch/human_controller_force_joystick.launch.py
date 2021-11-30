from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os.path

def generate_launch_description():
    ld = LaunchDescription()

    base_path = os.path.realpath(get_package_share_directory('force_joystick')) # also tried without realpath
    rviz_path=base_path+'/config/marker.rviz'
    print(rviz_path)

    node0 = Node(
            package = "tf2_ros", 
            executable = "static_transform_publisher",
            arguments = ["0", "0", "0", "0", "0", "0", "map", "marker_frame"]
        )              
    node1 = Node(
            package='joy',
            executable='joy_node',
            name='position_joystick',
            output='screen',
            emulate_tty=True
        )
    node2 = Node(
            package='force_joystick',
            executable='tracking',
            name='point_tracking',
            output='screen',
            emulate_tty=True
        )
    node3 = Node(
            package='rviz2', 
            executable='rviz2', 
            name="rviz2", 
            output='screen', 
            arguments=['-d', str(rviz_path)]
        )
    ld.add_action(node0)
    ld.add_action(node1)
    ld.add_action(node2)
    ld.add_action(node3)

    return ld
