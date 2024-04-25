import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    ns = 'drone1'
    world_path = os.path.join(get_package_share_directory('tello_gazebo'), 'worlds', 'demo_track.world')
    urdf_path = os.path.join(get_package_share_directory('tello_description'), 'urdf', 'tello_1.urdf')

    return LaunchDescription([
        # Launch Gazebo with the demo_track world
        ExecuteProcess(cmd=[
            'gzserver', # 'gzserver' for no gui
            '--verbose',
            '-s', 'libgazebo_ros_init.so',  # Publish /clock
            '-s', 'libgazebo_ros_factory.so',  # Provide gazebo_ros::Node
            world_path
        ], output='screen'),

        # Spawn tello.urdf
        Node(package='tello_gazebo', executable='inject_entity.py', output='screen',
             arguments=[urdf_path, '0', '0', '1', '1.57079632679']),

        # Publish static transforms
        Node(package='robot_state_publisher', executable='robot_state_publisher', output='screen',
             arguments=[urdf_path]),

        # Object detection node
        Node(package='tello_gazebo', executable='object_detection_node.py', output='screen'),

        # Liftoff client node to send the takeoff command to the drone
        Node(package='tello_gazebo', executable='takeoff_client_node.py', output='screen',),

        # Gate navigation node
        # MODIFY THIS TO LAUNCH NODE THAT HAS ALL THE MOVEMENT COMMANDS
        #Node(package='tello_gazebo', executable='gate_navigation_node.py', output='screen'),
    ])

