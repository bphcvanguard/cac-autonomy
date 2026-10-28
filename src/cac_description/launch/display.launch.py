"""Show the rover in RViz: robot_state_publisher + joint_state_publisher + rviz2."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('cac_description')
    with open(os.path.join(share, 'urdf', 'rover.urdf')) as f:
        urdf = f.read()
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': urdf}]),
        Node(package='joint_state_publisher', executable='joint_state_publisher'),
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', os.path.join(share, 'rviz', 'rover.rviz')]),
    ])
