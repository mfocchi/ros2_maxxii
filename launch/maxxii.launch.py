import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
   config = os.path.join(
      get_package_share_directory('ros2_maxxii'),
      'config',
      'roboteq_driver.yaml'
      )

   return LaunchDescription([
      Node(
        package='ros2_maxxii',
        executable='maxxii_node',
        namespace='',
        name='maxxii_node',
        parameters=[config]
      )
   ])