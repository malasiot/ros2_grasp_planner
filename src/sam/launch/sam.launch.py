import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('sam'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='sam',
            name='segmentation_service',
            executable='seg_points_srv',
            parameters=[config]
        ),
 #       Node(
 #           package='sam',
 #           name='segmentation_client',
 #           executable='seg_points_client'
  #      )
    ])
