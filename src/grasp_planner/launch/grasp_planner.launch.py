
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os 
from ament_index_python.packages import get_package_share_directory
# may raise PackageNotFoundError

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
   
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_package',
            default_value='iiwa_description',
            description='Description package with robot URDF/xacro files. Usually the argument \
                         is not set, it enables use of a custom description.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='iiwa_dual.config.xacro',
            description='URDF/XACRO description file with the robot.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'start_virtual_camera',
            default_value='true',
            description='Start virtual camera',
        )
    )
  
    declared_arguments.append(
        DeclareLaunchArgument(
            'namespace',
            default_value='/',
            description='Namespace of launched nodes, useful for multi-robot setup. \
                         If changed than also the namespace in the controllers \
                         configuration needs to be updated. Expected format "<ns>/".',
        )
    )
   

    # Initialize Arguments
  
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    start_rviz = LaunchConfiguration('start_rviz')
    namespace = LaunchConfiguration('namespace')
    start_virtual_camera = LaunchConfiguration('start_virtual_camera')
   
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare(description_package), 'config', description_file]
            ), ' ',
            'description_package:=', description_package,
            ' ',
            'namespace:=', namespace
        ]
    )

    robot_description = {'robot_description': robot_description_content}
    
    robot_description_kinematics = PathJoinSubstitution(
        [FindPackageShare(description_package), "moveit2", "dual_kinematics.yaml"]
    )
   
    robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('iiwa_bringup'),
            '/launch',
            '/iiwa_dual.launch.py'
        ]),
        launch_arguments={
            'description_package': description_package,
            'description_file': description_file,
            'start_rviz': start_rviz,
          
           
        }.items()
    )

    scene_urdf = get_package_share_directory("iiwa_description") + "/urdf/mesh.urdf"
    
    virtual_camera_node = Node(
        package="ros2_virtual_camera",
        executable="virtual_camera",
        namespace=namespace,
        output="screen",
        parameters=[
            robot_description,
            { "mesh" : scene_urdf}
        ],
        condition=IfCondition(start_virtual_camera)
    )

    robot_mask_node = Node(
        package="ros2_virtual_camera",
        executable="robot_mask",
        namespace=namespace,
        output="screen",
        parameters=[
            robot_description
        ]
    )

    graspnet_service_node = Node(
        package="graspnet_service",
        executable="service",
        namespace=namespace,
        output="screen"
    )
    
    package_share_directory = get_package_share_directory('grasp_planner')
    
    grasp_planning_node = Node(
        package="grasp_planner",
        executable="grasp_planner_service",
        namespace=namespace,
        output="screen",
        parameters=[
            robot_description,
            robot_description_kinematics,
        ],
        arguments=[package_share_directory + '/data/cap_00000_c.png', 
                   package_share_directory + '/data/cap_00000_d.png']
    )
    
    nodes = [
        robot_launch,
        virtual_camera_node,
        robot_mask_node,
        graspnet_service_node,
    #    grasp_planning_node
    ]

    return LaunchDescription(declared_arguments + nodes)