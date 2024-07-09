
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_param_builder import ParameterBuilder
import os 
from ament_index_python.packages import get_package_share_directory
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace

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
            default_value='false',
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

    declared_arguments.append(
        DeclareLaunchArgument(
            'camera_namespace',
            default_value='/camera/camera_213322072140/aligned_depth_to_color/',
            description='Namespace of camera where I should find image_raw and camerainfo topics',
        )
    )

#    declared_arguments.append(
#        DeclareLaunchArgument(
#            'camera_frame',
#            default_value='camera_213322072140_color_optical_frame',
#            description='The topic where camera static tf transform is published',
#        )
#    )
   

    # Initialize Arguments
  
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    start_rviz = LaunchConfiguration('start_rviz')
    namespace = LaunchConfiguration('namespace')
 #   camera_frame = LaunchConfiguration('camera_frame')

    camera_frame = "camera_213322072140_color_optical_frame"
    
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
            'start_rviz': start_rviz
        }.items()
    )
    
    robot_mask_node = Node(
        package="ros2_virtual_camera",
        executable="robot_mask",
        namespace=namespace,
        output="screen",
        parameters=[
            robot_description,
            { "camera_frame": camera_frame }
        ]
    )

    graspnet_service_node = Node(
        package="graspnet_service",
        executable="service",
        namespace=namespace,
        output="screen"
    )
  
    package_share_directory = get_package_share_directory('grasp_planner')
    
    grasp_planning_config = os.path.join(
            package_share_directory,
            'config',
            'params-rs2.yaml'
    )        

    camera_tf_node = Node(
        package = "tf2_ros", 
        executable = "static_transform_publisher",
        arguments = [ "--x",  "-0.030259363850628995", "--y", "0.3985303170963656", "--z",  "1.0012964230101555",
                     "--qw", "-0.24674339982780538", "--qx",  "0.68689410475534141", "--qy",  "-0.63356047244717506", "--qz", "0.25670082050177889",
                      "--frame-id", "iiwa_left_base", "--child-frame-id", "camera_213322072140_color_optical_frame" 
                    ]
    )
    
    grasp_planning_node = Node(
        package="grasp_planner",
        executable="grasp_planner_service",
        namespace=namespace,
        output="screen",
        parameters=[
            robot_description_kinematics,
            grasp_planning_config
        ],
    )
    
    nodes = [
        robot_launch,
        camera_tf_node,
        robot_mask_node,
        graspnet_service_node,
        grasp_planning_node
    ]

    return LaunchDescription(declared_arguments + nodes)