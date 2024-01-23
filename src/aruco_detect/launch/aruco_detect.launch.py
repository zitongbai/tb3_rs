from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterValue

def generate_launch_description():
    
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'aruco_id',
            default_value='2',
            description='aruco marker id to detect'
        )
    )
    # refer to: https://github.com/IntelRealSense/realsense-ros?tab=readme-ov-file#camera-name-and-camera-namespace
    declared_arguments.append(
        DeclareLaunchArgument(
            'camera_name',
            default_value='camera',
            description='camera name'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'view_image',
            default_value='true',
            description='whether to show the detect result in a window'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'tf_prefix',
            default_value='',
            description='tf prefix'
        )
    )
    
    aruco_id = LaunchConfiguration('aruco_id')
    camera_name = LaunchConfiguration('camera_name')
    tf_prefix = LaunchConfiguration('tf_prefix')
    view_image = LaunchConfiguration('view_image')
    
    
    image_topic = [camera_name, '/color/image_raw']
    image_info_topic = [camera_name, '/color/camera_info']
    camera_link_frame = [camera_name, '_link']
    camera_color_optical_frame = [camera_name, '_color_optical_frame']
    base_link_frame = [tf_prefix, '/base_link']
    
    tf_static_base_link_to_camera_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_static_base_link_to_camera_link',
        output='screen',
        arguments=[
            '0.062', '0.0', '0.1375',   # x, y, z
            '0.0', '0.0', '0.0',        # roll, pitch, yaw
            base_link_frame,
            camera_link_frame,
        ],
    )
    
    detect_aruco_node = Node(
        package='aruco_detect',
        executable='aruco_detect',
        name='aruco_detect',
        output='screen',
        parameters=[{
            'image_topic': image_topic,
            'image_info_topic': image_info_topic,
            'view_image': view_image,
            'aruco_id': aruco_id,
            'camera_color_optical_frame': camera_color_optical_frame,
            'aruco_marker_frame': 'aruco_marker_frame',
        }],
    )
    
    nodes = [
        tf_static_base_link_to_camera_link,
        detect_aruco_node,
    ]
    
    return LaunchDescription(declared_arguments + nodes)