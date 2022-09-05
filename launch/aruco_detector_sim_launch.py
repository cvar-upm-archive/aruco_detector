from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os
import yaml


def staticTransformNode(context, *args, **kwargs):

    config = os.path.join(get_package_share_directory('aruco_detector'),
                          'config/tf_cam',
                          'sim_arguments.yaml')

    with open(config, 'r') as config_file:
        tf_arguments = yaml.load(config_file, Loader=yaml.FullLoader)

    ns = LaunchConfiguration('drone_id').perform(context)

    base_link = ns + '/' + tf_arguments['frame_id']
    camera_link = ns + '/' + tf_arguments['child_frame_id']
    x = tf_arguments['x']
    y = tf_arguments['y']
    z = tf_arguments['z']
    roll = tf_arguments['roll']
    pitch = tf_arguments['pitch']
    yaw = tf_arguments['yaw']

    static_transform_publisher_node = Node(
        # Tf from baselink to RGB cam
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_tf',
        namespace=LaunchConfiguration('drone_id'),
        arguments=[f'{x:.2f}', f'{y:.2f}', f'{z:.2f}', f'{roll:.2f}', f'{pitch:.2f}', f'{yaw:.2f}',
                   f'{base_link}', f'{camera_link}'],
        output='screen',
        emulate_tty=True,
    )

    return [static_transform_publisher_node]


def generate_launch_description(ns='drone0'):

    drone_id = DeclareLaunchArgument('drone_id', default_value='drone0')
    log_level = DeclareLaunchArgument('log_level', default_value='info')

    config = os.path.join(get_package_share_directory('aruco_detector'),
                          'config/aruco_detector',
                          'sim_params.yaml')

    return LaunchDescription([
        drone_id,
        log_level,
        Node(
            package='aruco_detector',
            executable='aruco_detector_node',
            namespace=LaunchConfiguration('drone_id'),
            parameters=[config],
            output='screen',
            emulate_tty=True,
            remappings=[
                ("sensor_measurements/camera/image_raw", "camera1/image_raw"),
                ("sensor_measurements/camera/camera_info", "camera1/camera_info")]
        ),

        OpaqueFunction(function=staticTransformNode)
    ])
