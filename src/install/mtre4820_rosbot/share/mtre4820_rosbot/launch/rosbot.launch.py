import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_rosbot = get_package_share_directory('mtre4820_rosbot')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    os.environ['GAZEBO_MODEL_PATH'] = (
        pkg_rosbot
        + ':' + os.path.join(pkg_rosbot, 'meshes')
        + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pos = LaunchConfiguration('x_pos', default='0.0')
    y_pos = LaunchConfiguration('y_pos', default='0.0')
    z_pos = LaunchConfiguration('z_pos', default='0.1')

    # Process xacro into robot_description
    urdf_file = os.path.join(pkg_rosbot, 'urdf', 'rosbot.xacro')
    robot_description = Command(['xacro ', urdf_file])

    # Launch classic Gazebo with empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': '',
            'verbose': 'false',
        }.items()
    )

    # Publishes TF from URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }]
    )

    # Spawns ROSbot into Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_rosbot',
        output='screen',
        arguments=[
            '-entity', 'rosbot',
            '-topic', 'robot_description',
            '-x', x_pos,
            '-y', y_pos,
            '-z', z_pos,
        ]
    )

    # Publishes wheel joint states
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x_pos', default_value='0.0'),
        DeclareLaunchArgument('y_pos', default_value='0.0'),
        DeclareLaunchArgument('z_pos', default_value='0.1'),
        gazebo,
        robot_state_publisher,
        spawn_robot,
        joint_state_publisher,
    ])
