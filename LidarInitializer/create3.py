from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    lidar_pkg = get_package_share_directory('create3_lidar_slam')
    namespace = LaunchConfiguration('namespace')
    rviz2_config = PathJoinSubstitution(
        [lidar_pkg, 'rviz', 'create3_lidar_slam.rviz'])

    use_sim_time = LaunchConfiguration('use_sim_time')

    namespace_argument = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Robot namespace')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz2_config],

        # Remaps topics used by the 'rviz2' package from absolute (with slash) to relative (no slash).
        # This is necessary to use namespaces with 'rviz2'.
        remappings=[
            ('/tf_static', 'tf_static'),
            ('/tf', 'tf')],
        namespace=namespace,
        output='screen'
    )

    start_async_slam_toolbox_node = Node(
        parameters=[
            get_package_share_directory("create3_lidar_slam") + '/config/mapper_params_online_async.yaml',
            {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        namespace=namespace,
        # Remaps topics used by the 'slam_toolbox' package from absolute (with slash) to relative (no slash).
        # This is necessary to use namespaces with 'slam_toolbox'.
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('/scan', 'scan'),
            ('/map', 'map'),
            ('/map_metadata', 'map_metadata')
        ])

    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['-0.012', '0', '0.144', '0', '0', '0', 'base_footprint', 'laser_frame'],

        # Remaps topics used by the 'tf2_ros' package from absolute (with slash) to relative (no slash).
        # This is necessary to use namespaces with 'tf2_ros'.
        remappings=[
            ('/tf_static', 'tf_static'),
            ('/tf', 'tf')],
        namespace=namespace
    )

    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[
            get_package_share_directory("create3_lidar_slam") + '/config/rplidar_node.yaml'
        ],
        namespace=namespace
    )

    ld = LaunchDescription()

    ld.add_action(namespace_argument)
    ld.add_action(rviz_node)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(namespace_argument)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(static_transform_node)
    ld.add_action(TimerAction(period=2.0, actions=[rplidar_node]))

    return ld
