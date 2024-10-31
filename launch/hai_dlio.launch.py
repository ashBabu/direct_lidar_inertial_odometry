from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    ld = LaunchDescription()
    mapping_pkg = get_package_share_directory('direct_lidar_inertial_odometry')
    dlio_yaml_path = PathJoinSubstitution([mapping_pkg, 'cfg', 'dlio.yaml'])
    dlio_params_yaml_path = PathJoinSubstitution([mapping_pkg, 'cfg', 'params.yaml'])
    rviz_config_path = PathJoinSubstitution([mapping_pkg, 'launch', 'dlio.rviz'])

    # Set default arguments
    arg_pointcloud_topic = LaunchConfiguration('pointcloud_topic', default='ouster/points')
    arg_imu_topic = LaunchConfiguration('imu_topic', default='ouster/imu')
    arg_rviz = LaunchConfiguration('rviz', default='false')
    arg_simulation = LaunchConfiguration('simulation', default='false')

    # Define arguments
    ld.add_action(DeclareLaunchArgument('pointcloud_topic', default_value=arg_pointcloud_topic,
                                        description='Pointcloud topic name'))
    ld.add_action(DeclareLaunchArgument('imu_topic', default_value=arg_imu_topic,
                                        description='IMU topic name'))
    ld.add_action(DeclareLaunchArgument('rviz', default_value='false',
                                        description='whether to launch rviz or not'))
    ld.add_action(DeclareLaunchArgument('simulation', default_value='false',
                                        description='whether to launch rviz or not'))

    # dlio Odometry Node
    node_dlio_odom = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_odom_node',
        output='log',
        parameters=[dlio_yaml_path, dlio_params_yaml_path, {"simulation": arg_simulation}],
        remappings=[
            ('pointcloud', arg_pointcloud_topic),
            ('imu', arg_imu_topic),
            ('odom', 'dlio/odom_node/odom'),
            ('pose', 'dlio/odom_node/pose'),
            ('path', 'dlio/odom_node/path'),
            ('kf_pose', 'dlio/odom_node/keyframes'),
            ('kf_cloud', 'dlio/odom_node/pointcloud/keyframe'),
            ('deskewed', 'dlio/odom_node/pointcloud/deskewed'),
        ],
    )
    ld.add_action(node_dlio_odom)

    # dlio Mapping Node
    node_dlio_map = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_map_node',
        output='screen',
        parameters=[dlio_yaml_path, dlio_params_yaml_path],
        remappings=[
            ('keyframes', 'dlio/odom_node/pointcloud/keyframe'),
        ],
    )
    ld.add_action(node_dlio_map)

    # RViz node
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='dlio_rviz',
        arguments=['-d', rviz_config_path],
        output='screen',
        condition=IfCondition(arg_rviz)
    )
    ld.add_action(node_rviz)

    return ld
