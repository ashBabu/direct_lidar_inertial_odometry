from launch_ros.actions import Node
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    ld = LaunchDescription()
    mapping_pkg = get_package_share_directory('lidar_mapping')
    lm_params_path = PathJoinSubstitution([mapping_pkg, 'config', 'lidar_mapping.yaml'])
    rviz_config_path = PathJoinSubstitution([mapping_pkg, 'rviz', 'lidar_mapping.rviz'])

    # Set default arguments
    arg_pointcloud_topic = LaunchConfiguration('pointcloud_topic', default='ouster/points')
    arg_imu_topic = LaunchConfiguration('imu_topic', default='ouster/imu')
    arg_rviz = LaunchConfiguration('rviz', default='false')

    # Define arguments
    ld.add_action(DeclareLaunchArgument('pointcloud_topic', default_value=arg_pointcloud_topic,
                                        description='Pointcloud topic name'))
    ld.add_action(DeclareLaunchArgument('imu_topic', default_value=arg_imu_topic,
                                        description='IMU topic name'))
    ld.add_action(DeclareLaunchArgument('rviz', default_value='false',
                                        description='whether to launch rviz or not'))

    # lm Odometry Node
    node_lm_odom = Node(
        package='lidar_mapping',
        executable='lm_odom_node',
        output='log',
        parameters=[lm_params_path],
        remappings=[
            ('pointcloud', arg_pointcloud_topic),
            ('imu', arg_imu_topic),
            ('odom', 'lm/odom_node/odom'),
            ('pose', 'lm/odom_node/pose'),
            ('path', 'lm/odom_node/path'),
            ('kf_pose', 'lm/odom_node/keyframes'),
            ('kf_cloud', 'lm/odom_node/pointcloud/keyframe'),
            ('deskewed', 'lm/odom_node/pointcloud/deskewed'),
        ],
    )
    ld.add_action(node_lm_odom)

    # lm Mapping Node
    node_lm_map = Node(
        package='lidar_mapping',
        executable='lm_map_node',
        output='screen',
        parameters=[lm_params_path],
        remappings=[
            ('keyframes', 'lm/odom_node/pointcloud/keyframe'),
        ],
    )
    ld.add_action(node_lm_map)

    # RViz node
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='lm_rviz',
        arguments=['-d', rviz_config_path],
        output='screen',
        condition=IfCondition(arg_rviz)
    )
    ld.add_action(node_rviz)

    return ld
