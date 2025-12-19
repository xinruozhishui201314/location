from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # 获取包路径
    pkg_share = FindPackageShare('fused_localization').find('fused_localization')
    
    # 配置文件路径
    config_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'fused_localization.yaml'
    ])
    
    # 声明启动参数
    config_arg = DeclareLaunchArgument(
        'config',
        default_value=config_file,
        description='配置文件路径'
    )
    
    # 创建节点
    fused_localization_node = Node(
        package='fused_localization',
        executable='fused_localization_node',
        name='fused_localization',
        output='screen',
        parameters=[LaunchConfiguration('config')],
        remappings=[
            ('lidar/points', '/livox/lidar'),
            ('imu/data', '/imu/data'),
            ('camera/image', '/camera/image_raw'),
            ('gps/fix', '/gps/fix'),
            ('rtk/odometry', '/rtk/odometry'),
        ]
    )
    
    return LaunchDescription([
        config_arg,
        fused_localization_node,
    ])

