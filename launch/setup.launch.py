
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml



def generate_launch_description():
    param_config = LaunchConfiguration('param_config')
    param_config_cmd = DeclareLaunchArgument(
        'param_config',
        default_value=os.path.join(get_package_share_directory('moving_obstacles_prediction'), 'config', 'moving_obstacles_prediction.yaml'),
        description='')

    object_pt =  Node(
        package='moving_obstacles_prediction',
        executable='object_point_2d_lidar',
        name='object_point_2d_lidar',
        output='screen',
        parameters=[param_config],
    )

    predict_nd =  Node(
        package='moving_obstacles_prediction',
        executable='object_point_prediction',
        name='object_point_prediction',
        output='screen',
        parameters=[param_config],
    )
    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(param_config_cmd)
    # ld.add_action(object_pt)
    ld.add_action(predict_nd)
    return ld