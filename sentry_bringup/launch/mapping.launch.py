import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration 

def generate_launch_description():

  config_path = os.path.join(
      get_package_share_directory('sentry_bringup'), 'params') 

  
  # fast-lio localization   
  fast_lio_param = '/home/sentry_ws/src/sentry_bringup/params/fast_lio_mapping_param.yaml'
  fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        parameters=[
          fast_lio_param
        ],
        output='screen',
        remappings=[('/Odometry','/state_estimation')]
    )

  start_octomap_server = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('sentry_bringup'), 'launch', 'octomap_server_intensity.launch.py')])
  )
        
  rviz_config_file = os.path.join(
    get_package_share_directory('sentry_bringup'), 'rviz', 'loam_livox.rviz')
  start_rviz = Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', rviz_config_file,'--ros-args', '--log-level', 'warn'],
    output='screen'
  )

  delayed_start_mapping = TimerAction(
    period=8.0,
    actions=[
      fast_lio_node,
      start_octomap_server
    ]
  )

  ld = LaunchDescription()

  ld.add_action(start_rviz)
  ld.add_action(delayed_start_mapping)

  return ld