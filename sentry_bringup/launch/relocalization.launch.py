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

  # icp relocalization
  map_odom_trans = Node(
      package='icp_relocalization',
      executable='transform_publisher',
      name='transform_publisher',
      output='screen'
  )

  icp_node = Node(
      package='icp_relocalization',
      executable='icp_node',
      name='icp_node',
      output='screen',
      parameters=[
          {'initial_x':0.0},
          {'initial_y':0.0},
          {'initial_z':0.0},
          {'initial_a':0.0},
          # {'initial_roll':0.0},
          # {'initial_pitch':0.0},
          # {'initial_yaw':0.0},
          {'map_voxel_leaf_size':0.1},
          {'cloud_voxel_leaf_size':0.1},
          {'map_frame_id':'map'},
          {'solver_max_iter':75},
          {'map_path':'/home/sentry_ws/src/sentry_bringup/maps/test.pcd'},
          {'fitness_score_thre':0.2}, # 是最近点距离的平均值，越小越严格
      ],
  )
  
  # fast-lio localization   
  fast_lio_param = os.path.join(
      config_path, 'fast_lio_relocalization_param.yaml')
  fast_lio_node = Node(
      package='fast_lio',
      executable='fastlio_mapping',
      parameters=[
          fast_lio_param
      ],
      output='screen',
      remappings=[('/Odometry','/state_estimation')]
  )
        
  rviz_config_file = os.path.join(
    get_package_share_directory('sentry_bringup'), 'rviz', 'loam_livox.rviz')
  start_rviz = Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', rviz_config_file],
    output='screen'
  )

  delayed_start_lio = TimerAction(
    period=3.0,
    actions=[
      fast_lio_node
    ]
  )

  ld = LaunchDescription()

  ld.add_action(map_odom_trans)
  ld.add_action(icp_node)
  ld.add_action(start_rviz)
  ld.add_action(delayed_start_lio)

  return ld