#!/usr/bin/env python3

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory, get_package_share_path, get_package_prefix

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import IfCondition
# from launch.actions.append_environment_variable import AppendEnvironmentVariable
from launch.actions import ExecuteProcess, AppendEnvironmentVariable
from launch_ros.substitutions import FindPackageShare



# Enum for world types
class WorldType:
    RMUC = 'RMUC'
    RMUL = 'RMUL'

def get_world_config(world_type):
    world_configs = {
        WorldType.RMUC: {
            'x': '0.0',
            'y': '0.0',
            'z': '0.0',
            'roll':'0.0',
            'yaw': '0.0',
            'pitch': '0.0',
            'world_path': 'xzx_gazebo/robocon2026_map_foreset.world'
            # 'world_path': 'RMUC2024_world/RMUC2024_world.world'
        },
        WorldType.RMUL: {
            'x': '4.7',
            # 'x': '4.0',
            'y': '-2.5',
            'z': '0.0',
            'roll':'0.0',
            'yaw': '0.0',  # 90 degrees in radians 3.1416
            'pitch': '0.0',
            # 'world_path': 'xzx_gazebo/robocon2026_new.world'
            'world_path': 'xzx_gazebo/robocon2026_map_foreset_wall.world'
        }
    }
    return world_configs.get(world_type, None)

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('pb_rm_simulation')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Specify xacro path
    urdf_dir = get_package_share_path('pb_rm_simulation') / 'urdf' / 'simulation_waking_robot.xacro'
    # urdf_dir = Path('/home/sentry_ws/src/rm_simulation/pb_rm_simulation/RC_vision_2026/gazebo_for_humble/src/fishbot_description/urdf/point_cloud.urdf')
    # # 先检查文件是否存在，再读取（顺序修正）
    # if not urdf_dir.exists():
    #     raise FileNotFoundError(f"URDF 文件不存在！请检查路径：{urdf_dir}")
    
    # # 读取 URDF 文件内容（此时路径已存在，不会报错）
    # with open(urdf_dir, 'r') as f:
    #     robot_description_content = f.read()


    # Create the launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz', default='true')
    use_joint_state_publisher = LaunchConfiguration('use_joint_state_publisher', default='false')

    # Set Gazebo plugin path
    append_enviroment = AppendEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        os.path.join(os.path.join(get_package_share_directory('pb_rm_simulation'), 'meshes', 'obstacles', 'obstacle_plugin', 'lib'))
    )

    # 设置robot的网格路径
    mesh_path = "/home/sentry_ws/src/rm_simulation/pb_rm_simulation/RC_vision_2026/gazebo_for_humble/install/fishbot_description/share/fishbot_description/meshes"

    # 在当前 Python 进程环境中确保包含 mesh_path（影响当前进程）
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] += os.pathsep + mesh_path
    else:
        os.environ['GAZEBO_MODEL_PATH'] = "/usr/share/gazebo-11/models" + os.pathsep + mesh_path

    # 通过 Launch 的 AppendEnvironmentVariable 确保子进程（gzserver/gzclient）也能继承该路径
    append_gazebo_model_path = AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=':' + mesh_path
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=WorldType.RMUL,
        description='Choose <RMUC> or <RMUL>'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(bringup_dir, 'rviz', 'rviz2.rviz'),
        description='Full path to the RVIZ config file to use'
    )

    declare_use_joint_state_publisher = DeclareLaunchArgument(
        'use_joint_state_publisher',
        default_value='false',
        description='Whether to start joint_state_publisher (default false). Set true only if you need it (e.g. no hardware/plugin publishing /joint_states).'
    )

    # Specify the actions
    gazebo_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),
        launch_arguments={'gui': 'true'}.items()
    )


    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command(['xacro ', str(urdf_dir)]), value_type=str
            ),
            # 'robot_description': robot_description_content  # 直接传入 URDF 内容
        }],
        output='screen'
    )
    # start_joint_state_publisher_cmd = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     name='joint_state_publisher',
    #     # 只有当 use_joint_state_publisher 为 true 时才会启动，避免和插件发布的 /joint_states 冲突
    #     condition=IfCondition(LaunchConfiguration('use_joint_state_publisher')),
    #     output='screen'
    # )

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': ParameterValue(
                Command(['xacro ', str(urdf_dir)]), value_type=str
            ),
            # 'robot_description': ParameterValue(robot_description_content, value_type=str)
            # 'robot_description': robot_description_content  # 直接传入 URDF 内容
        }],
        output='screen'
    )

    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        namespace='',
        executable='rviz2',
        arguments=['-d' + os.path.join(bringup_dir, 'rviz', 'rviz2.rviz')]
    )

    def create_gazebo_launch_group(world_type):
        world_config = get_world_config(world_type)
        if world_config is None:
            return None

        return GroupAction(
            condition=LaunchConfigurationEquals('world', world_type),
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'robot',
                        '-topic', 'robot_description',
                        '-x', world_config['x'],
                        '-y', world_config['y'],
                        '-z', world_config['z'],
                        '-Y', world_config['yaw']
                    ],
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),
                    launch_arguments={'world': os.path.join(bringup_dir, 'world', world_config['world_path'])}.items(),
                )
            ]
        )

    bringup_RMUC_cmd_group = create_gazebo_launch_group(WorldType.RMUC)
    bringup_RMUL_cmd_group = create_gazebo_launch_group(WorldType.RMUL)

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(append_enviroment)
    ld.add_action(append_gazebo_model_path)

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_joint_state_publisher)


    ld.add_action(gazebo_client_launch)
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(bringup_RMUL_cmd_group) # type: ignore
    ld.add_action(bringup_RMUC_cmd_group) # type: ignore

    # Uncomment this line if you want to start RViz
    ld.add_action(start_rviz_cmd)

    return ld
