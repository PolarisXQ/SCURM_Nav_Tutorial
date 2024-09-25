# SCURM火锅战队 24赛季哨兵导航

For docker guidence, please read [DevcontainterGuide](./DevcontainterGuide.md)

## 包说明

| Package Name | Description |
|--------------|-------------|
| ✅auto_aim_interfaces |  |
| ✅[autonomous_exploration_development_environment](https://github.com/HongbiaoZ/autonomous_exploration_development_environment) | <span style="color:red">**MODIFIED**</span> development env for cmu-series planner |
| ✅[BehaviourTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) | <span style="color:red">**MODIFIED**</span> BehaviourTree lib |
| ✅cmd_chassis | - cmd_vel to chassis_cmd for communication and motion type of the chassis <br> - exexute rotation command in chassis_link since the true value is unavailable |
| ✅control_panel | a simple Qt GUI for simulating referee system |
| ✅FAST_LIO | fastlio mapping |
| ✅livox_ros_driver2 | Driver for livox lidar |
| ✅nav2_plugins <br> - behavior_ext_plugins <br> - costmap_intensity <br> - nav2_mppi_controller_ext <br> - velocity_smoother_ext | self defined nav2 plugins <br> - an enhenced back_up action that move toward free space <br> - 2 costmap_2d layer that use intensity filed of pointcloud msg rather than height (use with terrain analysis in autonomous_exploration_development_environment) <br> - an enhenced mppi controller that is able to adjust pose before some complex terrain(use with terrain analysis-pathNorm) <br> - an enhenced velocity smoother that increase the speed on slope automatically (use with terrain analysis-pathNorm) |
| ✅rm_decision_cpp | sentry desicion module based on BehaviourTree.CPP |
| ✅sentry_description | urdf of the robot model, publish tf from sensor frame to base_link |
| rm_simulation | an rm version of autonomous_exploration_development_environment |

## Quick Start

1. prepare workspace

**FOR DOCKER USER:(HIGHLY RECOMMENDED)**

pull the image from dockerhub and run it:

```bash
docker pull polarisxq/scurm:nav_tutorial

docker run --gpus all -dit --ipc=host --net=host --privileged -e DISPLAY=host.docker.internal:0.0 -e NVIDIA_DRIVER_CAPABILITIES=all polarisxq/scurm:nav_tutorial

# use XLaunch to open the GUI
```

**BUILD FROM SOURCE:(NOTE THAT SOME FILE PATH MUST BE CHANGED)**

clone this repo to your workspace:

```bash
mkdir -p ~/nav_tutorial_ws/src
cd ~/nav_tutorial_ws/src
git clone
```

build the workspace

```bash
cd ~/nav_tutorial_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

install groot2

2. run the simulation

```bash
ros2 launch pb_rm_simulation rm_simulation.launch.py
```

3. start mapping

```bash
ros2 launch sentry_bringup mapping.launch.py 
```

4. use the keyboard to control the robot

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

5. save the map

```bash
ros2 run nav2_map_server map_saver_cli -t /projected_map -f test_map --fmt png
ros2 service call /map_save std_srvs/srv/Trigger
```

6. close all the nodes and restart the simulation

```bash
ros2 launch pb_rm_simulation rm_simulation.launch.py
```

7. start localization

```bash
ros2 launch sentry_bringup bringup_all_in_one.launch.py
```

8. start decision module
    
```bash
ros2 run control_panel control_panel
```

```bash
ros2 launch rm_decision_cpp run.launch.py
```

## Acknowledgment

rm_simulation packages is from [pb_rm_simulation](https://github.com/LihanChen2004/pb_rm_simulation), big thanks to [LihanChen2004](https://github.com/LihanChen2004)
