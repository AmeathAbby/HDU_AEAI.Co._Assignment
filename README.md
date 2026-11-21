系统架构

```mermaid
flowchart LR
    Y[trajectory.yaml] --> C[ctrl / trajectory_follower]
    C -->|/mavros/setpoint_position/local| M[MAVROS]
    M -->|/mavros/state · /mavros/local_position/pose| C
    C -->|/mavros/set_mode · /mavros/cmd/arming| M
    M <-->|MAVLink UDP 14540 ↔ 14580| P[PX4 SITL]
    P <-->|模拟传感器 · 电机输出 TCP 4560| G[Gazebo Classic · BaseMap · Iris]
    G -->|/camera/rgb/image_raw| V[cv_bridge → OpenCV HSV]
    G -->|/camera/depth/points| S[camera_stream_guard]
    G -->|/clock| R[ROS 仿真时钟]
    V -->|/vision/pink_detected · Bool| B[订阅者]
    V -->|/vision/debug_image · Image| I[图像查看器]
    P <-->|MAVLink UDP 14550| Q[可选 QGroundControl]
    C -->|/ctrl/aircraft_markers · /ctrl/flight_path| VZ[RViz]
    M -->|/mavros/local_position/pose| VZ
```

目录结构

```text
project1/
├── README.md
├── src/
│   ├── CMakeLists.txt                          Catkin 工作空间入口
│   ├── ctrl/
│   │   ├── CMakeLists.txt / package.xml
│   │   ├── config/trajectory.yaml              航点、频率/超时与安全边界
│   │   ├── config/trajectory.rviz              RViz 显示配置
│   │   ├── launch/trajectory.launch            单独启动航线节点
│   │   ├── launch/visualization.launch         可视化节点 + RViz
│   │   ├── scripts/trajectory_follower.py      20 Hz OFFBOARD 位置控制
│   │   └── scripts/flight_visualizer.py        机体/轨迹 RViz 可视化
│   └── drone_vision/
│       ├── CMakeLists.txt / package.xml
│       ├── config/pink.yaml                    浅粉色 HSV 阈值（BaseMap）
│       ├── config/red.yaml                     红色 HSV 阈值（color_test）
│       ├── launch/basemap_mission.launch       一键入口：仿真 + 视觉 + 航线
│       ├── launch/sitl_rgbd.launch             Gazebo + PX4 + MAVROS + RGB-D
│       ├── launch/color_detection.launch       颜色检测节点入口
│       ├── models/basemap/                     BaseMap 几何、材质、贴图（含粉色立柱）
│       ├── models/iris_rgbd/                   带前向 RGB-D 相机的 Iris
│       ├── worlds/basemap.world                基础场景
│       ├── worlds/color_test.world             颜色检测测试场景
│       ├── scripts/color_detector.py           检测节点
│       ├── scripts/camera_stream_guard.py      点云保活
│       └── scripts/start_px4.sh                启动已编译的 PX4 SITL
└── vendor/PX4-Autopilot/                       PX4 v1.14.4（不随仓库提交，需自行克隆）
```

> 仓库不包含 `vendor/PX4-Autopilot`、`build/`、`devel/`。clone 后必须按“编译”一节的步骤克隆 PX4 并重新编译，不能直接运行。

主要话题、服务与参数

| 名称 | 类型 | 说明 |
|---|---|---|
| `/mavros/state` | `mavros_msgs/State` | 飞控连接、模式、解锁状态 |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | 本地 ENU 位姿 |
| `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | 位置设定值，20 Hz |
| `/mavros/set_mode` | `mavros_msgs/SetMode` | 请求 OFFBOARD |
| `/mavros/cmd/arming` | `mavros_msgs/CommandBool` | 请求解锁 |
| `/camera/rgb/image_raw` | `sensor_msgs/Image` | 640×480 RGB，约 10 Hz |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 深度图 |
| `/camera/depth/points` | `sensor_msgs/PointCloud2` | 深度点云 |
| `/vision/pink_detected` | `std_msgs/Bool` | 颜色检测结果 |
| `/vision/target_x` | `std_msgs/Float32` | 最大粉色目标的归一化水平偏移，0 为画面中心 |
| `/vision/debug_image` | `sensor_msgs/Image` | 带框调试图（有订阅者时发布） |
| `/ctrl/aircraft_markers` | `visualization_msgs/MarkerArray` | RViz 四旋翼模型 |
| `/ctrl/flight_path` | `nav_msgs/Path` | RViz 飞行轨迹 |
| `/clock` | `rosgraph_msgs/Clock` | Gazebo 仿真时间 |

编译：

仓库不提交 `build/`、`devel/`、`install/`。这些目录里的 `CMakeCache.txt`、`devel/setup.*` 写死了当前机器的绝对路径（例如 `/home/ameath/桌面/Project/project1`）。**换机器或换目录 clone 后不能直接 `source devel/setup.bash`，必须重新 `catkin_make`


编译 Catkin 工作空间

```bash
cd ~/project1
source /opt/ros/noetic/setup.bash
catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
source devel/setup.bash
```

验证包能被找到：

```bash
rospack find ctrl
rospack find drone_vision
```

编译 PX4 与 Gazebo Classic 插件

**本仓库不包含 `vendor/PX4-Autopilot`**（PX4 源码约 1.8 GB，单独下载）。克隆项目后必须先把它放到 `vendor/PX4-Autopilot`：

```bash
cd ~/project1
mkdir -p vendor
git clone -b v1.14.4 --recursive \
  https://github.com/PX4/PX4-Autopilot.git vendor/PX4-Autopilot
source /opt/ros/noetic/setup.bash
cd ~/project1/vendor/PX4-Autopilot
git config --global --add safe.directory '*'   # Docker/root 构建、仓库属于宿主用户时需要
DONT_RUN=1 make px4_sitl gazebo-classic
```

如果 PX4 放在其他位置，启动前设置：

```bash
export PX4_SITL_ROOT=/path/to/PX4-Autopilot
```
回到工作空间加载环境

```bash
cd ~/project1
source devel/setup.bash
```
之后才能执行下面的 `roslaunch`。



快速开始

一键启动： GazeboUI 定点飞行 视觉监测

```bash
cd ~/project1
source devel/setup.bash
roslaunch drone_vision basemap_mission.launch enable_control:=true
```

飞行模式控制

```bash
# 起飞2 m 悬停 5 秒 飞三个航点
roslaunch drone_vision basemap_mission.launch enable_control:=true

# 正方形：边长自定义（这个地图用的是自己建模的26浙江工创赛地图，可以提供BLENDER工程文件，因为地图的障碍物和外墙高度是3m，为了能保证正常飞行把飞行高度升到3.3m）
roslaunch drone_vision basemap_mission.launch enable_control:=true \
  mission:=square square_side:=1.40 #这里传入边长参数

# 越障：飞越粉色立柱
roslaunch drone_vision basemap_mission.launch enable_control:=true mission:=avoid
```
