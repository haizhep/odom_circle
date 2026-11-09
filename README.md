# odom_circle

ROS 2 Humble 节点：发布一个做**正逆时针交替的匀速圆周运动**的里程计，并在 RViz 中可视化。

## 功能

- 以 20 Hz 向 `/odom` 发布 `nav_msgs/msg/Odometry`
- 同时广播 TF 变换 `odom -> base_link`
- 机器人沿半径 `radius` 的圆做匀速运动，每走完一圈自动反向
- 朝向始终沿圆周切线方向

## 设计思路

### 1. 位姿生成

用角度 θ 作为唯一状态量，每帧按实际经过时间 dt 积分：

```
θ ← θ + ω·dt
x = R·cos(θ)
y = R·sin(θ)
yaw = θ + π/2   (ω > 0，逆时针)
yaw = θ − π/2   (ω < 0，顺时针)
```

### 2. 正逆时针交替

θ 被限制在 [0, 2π]：

- θ 到达 2π（逆时针走完一圈）时，把 ω 取反
- θ 降到 0（顺时针走完一圈）时，再把 ω 取反

轨迹始终是同一个圆，运动方向来回交替。

### 3. 速度

- 线速度 `v = |ω|·R`，大小恒定，因此是匀速圆周运动
- 角速度 `twist.twist.angular.z = ω`，符号表示转向

### 4. 朝向与四元数

平面运动只有一个 yaw，转成四元数的公式：

```
x = 0, y = 0, z = sin(yaw/2), w = cos(yaw/2)
```

## 参数

| 参数 | 默认值 | 说明 |
|---|---|---|
| `radius` | 1.0 | 圆周半径（米） |
| `angular_speed` | 0.5 | 角速度（弧度/秒），一圈约 12.6 秒 |

## 编译与运行

```bash
cd ~/ros2_ws
colcon build --parallel-workers 2
source install/setup.bash

# 方式一：节点 + rviz 一起启动
ros2 launch odom_circle odom_circle.launch.py

# 方式二：只启动节点（自行打开 rviz2）
ros2 run odom_circle odom_publisher

# 自定义参数
ros2 run odom_circle odom_publisher --ros-args -p radius:=2.0 -p angular_speed:=1.0
```

## RViz 配置

- Fixed Frame: `odom`
- Add → By topic → `/odom` → Odometry

## 验证

```bash
# 查看里程计消息
ros2 topic echo /odom

# 查看坐标变换
ros2 run tf2_ros tf2_echo odom base_link
```

## 目录结构

```
odom_circle/
├── src/odom_publisher.cpp
├── launch/odom_circle.launch.py
├── rviz/odom_circle.rviz
├── CMakeLists.txt
├── package.xml
└── README.md
```
