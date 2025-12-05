# Diana7-ROS2

ROS 2 驱动程序，用于控制 **Diana 7** 七轴协作机械臂，集成 ros2_control 和 MoveIt2 运动规划。

## 效果展示

![MoveIt2 + RViz 可视化界面](Moveit-Rviz.png)

## 项目结构

```
Diana7-ROS2/
├── diana7_description/    # 机器人URDF描述和3D模型
├── diana7_hardware/       # ros2_control 硬件接口
├── diana7_bringup/        # 启动文件配置
└── diana7_moveit_config/  # MoveIt2 运动规划配置
```

## 功能特性

- 7自由度协作机械臂完整驱动
- 基于 ros2_control 的硬件抽象层
- 集成 MoveIt2 运动规划框架
- 支持位置控制模式
- 自动抱闸管理

## 依赖项

### 系统要求

- Ubuntu 22.04
- ROS 2 Humble

### ROS 2 依赖

```bash
sudo apt install \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-moveit \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-controller-manager
```

## 安装

1. 创建工作空间并克隆仓库：

```bash
mkdir -p ~/diana7_ws/src
cd ~/diana7_ws/src
git clone <repository_url> Diana7-ROS2
```

2. 编译工作空间：

```bash
cd ~/diana7_ws
colcon build --symlink-install
source install/setup.bash
```

## 使用方法

### 1. 启动真实硬件

连接机械臂并启动硬件驱动：

```bash
ros2 launch diana7_bringup hardware_real.launch.py ip:=192.168.10.75
```

**参数说明：**

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `ip` | `192.168.10.75` | 机械臂 IP 地址 |
| `timeout_ms` | `100` | 连接超时时间（毫秒）|

### 2. 启动 MoveIt2 运动规划

在另一个终端中启动 MoveIt2（用于可视化和运动规划）：

```bash
ros2 launch diana7_moveit_config demo.launch.py
```

**可选参数：**

| 参数 | 默认值 | 描述 |
|------|--------|------|
| `use_rviz` | `true` | 是否启动 RViz 可视化 |
| `db` | `false` | 是否启动数据库 |

### 3. 使用流程

典型的使用流程：

```bash
# 终端 1：启动硬件驱动
ros2 launch diana7_bringup hardware_real.launch.py ip:=192.168.10.75

# 终端 2：启动 MoveIt2（等待终端1完成初始化后）
ros2 launch diana7_moveit_config demo.launch.py
```

## 包说明

### diana7_description

机器人描述包，包含：
- URDF/xacro 模型文件
- 3D 网格模型（STL 格式）
- DH 参数配置
- ros2_control 硬件配置

### diana7_hardware

ros2_control 硬件接口，提供：
- `Diana7System` 硬件接口插件
- 与 Diana API 的通信实现
- 关节位置/速度状态读取
- 关节位置命令发送
- 抱闸控制

### diana7_bringup

启动配置包，包含：
- `hardware_real.launch.py` - 真实硬件启动
- `hardware_sim.launch.py` - 仿真启动
- 控制器配置文件

### diana7_moveit_config

MoveIt2 配置包（由 MoveIt Setup Assistant 生成），包含：
- SRDF 语义描述
- 运动学求解器配置
- 运动规划器配置
- 控制器配置
- RViz 配置

## 注意事项

1. **网络配置**：确保计算机与机械臂在同一网段，且能够 ping 通机械臂 IP
2. **急停**：操作前确保急停按钮可用
3. **启动顺序**：先启动硬件驱动，待连接成功后再启动 MoveIt2
4. **抱闸**：硬件接口激活时自动释放抱闸，停用时自动抱闸

## 故障排除

### 无法连接机械臂

- 检查网络连接和 IP 配置
- 确认机械臂已上电
- 检查防火墙设置

### MoveIt 规划失败

- 检查当前位姿是否在关节限位内
- 确认硬件驱动正常运行
- 检查 `joint_state_broadcaster` 是否正常发布状态

## License

BSD

## 作者

zym (yeeman@buaa.edu.cn)
