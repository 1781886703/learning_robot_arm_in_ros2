# README

# ROS2 机械臂仿真项目实战笔记

这是一个集成了 URDF、RViz2、ROS2_Control、MoveIt2 和 Gazebo 的综合性 ROS2 机械臂仿真项目。本项目记录了从零开始搭建一个可仿真、可控制的机械臂的全过程，旨在为学习者提供清晰的实战指导和思路整理。



## 目录

- **项目简介**
- **核心技术栈**
- **项目结构**
- **快速开始**
  - 环境要求
  - 安装依赖
  - 项目编译
- **主要功能与工作流**
  - 工作流一：从零手搓可动的机械臂（ROS2_Control）
  - 工作流二：使用 MoveIt2 配置助手快速实现运动规划
  - 工作流三：实现 Gazebo 与 MoveIt2/ROS2_Control 的联动仿真
  - 工作流四：使用 MoveIt2 Python API 进行编程控制
- **未来展望**
- **已知问题与解决方案**
- **致谢**

## 项目简介

本项目旨在通过一系列实际操作，深入理解和掌握 ROS2 中机械臂仿真的核心组件。它涵盖了从最基础的 **URDF** 模型构建到复杂的 **MoveIt2** 运动规划，以及 **Gazebo** 物理引擎仿真的全链条。通过该项目，您可以：

- 学会如何使用 **URDF**（或 Xacro）文件定义机械臂的结构和物理属性。
- 掌握 **ROS2_Control** 库，用于实现机械臂关节的底层控制。
- 利用 **MoveIt2** 的强大功能，快速配置机械臂的运动规划、碰撞检测和可视化。
- 将上述配置无缝迁移到 **Gazebo** 物理仿真环境中，实现更真实的机器人行为模拟。
- 通过 **MoveIt2 Python API**，使用代码而不是 GUI 来控制机械臂，为自动化任务打下基础。

## 核心技术栈

| 组件             | 描述                                                         |
| ---------------- | ------------------------------------------------------------ |
| **URDF**         | 机器人描述文件，用于定义机械臂的连杆（link）和关节（joint）结构。 |
| **ROS2_Control** | 机器人控制库，提供控制器管理器、控制器和硬件接口，是连接高层规划与底层执行的关键。 |
| **MoveIt2**      | 强大的运动规划框架，集成了多种规划器、碰撞检测和逆运动学求解器，是高层控制的核心。 |
| **Gazebo**       | 开源的机器人物理仿真平台，支持物理引擎，用于模拟真实的机器人行为和环境交互。 |
| **RViz2**        | ROS2 的三维可视化工具，用于显示机器人模型、传感器数据、规划轨迹等。 |

## 项目结构

```
ARM_WS/
├── build/
├── install/
├── log/
└── src/
    ├── arm1/                      ---->从0手搓可以动的机械臂 功能包
    │   ├── __pycache__/
    │   ├── __init__.py
    │   ├── test_arm_topic_control.py    ---> 使用话题通讯控制arm规划组
    │   ├── test_hand_control.py		---> 使用话题通讯控制hand规划组
    │   ├── config/
    │   │   ├── arm_controllers.yaml	---> ros2_control控制器配置文件
    │   │   └── rviz.rviz				---> rviz2页面配置文件
    │   ├── launch/
    │   │   ├── arm_ros2_control_urdf_launch.py  ---> 在rviz2中打开带有ros2_control控制器的机械臂，并打开手臂控制节点
    │   │   ├── arm_urdf_launch.py               ---> 在rviz2中打开原始的机械臂模型
    │   │   └── hand_control_launch.py			---> 在rviz2中打开带有ros2_control控制器的机械手，并打开夹爪控制节点
    │   ├── resource/
    │   ├── test/
    │   ├── urdf/
    │   │   ├── arm_ros2_control.urdf         ---> 带有ros2_control控制器的机械臂模型
    │   │   ├── arm.urdf					  ---> 原始的机械臂模型
    │   │   └── robot.urdf					  ---> 简单的urdf模型示例
    │   ├── LICENSE
    │   ├── package.xml
    │   ├── setup.cfg
    │   └── setup.py						 ---> 编译配置文件
    ├── moveit_config/            ----> moveit2搓出可以动的机械臂(由moveit配置助手得到的文件)
    │   ├── config/
    │   │   ├── initial_positions.yaml
    │   │   ├── joint_limits.yaml         --->关节限位，需要将整数改为浮点数、并把a、v、p限位改为true！！！
    │   │   ├── kinematics.yaml
    │   │   ├── moveit_controllers.yaml
    │   │   ├── moveit.rviz
    │   │   ├── pilz_cartesian_limits.yaml
    │   │   ├── robot_name.gazebo.ros2_control.xacro
    │   │   ├── robot_name.ros2_control.xacro   ----> moveit2生成的带ros2_control标签的模型
    │   │   ├── robot_name.srdf					---->机械臂描述文件
    │   │   ├── robot_name.urdf.xacro			----> moveit2生成的原始模型
    │   │   └── ros2_controllers.yaml			---> ros2_control控制器配置文件
    │   ├── launch/
    │   │   ├── demo.launch.py                 ---->配置完moveit后就可以跑的机械臂（集成了下面其他的launch）
    │   │   ├── gazebo.launch.py			  ----->自己编写改造的，用于打开gazebo中跑机械臂模型
    │   │   ├── move_group.launch.py
    │   │   ├── moveit_rviz.launch.py
    │   │   ├── rsp.launch.py
    │   │   ├── setup_assistant.launch.py
    │   │   ├── spawn_controllers.launch.py
    │   │   ├── static_virtual_joint_tfs.launch.py
    │   │   └── warehouse_db.launch.py
    │   ├── .setup_assistant
    │   ├── CMakeLists.txt
    │   └── package.xml
    └── moveit_motion_api/      ----> 使用moveit2的python编程接口完成控制
        ├── config/
        │   └── moveit_cpp.yaml  ---> 使用编程接口前所需的配置文件（基本是固定模板）
        ├── moveit_motion_api/
        │   ├── __pycache__/
        │   ├── __init__.py
        │   └── test1.py        ---> 使用moveit接口编程实现对机械臂的控制（基本是固定模板）
        ├── resource/
        ├── test/
        ├── LICENSE
        ├── package.xml
        ├── setup.cfg
        └── setup.py          ---> 节点编译配置
```

## 快速开始

### 环境要求

- **操作系统：** Ubuntu 24.04 LTS (或更高版本)
- **ROS2 版本：** Jazzy 
- **依赖库：** `moveit2`, `rviz2`, `gazebo` (默认最新版), `joint_state_publisher_gui`, `ros2_control`, `xacro` 等。

### 安装依赖

在您的 ROS2 工作空间中，克隆本项目并安装所有必要的依赖：

```
cd ~/ARM_WS/src
git clone https://github.com/1781886703/learning_robot_arm_in_ros2
```

### 项目编译

使用 `colcon build` 命令编译项目：

```
colcon build --symlink-install 
source install/setup.bash
```

## 主要功能与工作流

### 工作流一：从零手搓可动的机械臂（ROS2_Control）

该工作流不依赖 MoveIt2，通过手动配置 **`arm1`** 功能包，让您深入理解 ROS2_Control 的工作原理。

- **URDF 建模：** 在 `arm1/urdf` 目录下创建基本的 URDF 模型。
- **ROS2_Control 配置：** 在 `arm1/config` 目录下编写 `arm_controllers.yaml` 文件，定义控制器。
- **URDF 插件：** 在 URDF 文件中添加 `<ros2_control>` 标签，引用硬件接口和控制器。
- **Launch 启动：** 运行 `arm_ros2_control_urdf_launch.py`，启动 `robot_state_publisher`、`controller_manager`、`spawner` 和 RViz2，并通过自定义 Python 节点控制机械臂。

### 工作流二：使用 MoveIt2 配置助手快速实现运动规划

利用 MoveIt2 配置助手，可以极大地简化机械臂配置过程。

- **启动配置助手：** 运行 `ros2 launch moveit_setup_assistant setup_assistant.launch.py`。
- **配置步骤：**
  1. 导入原始 URDF 模型。
  2. 添加虚拟关节（推荐）。
  3. 定义规划组（`arm` 和 `hand`）。
  4. 添加末端执行器。
  5. 定义预设姿态（如 `ready`、`open`、`close`）。
  6. 生成 MoveIt2 配置文件。
- **运行：** 运行 `demo.launch.py` 即可在 RViz2 中通过 MoveIt2 插件控制机械臂。

### 工作流三：实现 Gazebo 与 MoveIt2/ROS2_Control 的联动仿真

将机械臂模型导入 Gazebo 仿真环境，实现更真实的物理交互。

- **修改 URDF/Xacro：** 在 MoveIt2 生成的 Xacro 文件中，将 `hardware_interface` 插件从 `mock_components/GenericSystem` 更改为 **`gz_ros2_control/GazeboSimSystem`**。
- **修改 Launch 文件：** 在 `gazebo.launch.py` 中，添加 Gazebo 启动节点、ROS-Gazebo 时钟桥接节点和 Gazebo 模型加载节点。
- **运行：** 启动修改后的 `gazebo.launch.py` 文件，即可在 Gazebo 中看到机械臂模型，并在 RViz2 中进行控制。

### 工作流四：使用 MoveIt2 Python API 进行编程控制

通过编写 Python 代码，实现对机械臂的自动化控制。

- **配置文件：** 在 `moveit_motion_api/config` 中创建 `moveit_cpp.yaml`，声明规划器和参数。
- **编写代码：** 在 `test1.py` 中，使用 `MoveItPy` 库实例化机器人，调用 `set_goal_state` 和 `plan_and_execute` 等函数，实现关节角、笛卡尔坐标等多种方式的运动控制。

## 未来展望

- **动作通讯：** 完善基于 ROS2 Action 的机械臂控制方式。
- **实物控制：** 编写自定义 `hardware_interface`，将该项目扩展到实物机械臂控制。
- **算法集成：** 将强化学习等高级控制算法集成到项目中，探索更智能的机器人行为。
- **更多控制器：** 尝试编写自定义控制器，以实现更复杂的控制策略。

## 已知问题与解决方案

- **`joint_limit.yaml` 错误：** MoveIt2 配置助手在某些版本中生成的 `joint_limit.yaml` 文件可能存在 bug。需要手动将整数值改为浮点数，并将 `has_velocity_limits`, `has_acceleration_limits`, `has_position_limits` 全部设置为 `true`。
- **Gazebo 黑屏/白屏：** 在虚拟机中使用 Gazebo 时，可能需要禁用 3D 渲染加速。
- **URDF/Xacro 文件路径：** 在 Launch 文件中引用 URDF/Xacro 路径时，确保在 `setup.py` 的 `data_files` 中声明。

## 致谢

该项目笔记基于 **Bilibili UP 主“后来老师”和“路一直都在456”** 共同创作的机械臂仿真实战课程。在此向两位老师的无私视频分享表示衷心感谢。

**B站课程链接：** https://www.bilibili.com/video/BV1SBunzJEdE

再次感谢 **“后来老师”** 和 **“路一直都在456”** 两位 Bilibili UP 主的视频教学，他们的课程为本项目的学习和实践提供了宝贵的指导，如需更深入的学习，请去b站联系他们！

如果您对项目代码或内容有任何疑问或建议，欢迎提交 Issues 或 Pull Requests。