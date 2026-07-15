# metal-python-ros

MakerMods Metal 机械臂 ROS 2 (Humble) 工作区。原生 C++/Python（去 ROS1 依赖）驱动 + MoveIt 支持。

## 目录结构

```
metal-python-ros/
├── scripts/                    # CAN 激活脚本、控制示例脚本、install_deps.sh
├── docs/
│   └── metal_sdk_known_limitations.md   # metal_sdk 已知局限（务必先读）
└── src/
    ├── metal_sdk/               # native 库 + pybind11 绑定（非 ROS 包，需单独构建）
    ├── metal_arm_msgs/          # ROS 2 消息定义
    ├── metal_arm_description/   # URDF / meshes / rviz
    ├── metal_arm_driver/        # 驱动节点（metal_sdk 后端）
    └── metal_arm_moveit_config/ # MoveIt 配置
```

## 构建步骤（已验证的顺序，不可跳步/换序）

### 1. 安装系统依赖

```bash
bash scripts/install_deps.sh
```

该脚本需要 `sudo`，请在提示时输入密码。内容分两组：构建必需依赖（urdfdom、pybind11、nlopt、glog、eigen、can-utils、ROS2 kdl/robot_state_publisher 等）与 GUI/运行时依赖（ethtool、xacro、rviz2、moveit，用于可视化与规划，非编译强制项）。

### 2. 构建环境注意事项（重要）

如果 conda / pyenv 处于激活状态，会污染 `python3`/`pip` 指向，导致 ROS 编译失败或找不到系统包。构建前务必：

```bash
conda deactivate 2>/dev/null; export PATH=/usr/bin:$PATH; source /opt/ros/humble/setup.bash
```

确保 `python3` 解析到 `/usr/bin/python3`（系统 3.10，与 ROS Humble 一致），并且每次新开终端都要先 `source /opt/ros/humble/setup.bash`。

### 3. 构建 metal_sdk（native 库 + Python 绑定）

```bash
bash src/metal_sdk/build_metal_sdk.sh
```

该脚本会：编译 native `.so`（通过 `colcon build --packages-select metal_sdk`）→ 把 `.so` 拷到 `src/metal_sdk/metal_sdk/lib/<arch>/` → 用 `/usr/bin/python3 -m pip install --user --no-build-isolation` 安装 pybind11 绑定包 `metal_sdk`。

注意：`import metal_sdk` 需要先 `source /opt/ros/humble/setup.bash`（native 库链接了 `rclcpp`），否则会因动态库找不到而导入失败。

### 4. 构建 ROS 2 包

```bash
cd ~/makermods/metal-python-ros && colcon build
```

这一步构建 4 个 ROS 2 包：`metal_arm_msgs`、`metal_arm_description`、`metal_arm_driver`、`metal_arm_moveit_config`（`metal_sdk` 已在第 3 步单独构建过，此处会被再次触及但已是幂等的）。

### 5. 运行示例

先 `source install/setup.bash`，再任选一个 launch 文件启动驱动节点：

```bash
source ~/makermods/metal-python-ros/install/setup.bash && ros2 launch metal_arm_driver one_master.launch.py
```

可用的 launch 文件（均位于 `src/metal_arm_driver/launch/`）：

- `one_master.launch.py` — 单主臂
- `one_slave.launch.py` — 单从臂
- `one_master_slave.launch.py` — 单主 + 单从（一发一收）
- `two_master.launch.py` — 双主臂
- `two_master_slave.launch.py` — 双主 + 双从
- `single_arm_control.launch.py` — 单臂控制（含 MoveIt/控制流程）
- `two_arm_control.launch.py` — 双臂控制

CAN 网口需先激活，例如：

```bash
sudo bash scripts/start_can0.sh
```

`scripts/` 下还提供 `search.sh`（探测 USB-CAN 设备）、`set_only_one_can.sh` / `set_rules.sh`（生成/安装 udev 规则以固定设备符号链接）、以及 `joint_position_control.sh` / `end_pose_control.sh` / `go_zero_position.sh`（`ros2 topic pub` 控制示例）。

### 6. 已知局限

务必在使用前阅读 [`docs/metal_sdk_known_limitations.md`](docs/metal_sdk_known_limitations.md)，其中记录了从 SDK 源码 TODO 分流出的实质性未完成功能（碰撞检测未接入控制下发路径、速度/加加速度限制为临时值、关节软限位非无条件生效、trac_ik 静态库仅支持 x86-64 等）。

## 英文文档

See [README_EN.md](README_EN.md) for the English version.

> 前置假设：本机已安装 ROS 2 Humble desktop 及 colcon（`ros-humble-desktop` / `python3-colcon-common-extensions`）；`install_deps.sh` 只补充本项目额外依赖，不安装 ROS 本体。

> C++ 底层源码怎么用（架构/编译/API/如何修改）见 `docs/metal_sdk_cpp_source_guide.md`。
