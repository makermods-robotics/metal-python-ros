# metal_sdk C++ 源码使用说明

本文说明 `src/metal_sdk/` 下的 C++ 源码是**做什么的、怎么编、怎么被上层调用、以及怎么改**。面向需要理解或修改 Metal 臂底层驱动的开发者。

---

## 1. 两层架构总览

Metal 臂的底层能力由 C++ 实现，通过 pybind11 暴露给 Python，再被 ROS2 节点消费。数据流是一条清晰的三段链：

```
┌─────────────────────────┐   pybind11    ┌──────────────┐   import   ┌───────────────────────┐
│  native C++ 核心库        │  ───绑定──▶   │  metal_sdk    │  ───────▶  │  metal_arm_driver 节点  │
│  libmetal_sdk_<arch>.so  │   wrapper.cpp │  (python 包)  │            │  (rclpy 控制节点)       │
│  CAN / 运动学 / 轨迹      │               │  .so 扩展模块 │            │  话题/服务 ⇄ SDK 调用   │
└─────────────────────────┘               └──────────────┘            └───────────────────────┘
        C++（本文重点）                      C++↔Python 桥               Python / ROS
```

- **native 层**：纯 C++（CAN 通信、KDL 运动学、轨迹插值、电机读写）。编译产物是一个架构相关的动态库 `libmetal_sdk_x64.so` / `libmetal_sdk_arm64.so`。
- **binding 层**：一个 pybind11 wrapper，把 native 层的 `MetalSDKInterface` 类映射成 Python 可调用对象，编译成 `metal_sdk/metal_sdk.cpython-310-*.so`。
- **消费层**：`metal_arm_driver` 节点 `from metal_sdk import MetalSDKInterface, ControlMode`，把 ROS 话题转成 SDK 调用。

> 关键点：native 层是**厂商一方源码**（原命名空间 `makermods::metal`），本工程从 `y1_sdk_ubuntu22_04` vendored 进来并做了**表面级清理（仅改名/注释，未改逻辑）**。它不是第三方库，可放手维护。

---

## 2. 目录结构

```
src/metal_sdk/
├── native/                     # ① C++ 核心库源码（ament_cmake 包，名为 metal_sdk）
│   ├── CMakeLists.txt          #   构建脚本，含 x64/arm64 架构判断
│   ├── package.xml             #   ROS2 包清单
│   ├── metal_sdk_interface.h   #   ★ 对外唯一头文件（公开 API，见 §5）
│   ├── metal_sdk_interface.cpp #   接口实现（PImpl 转发）
│   ├── can_manager.{h,cpp}     #   CAN 总线收发、控制循环调度
│   ├── kinodynamic/            #   运动学/动力学
│   │   ├── kdl_solver.*        #     KDL 正/逆运动学、雅可比、重力/科氏项
│   │   ├── payload_estimator/  #     负载估计
│   │   └── cartesian_impedance_controller/  # 笛卡尔阻抗控制（当前未编入源列表）
│   ├── trajectory/             #   多项式插值、时间分配
│   ├── motor_readers/          #   达妙(DM)电机状态解析
│   ├── motor_writers/          #   达妙电机指令下发
│   ├── motor_interface_base/   #   读写抽象基类
│   ├── config/                 #   电机配置
│   ├── common/                 #   日志(log.h)、电机状态、时间工具
│   └── third_lib/trac_ik_lib/  #   trac_ik 逆解静态库（vendored，⚠ 仅 x64，见 §8）
│
├── binding/                    # ② pybind11 绑定层
│   ├── wrapper.cpp             #   ★ PYBIND11_MODULE，把 MetalSDKInterface 映射到 Python
│   ├── setup.py                #   编绑定 .so，链接 ① 的产物（架构自适应）
│   ├── pyproject.toml
│   └── COLCON_IGNORE           #   让 colcon 忽略此目录（避免与 native 撞名）
│
├── metal_sdk/                  # ③ Python 包（安装后可 import metal_sdk）
│   ├── __init__.py             #   导出 MetalSDKInterface, ControlMode
│   ├── lib/{x64,arm64}/        #   native .so 落点（.gitignore 忽略，由构建脚本填充）
│   └── COLCON_IGNORE
│
└── build_metal_sdk.sh          # ★ 一键构建：编 native → 拷 .so → pip 装绑定
```

**为什么 native 和 binding 同名 `metal_sdk` 却不冲突**：native 是 colcon/ament 包，binding+python 包走 pip 安装，分属两套构建体系；两个 `COLCON_IGNORE` 保证 `colcon build` 只认 native 那一个包。

---

## 3. native 各模块职责

| 模块 | 职责 |
|------|------|
| `metal_sdk_interface.*` | 对外门面（Facade），PImpl 模式隐藏实现，唯一对外头文件 |
| `can_manager.*` | 打开 SocketCAN、收发帧、按控制模式跑实时/非实时控制循环 |
| `kinodynamic/kdl_solver.*` | 由 URDF 建 KDL 链，算正解(FK)/逆解(IK)、雅可比、重力补偿、科氏项 |
| `kinodynamic/payload_estimator/` | 末端负载估计（存在已知局限，见 known-limitations 文档） |
| `trajectory/` | 关节空间多项式插值 + 轨迹时间分配（NRT 模式用） |
| `motor_readers/` `motor_writers/` | 达妙(DM)电机 CAN 协议的解析与下发 |
| `motor_interface_base/` | 读写器抽象基类 |
| `config/` | 各关节电机参数配置 |
| `common/` | `log.h`（glog 封装，MODULE_NAME="metal_arm"）、电机状态结构、时间 |
| `third_lib/trac_ik_lib/` | TRAC-IK 逆解静态库（`.a`，静态链接进 .so） |

**依赖**：native 链接 ROS 库（rclcpp、sensor_msgs、kdl_parser、orocos_kdl、urdf/urdfdom）+ 系统库（nlopt、glog、Eigen3）+ trac_ik 静态库。因此编译与运行都需先 `source /opt/ros/humble/setup.bash`。

---

## 4. 怎么编（构建流程）

**一条命令**（x64 和 Jetson arm64 通用）：

```
bash src/metal_sdk/build_metal_sdk.sh
```

它内部做三步：

1. `colcon build --packages-select metal_sdk` —— 编 native，`CMakeLists.txt` 按 `CMAKE_SYSTEM_PROCESSOR` 自动产出 `lib/x64/libmetal_sdk_x64.so` 或 `lib/arm64/libmetal_sdk_arm64.so`。
2. 把该 `.so` 拷进 `src/metal_sdk/metal_sdk/lib/<arch>/`。
3. `/usr/bin/python3 -m pip install --user --no-build-isolation ./binding` —— 编 pybind11 绑定并链接第 1 步的 native 库。

**环境陷阱（本机务必）**：conda base 会把 `python3` 遮蔽成 3.13，而 ROS 用 `/usr/bin/python3`(3.10)。构建前先：

```
conda deactivate 2>/dev/null; export PATH=/usr/bin:$PATH; source /opt/ros/humble/setup.bash
```

> 改了 native 源码后必须**重跑 `build_metal_sdk.sh`**（重编 native + 重装绑定），否则 Python 侧还是旧 .so。

---

## 5. C++ 公开 API（`MetalSDKInterface`）

命名空间 `makermods::metal`。这是唯一对外类，所有能力都经它暴露：

**构造 / 生命周期**
```cpp
MetalSDKInterface(const std::string& can_id,     // CAN 接口名，如 "can0"
                  const std::string& urdf_path,  // URDF 路径（建 KDL 链用）
                  int arm_end_type,              // 末端类型（无夹爪/有夹爪）
                  bool enable_arm);              // 构造时是否使能电机
bool Init();                                     // 必须调用，返回是否初始化成功
```

**控制模式**
```cpp
enum ControlMode { GRAVITY_COMPENSATION=0, RT_JOINT_POSITION=1, NRT_JOINT_POSITION=2 };
void SetArmControlMode(const ControlMode& mode);
```

**下发指令**
```cpp
void SetArmJointPosition(const std::array<double,6>& q, int velocity_ratio=5); // 6 关节，速度档 1-10
void SetArmJointPosition(const std::vector<double>& q);                        // 含夹爪的跟随控制
void SetArmEndPose(const std::array<double,6>& xyzrpy);                        // 末端位姿 x y z r p y
void SetGripperStroke(double stroke_mm, int velocity_ratio=5);                 // 夹爪行程 [0,80]mm
void SetEnableArm(bool enable_flag);                                          // 使能/失能全部电机
void SaveJ6ZeroPosition();                                                     // 重装末端后标定 J6 零点
```

**读取状态**
```cpp
std::vector<std::string> GetJointNames();       // 6 或 7(含夹爪) 个关节名
std::vector<double> GetJointPosition();         // 关节位置
std::vector<double> GetJointVelocity();         // 关节速度
std::vector<double> GetJointEffort();           // 关节力矩
std::array<double,6> GetArmEndPose();           // 末端位姿 x y z r p y
std::vector<double> GetRotorTemperature();      // 转子温度
std::vector<double> GetMotorCurrent();          // 电机电流
std::vector<int>    GetJointErrorCode();        // 错误码 0-8（0禁用/1使能/2掉线/3过压/4欠压/5过流/6MOS过温/7转子过温/8过载）
```

---

## 6. pybind11 绑定层（`binding/wrapper.cpp`）

wrapper.cpp 用 `PYBIND11_MODULE(metal_sdk, m)` 把上面的类逐方法映射到 Python：

```cpp
py::enum_<MetalSDKInterface::ControlMode>(m, "ControlMode")
    .value("GRAVITY_COMPENSATION", ...).value("RT_JOINT_POSITION", ...)...export_values();
py::class_<MetalSDKInterface>(m, "MetalSDKInterface")
    .def(py::init<const std::string&, const std::string&, int, bool>(), ...)
    .def("Init", &MetalSDKInterface::Init)
    .def("SetArmControlMode", &MetalSDKInterface::SetArmControlMode)
    .def("SetArmJointPosition", ...) ... ;
```

它 `#include "metal_sdk_interface.h"`（`native/` 在 include 路径上），只依赖公开头文件，不重编 native——绑定 .so 在链接期挂到 native .so 上（`$ORIGIN/lib/<arch>` 运行时查找）。

---

## 7. Python / ROS 侧怎么用

安装后（`build_metal_sdk.sh` 完成，且 `source /opt/ros/humble/setup.bash`——native .so 依赖 librclcpp）：

```python
from metal_sdk import MetalSDKInterface, ControlMode

arm = MetalSDKInterface(can_id="can0", urdf_path="/path/metal_with_gripper.urdf",
                        arm_end_type=1, enable_arm=True)
arm.Init()
arm.SetArmControlMode(ControlMode.RT_JOINT_POSITION)
arm.SetArmJointPosition([0, 0, 0, 0, 0, 0], velocity_ratio=5)
print(arm.GetJointPosition(), arm.GetArmEndPose())
```

`metal_arm_driver` 节点（`src/metal_arm_driver/metal_arm_driver/metal_arm_driver_single_node.py`）正是这样封装：订阅 `ArmJointPositionControl`/`ArmEndPoseControl` 话题 → 调 `SetArmJointPosition`/`SetArmEndPose`；定时读 `GetJointPosition/...` → 发布 `ArmJointState`/`ArmStatus`。启动：`ros2 launch metal_arm_driver <launch>`。

---

## 8. 跨架构说明

- native 的 `CMakeLists.txt` 与 binding 的 `setup.py` 都内置架构判断，x64/arm64 各自产出对应命名的库，源码只有一份。
- **⚠ Jetson arm64 阻塞项**：`third_lib/trac_ik_lib/lib/libtrac_ik_lib.a` **仅 x86-64**，arm64 链接会失败，需先取得/编译 arm64 版 trac_ik。详见 `docs/metal_sdk_known_limitations.md`。

---

## 9. 怎么修改 / 扩展 C++

1. 改 `native/` 下源码（清理约定：仅在需要时动逻辑，注释用 Doxygen `/** */`）。
2. **若改动了公开 API**（`metal_sdk_interface.h` 加/改方法）：同步在 `binding/wrapper.cpp` 补一条 `.def("Xxx", &MetalSDKInterface::Xxx)`，否则 Python 侧看不到。
3. 若新增 `.cpp` 文件：加进 `native/CMakeLists.txt` 的 `METAL_SDK_SRCS` 列表。
4. 重跑 `bash src/metal_sdk/build_metal_sdk.sh` 重编重装。
5. 验证：`source /opt/ros/humble/setup.bash && /usr/bin/python3 -c "from metal_sdk import MetalSDKInterface, ControlMode; print('ok')"`。

---

## 10. 已知局限

底层未完成/待优化项（碰撞检测未接线、速度/加加速度限制待定、负载估计优化、trac_ik 跨架构等）统一记录在 **`docs/metal_sdk_known_limitations.md`**，修改前请先查阅。
