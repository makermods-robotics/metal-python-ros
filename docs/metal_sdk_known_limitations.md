# metal_sdk 已知局限（Known Limitations）

> 从 SDK 源码 TODO 分流而来的实质性未完成功能。表面级重构未实现这些，仅记录。

## 安全相关（优先）
- **碰撞检测未实现**：`can_manager.cpp` 指令下发前未检查碰撞（原 TODO：发生碰撞则保持当前位置）。`IsCollisionDetected()` / `KeepCurrentPostion()` 已有实现，但当前未被控制下发路径调用。
- **速度/加加速度限制待定**：`can_manager.cpp` 速度条件为临时值，缺 max vel/acc/jerk 限制。
- **关节软限位未实现**：`dm_motor_writer.cpp`（原 TODO: joint soft limit）。仅在调用方显式传入 `need_position_limit=true` 时才生效，并非无条件强制。

## 算法/优化
- **负载估计**：`payload_estimator.cpp` 前馈力矩存在重复计算，雅可比负载估计法待优化。
- **Coriolis 计算**：`kdl_solver.cpp` 需确认 KDL `JntToCoriolis` 是否已内含与 q_ddot 相乘。

## 待补控制模式
- `can_manager.cpp`：other control mode 未覆盖。

## 跨架构 (Cross-arch)
- **trac_ik 静态库仅支持 x86-64**：`src/metal_sdk/native/third_lib/trac_ik_lib/lib/libtrac_ik_lib.a` 是 x86-64-only 的预编译静态库，不含 arm64 切片。在 Jetson（arm64）上构建 `metal_sdk` 会在链接阶段失败，除非获得该库的 arm64 构建版本。此结论已在 Task 2 中通过 `file`/`lipo` 检查确认。

## Jetson arm64 验收（待办）

x64 已全量验收通过（干净构建 5 包 + import/接口/节点冒烟）。Jetson arm64 尚未验收，需在 Jetson 上：

1. `bash scripts/install_deps.sh` 装依赖；`bash src/metal_sdk/build_metal_sdk.sh` 现编原生库（产出 `libmetal_sdk_arm64.so`）+ 绑定；`colcon build`。
2. **阻塞项**：vendored `third_lib/trac_ik_lib/lib/libtrac_ik_lib.a` 仅 x86-64（见上「跨架构」节），Jetson 链接前需先取得/编译 arm64 版 trac_ik。
3. **真臂行为一致性**：本次 SDK 清理为表面级（仅注释/改名，未改逻辑），但「行为与清理前完全一致」只能在真臂上跑 joint/end-pose 对比最终确认。

来源仓库存档：旧集成 `metal-python-ros` 已推送 GitHub `makermods-robotics/metal-python-ros`（含未移植的 metal_gazebo / metal_teach_mode / docker），需要时可 clone 找回。

## 重力补偿静态托不住（2026-07-15 实机排查根因）

**现象**：GRAVITY_COMPENSATION 示教模式下，臂无法悬停，缓慢下沉。

**根因**（非管线 bug，属模型/标定）：前馈力矩 = `重力系数·重力 + 科氏系数·科氏 + 摩擦`（`kdl_solver.cpp:FeedforwardTorqueCompensation`）。摩擦项**只有粘滞摩擦**（∝速度），库伦（静）摩擦被注释掉。静止时速度≈0（控制环还把<0.05 归零）→ 粘滞、科氏均为 0 → **前馈只剩「重力系数×重力力矩」**。缺任何静态兜底项（静摩擦/积分/微位置保持），只要重力估计比真实载荷略低（质心/惯量误差、电机力矩常数、减速器摩擦未建模），关节即无补偿地下沉。注释里的库伦项为 `sign(速度)×系数`，静止 `sign(0)=0` 亦无效。

**处理**：
- 快速实验（已做）：`gravity_torque_coe` 上调 ~15% → {1.38,1.32,1.27,1.32,1.15,1.15}（原 {1.2,1.15,1.1,1.15,1.0,1.0}）。若上飘则回调。
- 正解（待做）：加静态兜底（小 kp 位置微保持或重力误差积分）；重标定质心/电机力矩常数。

### 实机数据佐证（2026-07-15）
倒-L 姿势 q=[0.003,-0.978,1.051,...] 下对比「算出的前馈 total」vs「电机实际 eff」：
- **J3**：算 4.05 ≈ 实测 4.77 → 力矩链通、J3 重力补偿基本正确。
- **J2**：算 total 仅 0.229（grav 0.708 被 fric −0.338、cor −0.141 削掉），实测 eff **−1.08（反号）** → J2 重力项**偏低且方向不对**，强烈指向**该关节轴符号/零位与真实电机不一致**；且粘滞摩擦按当前速度计算，臂一沉即产生负摩擦、把保持力矩越削越小 → 越沉越快的正反馈。

**性质**：逐关节标定 + 摩擦前馈结构问题（非管线 bug，非百分比系数能救）。位置控制（保持/运动）完好，遥操/数采可用位置控制，故重力补偿**搁置**，待单独一轮标定：核对各关节轴符号、重标 COM、摩擦前馈改低速门控/加静态兜底。
