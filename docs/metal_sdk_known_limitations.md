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
