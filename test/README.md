# Metal 臂实机测试脚本

按编号**从上到下一个一个跑**，安全等级递增。每个都是一条命令：`bash test/tX_xxx.sh`。

> 环境陷阱已由 `_env.sh` 自动处理（conda 遮蔽 → 强制 `/usr/bin/python3` 3.10 + source ROS/workspace）。无需手动 source。

| 脚本 | 等级 | 作用 | 会不会动 |
|------|------|------|---------|
| `t0_can_up.sh` | 🟢 sudo | 拉起 can0（CANable slcan @1Mbps）+ candump 检查 | 否 |
| `t1_read.sh` | 🟢 只读 | metal_sdk 直连读关节/温度/错误码/末端位姿 | 否（不使能） |
| `t2_node.sh` | 🟢 只读 | 跑 ROS 驱动节点(leader_arm, auto_enable=false)，echo 话题 | 否（电机 OFF） |
| `t3_rviz.sh` | 🟢 只读 | RViz 里渲染 URDF 模型（滑条控关节，验证 mesh/描述包） | 否（需装 rviz2/xacro） |
| `t4_gravity.sh` | 🟡 通电 | 重力补偿示教：臂托住自重、可手动拖动，**不自主运动** | 手动可动 |
| `t5_hold.sh` | 🟡 通电 | 使能 + 指令当前位置，臂应**保持不动**（验证指令通路） | 基本不动 |
| `t6_move.sh` | 🔴 运动 | 单关节小幅运动再回位（默认 J1 +0.15rad，最慢速） | **会动** |

## 用法要点

- **顺序**：先 `t0` 拉 CAN，再逐级往下。`t1` 过了才说明 CAN↔臂通。
- **安全**：🟡/🔴 脚本会通电，跑前**清空臂周围、手放 e-stop**。`t4/t5/t6` 会要你输入 `yes` 确认（加 `--yes` 可跳过）。
- **停止**：`t4`（重力补偿循环）按 `Ctrl-C` 退出；`t6` 跑完自动回位并切到重力补偿（安全、可拖动）。
- **t6 自定义**：`bash test/t6_move.sh --joint 2 --delta 0.1`（动 J3、幅度 0.1rad）。
- **收尾状态**：运动类脚本结束会把臂留在**重力补偿**（托住自重、可手动摆放），不是刚性锁死也不是掉电软塌。

## 前置

- CAN 适配器：CANable(slcan) @ `/dev/ttyACM0`；udev 规则 `scripts/makermods_metal_can.rules`（序列号需匹配本机适配器）。
- `t3` 需要：`sudo apt install -y ros-humble-rviz2 ros-humble-xacro ros-humble-joint-state-publisher-gui`。
- SDK 已构建：`bash src/metal_sdk/build_metal_sdk.sh` + `colcon build`（详见根目录 README）。

## 已知点

- SDK 的 `kdl_solver` 硬编码链名 `base_link → Link6`（**大写**）。脚本用的是 `src/metal_arm_driver/urdf/`（大写连杆名），**不是** `metal_arm_description/` 的 urdf（小写、给 RViz 用）。
- DM 电机是请求-应答式，空闲 CAN 总线 `candump` 可能为空，属正常。
