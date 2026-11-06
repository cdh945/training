# 第二次作业：ROS2基础

本作业配套《Venom 算法组：ROS2基础》。使用 Ubuntu 22.04、Python3.10、VS Code 和 Git 完成练习。

## 控制结构

| 模块 | 职责 |
|---|---|
| 状态解算 | 解析 6 关节角/角速度、IMU 四元数/角速度，得到 pitch/roll |
| 五连杆正解 (`vmc_cal.py`) | 由左右髋角解算虚拟腿长 `L_0` 与虚拟腿倾角 `theta` |
| LQR (`lqr_update.py`) | 按当前 `L_0` 查 12 维状态反馈矩阵 `K` |
| 腿长闭环 | `F_0 = 4/cos(theta) + kp·e + kd·de/dt`，把 `L_0` 稳到目标 |
| 横滚分配 (`roll_leg_control.py`) | 根据 roll 角差模分配左右目标腿长 |
| VMC 映射 | 把 `F_0` 和 LQR 输出的髋部力矩 `T_p` 映射为 2 个关节力矩 |
| 轮力矩 | 由 LQR 的 `K` 直接组合输出，符号左右镜像 |

控制节拍与仿真同步：**每收到一条 `/joint_states` 就发布一条 `/actuator_cmd`**，dt 固定 0.002 s。

## K 与参数来源

| 项 | 来源 |
|---|---|
| 12 组 `K` 多项式系数 (`POLY_COEFFICIENT`) | MATLAB 离线整定（`Wheel_Leg/LQR_VMC` 原始工程），按 `L_0` 三次多项式插值 |
| 腿长 `L_0` / `theta` 正解 | `Wheel_Leg/LQR_VMC/vmc_cal.py` 原样复制 |
| 腿长 PID 初始值 (`kp=700, kd=10`) | 原始工程 `main.py` |
| 前馈 `4/cos(theta)` | 原始工程重力补偿 |
| 横滚差模 (`RollLegController`) | `Wheel_Leg/LQR_VMC/roll_leg_control.py` 原样复制 |
| 离地检测 (`ground_detection.py`) | 原始工程（默认关闭） |

ROS 参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `target_length` | 0.10 m | 目标腿长 |
| `leg_kp / leg_kd` | 700 / 10 | 腿长闭环 |
| `feed_forward_force` | 4.0 N | 重力前馈 |
| `wheel_torque_limit` | 1.5 N·m | 轮执行器限幅 |
| `yaw_kp / yaw_kd` | 1.0 / 0.1 | 转向差力矩 |
| `ground_detection_enabled` | false | 离地保护 |
| `log_period` | 0.5 s | 日志限频 |

## 实测数据

原地平衡（平衡30s中后25s数据）：

| 项 | 均值 | 是否可接受 |
|---|---|---|
| 腿长 | 0.0985m | 通过 |
| pitch | -0.002rad | 通过 |
| 漂移 | -0.087m | 通过 |

## 学习心得

本次作业大多由AI辅助完成
1. ros2的模块化设计使它对项目结构有一定要求，基本上是/src的同级文件夹及其子目录的名字不能乱动，而/src的父文件夹代表项目名，可以改动。
2. 动力学及数学模块中的函数我没看懂，参数也不会填，故大量运用了AI工具完成，但代码逻辑我还是能看懂的。
3. 同一个.py文件下可以有多个节点分担不同的功能，可通过rclpy.executors.MultiThreadedExecutor()并行运行节点。
4. 类似于3.，同一个节点可以挂载不同的功能，因此监听数据可以处理后作为发送数据。
5. 通过ros2 pkg create可方便地创建易管理的包，此时包内的.py文件中导入的库会自动写入.xml文件的depend中
6. 自动创建的包中，有test文件夹test_depend依赖，ros2藉此检查依赖、许可证等，因此可能会弹出无许可证的警告。（也可删除测试文件）
7. 现在VS Code中已无ros2插件，更名为Robot Developer Extensions且有两个包。
