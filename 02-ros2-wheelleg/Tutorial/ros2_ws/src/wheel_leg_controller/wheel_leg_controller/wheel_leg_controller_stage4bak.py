"""阶段 4：LQR + VMC 平衡控制。

移植自原始 MuJoCo 工程的 control_left_leg / control_right_leg，保持以下约定：

- 一条 /joint_states 对应一条 /actuator_cmd、一个仿真 dt (0.002 s)；
- IMU 的 pitch / gyro_pitch / roll 取负，gyro_yaw 取 angular_velocity.x，与原始工程一致；
- 左腿 VMC 输出取负，轮力矩为 -wheel_T - yaw_ctrl；右腿直接使用；
- 离地检测默认关闭，排查时可用 -p ground_detection_enabled:=true 打开。
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, JointState

from rm_msgs.msg import WheelLegCmd

from .vmc_cal import five_links_param
from .lqr_update import lqr_controller
from .roll_leg_control import RollLegController
from .ground_detection import detect_leg_ground_status


def clip(value, low, high):
    return max(low, min(high, value))


class PIDController:
    """与原始工程一致的离散 PID：积分、微分都不按 dt 缩放。"""

    def __init__(self, kp, ki, kd, max_out, max_i):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_out = max_out
        self.max_i = max_i
        self.integral = 0.0
        self.prev_error = 0.0

    def compute(self, current, target):
        error = target - current
        self.integral += error
        self.integral = clip(self.integral, -self.max_i, self.max_i)
        derivative = error - self.prev_error
        self.prev_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return clip(output, -self.max_out, self.max_out)


class WheelLegBalanceController(Node):
    def __init__(self):
        super().__init__("wheel_leg_balance_controller")

        # ---------------- 参数 ----------------
        self.declare_parameter("dt", 0.002)
        self.declare_parameter("wheel_radius", 0.06)
        self.declare_parameter("target_length", 0.10)

        self.declare_parameter("leg_kp", 700.0)
        self.declare_parameter("leg_ki", 0.0)
        self.declare_parameter("leg_kd", 5.0)
        self.declare_parameter("leg_force_limit", 150.0)
        self.declare_parameter("max_leg_force", 100.0)

        self.declare_parameter("yaw_kp", 0.5)
        self.declare_parameter("yaw_ki", 0.0)
        self.declare_parameter("yaw_kd", 1.0)
        self.declare_parameter("yaw_limit", 2.0)
        self.declare_parameter("wheel_torque_limit", 1.5)

        self.declare_parameter("roll_kp", 1.0)
        self.declare_parameter("roll_ki", 0.0)
        self.declare_parameter("roll_kd", 0.0)
        self.declare_parameter("roll_max_out", 0.14)
        self.declare_parameter("roll_min_leg", 0.10)
        self.declare_parameter("roll_max_leg", 0.14)

        self.declare_parameter("ground_detection_enabled", False)
        self.declare_parameter("log_period", 0.5)

        self.dt = float(self.get_parameter("dt").value)
        self.wheel_radius = float(self.get_parameter("wheel_radius").value)
        self.target_length = float(self.get_parameter("target_length").value)

        self.leg_force_limit = float(self.get_parameter("leg_force_limit").value)
        self.max_leg_force = float(self.get_parameter("max_leg_force").value)
        self.yaw_limit = float(self.get_parameter("yaw_limit").value)
        self.wheel_torque_limit = float(self.get_parameter("wheel_torque_limit").value)
        self.ground_detection_enabled = bool(
            self.get_parameter("ground_detection_enabled").value)
        self.log_period = float(self.get_parameter("log_period").value)

        # ---------------- 状态 ----------------
        self.pitch = 0.0
        self.gyro_pitch = 0.0
        self.roll = 0.0
        self.gyro_yaw = 0.0

        self.target_v = 0.0        # /cmd_vel 线速度 -> v_set
        self.target_w = 0.0        # /cmd_vel 角速度 -> 偏航目标
        self.x_b_set = 0.0         # 位置参考：运动时跟随，站桩冻结
        self.is_airborne = False   # 上一拍是否触发离地保护
        self.sim_time = 0.0        # 累计仿真时间（每条 /joint_states = 一个 dt）
        self.last_log_time = 0.0
        self.missing_warned = False

        # ---------------- 控制对象 ----------------
        self.fl_right = five_links_param(self.dt)
        self.fl_left = five_links_param(self.dt)

        self.lqr_right = lqr_controller(self.dt)
        self.lqr_left = lqr_controller(self.dt)

        self.right_l0_pid = PIDController(
            float(self.get_parameter("leg_kp").value),
            float(self.get_parameter("leg_ki").value),
            float(self.get_parameter("leg_kd").value),
            self.leg_force_limit, 0.0)
        self.left_l0_pid = PIDController(
            float(self.get_parameter("leg_kp").value),
            float(self.get_parameter("leg_ki").value),
            float(self.get_parameter("leg_kd").value),
            self.leg_force_limit, 0.0)

        self.right_yaw_pid = PIDController(
            float(self.get_parameter("yaw_kp").value),
            float(self.get_parameter("yaw_ki").value),
            float(self.get_parameter("yaw_kd").value),
            self.yaw_limit, 0.0)
        self.left_yaw_pid = PIDController(
            float(self.get_parameter("yaw_kp").value),
            float(self.get_parameter("yaw_ki").value),
            float(self.get_parameter("yaw_kd").value),
            self.yaw_limit, 0.0)

        self.roll_balancer = RollLegController(
            kp=float(self.get_parameter("roll_kp").value),
            ki=float(self.get_parameter("roll_ki").value),
            kd=float(self.get_parameter("roll_kd").value),
            max_out=float(self.get_parameter("roll_max_out").value),
            min_leg=float(self.get_parameter("roll_min_leg").value),
            max_leg=float(self.get_parameter("roll_max_leg").value))

        # ---------------- 接口 ----------------
        self.cmd_pub = self.create_publisher(WheelLegCmd, "actuator_cmd", 10)
        self.create_subscription(Imu, "imu", self.on_imu, 10)
        self.create_subscription(JointState, "joint_states", self.on_joint_state, 10)
        self.create_subscription(Twist, "cmd_vel", self.on_cmd_vel, 10)

        self.get_logger().info(
            f"阶段 4 LQR 平衡已启动：目标腿长={self.target_length:.3f} m, "
            f"wd={self.wheel_radius} m, 离地检测={'开' if self.ground_detection_enabled else '关'}")

    # ------------------------------------------------------------------
    def on_imu(self, msg):
        q = msg.orientation
        # 与原始工程一致：pitch / roll 取负，gyro_yaw 用 angular_velocity.x
        sin_pitch = 2.0 * (q.w * q.y - q.z * q.x)
        sin_pitch = clip(sin_pitch, -1.0, 1.0)
        pitch = math.asin(sin_pitch)
        roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z),
                          1.0 - 2.0 * (q.x * q.x + q.y * q.y))

        self.pitch = -pitch
        self.gyro_pitch = -float(msg.angular_velocity.y)
        self.roll = -roll
        self.gyro_yaw = float(msg.angular_velocity.x)

    def on_cmd_vel(self, msg):
        self.target_v = float(msg.linear.x)
        self.target_w = float(msg.angular.z)

    # ------------------------------------------------------------------
    def on_joint_state(self, msg):
        pos = {name: float(p) for name, p in zip(msg.name, msg.position)}
        vel = {name: float(v) for name, v in zip(msg.name, msg.velocity)}

        need = ("right1", "right2", "left1", "left2", "rightwheel", "leftwheel")
        if any(name not in pos or name not in vel for name in need):
            if not self.missing_warned:
                self.get_logger().warn(
                    f"/joint_states 字段不全，当前: {list(pos)}")
                self.missing_warned = True
            return

        # 共模提取：前进为正
        common_pos = (pos["leftwheel"] - pos["rightwheel"]) / 2.0
        common_speed = (vel["leftwheel"] - vel["rightwheel"]) / 2.0
        self.sim_time += self.dt

        # 横滚平衡：把基础腿长差模分配给左右腿
        target_len_left, target_len_right = self.roll_balancer.compute_leg_lengths(
            self.roll, 0.0, self.target_length)

        # 位置参考：运动时跟随轮子位置，站桩时冻结（与原始工程一致）
        if abs(self.target_v) > 1e-6:
            self.x_b_set = common_pos * self.wheel_radius
        self.lqr_left.x_b_set = self.x_b_set
        self.lqr_right.x_b_set = self.x_b_set
        self.lqr_left.v_set = self.target_v
        self.lqr_right.v_set = self.target_v

        # 左右腿控制解算
        l_t1, l_t4, l_wheel, l_f0, l_air = self._control_leg(
            1, self.fl_left, self.lqr_left, self.left_l0_pid, self.left_yaw_pid,
            pos["left1"], pos["left2"], target_len_left, common_pos, common_speed)
        r_t1, r_t4, r_wheel, r_f0, r_air = self._control_leg(
            0, self.fl_right, self.lqr_right, self.right_l0_pid, self.right_yaw_pid,
            pos["right1"], pos["right2"], target_len_right, common_pos, common_speed)

        self.is_airborne = l_air or r_air

        cmd = WheelLegCmd()
        cmd.right_phi1 = r_t1
        cmd.right_phi4 = r_t4
        cmd.left_phi1 = l_t1
        cmd.left_phi4 = l_t4
        cmd.right_wheel = r_wheel
        cmd.left_wheel = l_wheel
        self.cmd_pub.publish(cmd)

        self.log_status(l_t1, l_t4, r_t1, r_t4, l_wheel, r_wheel, l_f0, r_f0)

    # ------------------------------------------------------------------
    def _control_leg(self, leg_flag, five_links, lqr, l0_pid, yaw_pid,
                     motor1, motor2, target_length, common_pos, common_speed):
        """单腿 LQR + VMC 解算，返回 (phi1 力矩, phi4 力矩, 轮力矩, F_0, 是否离地)。"""
        five_links.forward_kinematics_cal(
            leg_flag, motor1, motor2, self.pitch, self.gyro_pitch)

        # 右腿传负共模，左腿传正共模
        if leg_flag == 0:
            lqr.states_update(0, five_links, self.pitch, self.gyro_pitch,
                              -common_pos, -common_speed, self.wheel_radius)
        else:
            lqr.states_update(1, five_links, self.pitch, self.gyro_pitch,
                              common_pos, common_speed, self.wheel_radius)
        lqr.calc_k_matrix_from_poly(five_links.L_0)

        # 腿长闭环 + 自重前馈 -> 沿腿方向的虚拟力
        cos_theta = math.cos(five_links.theta)
        feed_forward = 4.0 / cos_theta if abs(cos_theta) > 1e-6 else 0.0
        f0 = clip(feed_forward + l0_pid.compute(five_links.L_0, target_length),
                  -self.max_leg_force, self.max_leg_force)

        # 髋部虚拟力矩（正常着地状态）
        t_p = (lqr.K[1] * (lqr.theta - 0.0)
               + lqr.K[3] * (lqr.d_theta - 0.0)
               + lqr.K[5] * (lqr.x_b - lqr.x_b_set)
               + lqr.K[7] * (lqr.v_b_whole - lqr.v_set)
               + lqr.K[9] * (lqr.phi - 0.0)
               + lqr.K[11] * (lqr.d_phi - 0.0))

        airborne = False
        if (self.ground_detection_enabled
                and (detect_leg_ground_status(five_links, f0, t_p) == 1
                     or self.is_airborne)):
            # 离地保护：轮力矩清零，髋关节只保留空中阻尼
            wheel_t = 0.0
            t_p = lqr.K[1] * (lqr.theta - 0.0) + lqr.K[3] * (lqr.d_theta - 0.0)
            airborne = True
        else:
            wheel_raw = (lqr.K[0] * (lqr.theta - 0.0)
                         + lqr.K[2] * (lqr.d_theta - 0.0)
                         + lqr.K[4] * (lqr.x_b - lqr.x_b_set)
                         + lqr.K[6] * (lqr.v_b_whole - lqr.v_set)
                         + lqr.K[8] * (lqr.phi - 0.0)
                         + lqr.K[10] * (lqr.d_phi - 0.0))
            yaw_ctrl = yaw_pid.compute(self.gyro_yaw, self.target_w)
            if leg_flag == 1:
                wheel_t = -wheel_raw - yaw_ctrl
            else:
                wheel_t = wheel_raw - yaw_ctrl
            wheel_t = clip(wheel_t, -self.wheel_torque_limit, self.wheel_torque_limit)

        # VMC 映射；左腿取负
        t1, t4 = five_links.VMC_torque_cal(f0, t_p)
        if leg_flag == 1:
            t1, t4 = -t1, -t4

        return float(t1), float(t4), float(wheel_t), float(f0), airborne

    # ------------------------------------------------------------------
    def log_status(self, l_t1, l_t4, r_t1, r_t4, l_wheel, r_wheel, l_f0, r_f0):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last_log_time < self.log_period:
            return
        self.last_log_time = now

        phi_deg = math.degrees(self.pitch)
        roll_deg = math.degrees(self.roll)
        # x_b = 轮子转角 × 轮半径，即机体前进位移；x_b - x_b_set 是位置环偏差
        x_b = self.lqr_left.x_b
        x_b_err = x_b - self.lqr_left.x_b_set

        self.get_logger().info(
            f"t={self.sim_time:.2f}s | "
            f"phi={self.pitch:+.3f} rad ({phi_deg:+.1f}°), "
            f"roll={self.roll:+.3f} rad ({roll_deg:+.1f}°) | "
            f"位移 x_b={x_b:+.3f} m (漂移 {x_b_err:+.3f} m), "
            f"车速 v_b={self.lqr_left.v_b_whole:+.3f} m/s | "
            f"theta: 左={self.lqr_left.theta:+.3f}, 右={self.lqr_right.theta:+.3f} | "
            f"L0: 左={self.fl_left.L_0:.4f}, 右={self.fl_right.L_0:.4f} m | "
            f"F0: 左={l_f0:+.1f}, 右={r_f0:+.1f} N | "
            f"轮力矩: 左={l_wheel:+.2f}, 右={r_wheel:+.2f} N·m"
            + (" [离地保护]" if self.is_airborne else ""))


def main(args=None):
    rclpy.init(args=args)
    node = WheelLegBalanceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
