import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


class BSMCController(Node):
    def __init__(self):
        super().__init__('bsmc_controller')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )

        self.current_x     = 0.0
        self.current_y     = 0.0
        self.current_theta = 0.0
        self.odom_received = False
        self.last_odom_time = None

        self.timer_period = 0.2
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.start_time = None

        # ── Backstepping gains ──────────────────────────────────────────
        # WiFi (cũ): k1=0.5, k2=8.0, k3=3.0
        # ESP-NOW: giảm k2, k3 vì round-trip delay tăng ~30-80ms
        # k2 cao + delay → oscillate ngang → không đi thẳng
        self.k1 = 0.3      # longitudinal: giảm nhẹ
        self.k2 = 2.5      # lateral:      giảm 50% (nguyên nhân chính oscillation)
        self.k3 = 1.5      # heading:      giảm ~33%

        # ── Sliding Mode gains ──────────────────────────────────────────
        # Giảm Ks để tránh chattering với data rate không đều của ESP-NOW
        self.Ks1 = 0.02
        self.Ks2 = 0.04

        # ── Boundary layer ──────────────────────────────────────────────
        # Tăng phi → vùng sat rộng hơn → mượt hơn khi có jitter
        self.phi1 = 0.2   # (cũ: 0.10)
        self.phi2 = 0.4   # (cũ: 0.20)

        # ── Sliding surface coefficient ─────────────────────────────────
        self.c = 1.0       # (cũ: 1.5) — giảm nhẹ, ít nhạy với e_y

        # ── Safety limits ───────────────────────────────────────────────
        self.MAX_V = 0.3
        self.MAX_W = 1.0   # (cũ: 1.5) — giảm giới hạn xoay để ổn định hơn

        # ── Dead-band nhỏ cho e_y và e_theta ───────────────────────────
        # Bỏ qua sai số nhỏ sinh ra từ noise packet / jitter timestamp
        self.DEADBAND_EY    = 0.005  # m   — dưới 5mm không hiệu chỉnh ngang
        self.DEADBAND_ETHETA = 0.01  # rad — dưới ~0.6° không hiệu chỉnh góc

        self.get_logger().info(
            "BSMC (ESP-NOW tuned) started. "
            "k1=%.2f k2=%.2f k3=%.2f  phi1=%.2f phi2=%.2f" %
            (self.k1, self.k2, self.k3, self.phi1, self.phi2)
        )

    # ────────────────────────────────────────────────────────────────────
    def sat(self, z):
        return max(-1.0, min(1.0, z))

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def euler_from_quaternion(self, quaternion):
        x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
        t0 = 2.0 * (w * x + y * z)
        t1 = 1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        t2 = max(min(2.0 * (w * y - z * x), 1.0), -1.0)
        pitch_y = math.asin(t2)
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        return roll_x, pitch_y, yaw_z

    # ────────────────────────────────────────────────────────────────────
    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        _, _, yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.current_theta = yaw

        if not self.odom_received:
            self.start_time = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info("First odometry received — trajectory tracking started.")

        self.odom_received = True
        self.last_odom_time = self.get_clock().now().nanoseconds / 1e9

    # ────────────────────────────────────────────────────────────────────
    def generate_desired_trajectory(self, t):
        v_const = 0.2
        x_d     = v_const * t
        y_d     = 0.0
        theta_d = 0.0
        v_d     = v_const
        w_d     = 0.0
        return x_d, y_d, theta_d, v_d, w_d

    # ────────────────────────────────────────────────────────────────────
    def control_loop(self):
        if not self.odom_received:
            return

        # Timeout guard
        now_s = self.get_clock().now().nanoseconds / 1e9
        if self.last_odom_time is not None and (now_s - self.last_odom_time) > 2.0:
            self.get_logger().warn(
                "[BSMC] /odometry/filtered timeout (>2s) — dừng gửi cmd_vel!",
                throttle_duration_sec=2.0
            )
            return

        t = now_s - self.start_time
        x_d, y_d, theta_d, v_d, w_d = self.generate_desired_trajectory(t)

        dx = x_d - self.current_x
        dy = y_d - self.current_y

        # Sai số trong hệ tọa độ thân xe
        e_x     = math.cos(self.current_theta) * dx + math.sin(self.current_theta) * dy
        e_y     = -math.sin(self.current_theta) * dx + math.cos(self.current_theta) * dy
        e_theta = self.normalize_angle(theta_d - self.current_theta)

        # ── Dead-band: bỏ qua noise nhỏ sinh ra từ jitter ESP-NOW ──────
        if abs(e_y) < self.DEADBAND_EY:
            e_y = 0.0
        if abs(e_theta) < self.DEADBAND_ETHETA:
            e_theta = 0.0

        # ── Mặt trượt ───────────────────────────────────────────────────
        s1 = e_x
        s2 = e_theta + self.c * e_y

        sat_s1 = self.sat(s1 / self.phi1)
        sat_s2 = self.sat(s2 / self.phi2)

        # ── Backstepping + SMC ──────────────────────────────────────────
        v_cmd = (
            v_d * math.cos(e_theta)
            + self.k1 * e_x
            + self.Ks1 * sat_s1
        )

        w_cmd = (
            w_d
            + self.k2 * e_y
            + self.k3 * math.sin(e_theta)
            + self.Ks2 * sat_s2
        )

        # Clamp
        v_cmd = max(-self.MAX_V, min(self.MAX_V, v_cmd))
        w_cmd = max(-self.MAX_W, min(self.MAX_W, w_cmd))

        cmd_msg = Twist()
        cmd_msg.linear.x  = float(v_cmd)
        cmd_msg.angular.z = float(w_cmd)
        self.cmd_pub.publish(cmd_msg)


# ════════════════════════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)
    node = BSMCController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_msg = Twist()
        node.cmd_pub.publish(stop_msg)
        node.get_logger().info("BSMC stopped — STOP command sent.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()