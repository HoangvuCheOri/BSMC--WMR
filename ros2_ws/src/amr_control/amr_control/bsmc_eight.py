import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import math


class BSMCEight(Node):
    def __init__(self):
        super().__init__('bsmc_eight')

        # ── Publishers / Subscribers ───────────────────────
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.err_pub = self.create_publisher(Point, '/tracking_error', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10
        )

        # ── Robot state ────────────────────────────────────
        self.current_x     = 0.0
        self.current_y     = 0.0
        self.current_theta = 0.0
        self.odom_received  = False
        self.last_odom_time = None
        self.start_time     = None

        # ── Initial pose (set on first odom) ──────────────
        self.x0     = 0.0
        self.y0     = 0.0
        self.theta0 = 0.0
        self.rot    = 0.0   # rotation to align traj with robot heading

        # ── Startup delay ──────────────────────────────────
        self.STARTUP_DELAY = 2.0

        # ── Figure-8 trajectory params ─────────────────────
        # x_d(t) = A·sin(w·t)
        # y_d(t) = B·sin(2·w·t)
        # Heading tại t=0: atan2(2Bw, Aw) = atan2(2B, A)
        self.A      = 0.5    # m — biên độ trục X
        self.B      = 0.25   # m — biên độ trục Y
        self.W      = 0.10   # rad/s — chu kỳ ≈ 63 s/vòng

        # ── Robot geometry ─────────────────────────────────
        self.L = 0.17        # wheelbase (m)

        # ── Backstepping gains (Kanayama-stable form) ──────
        # v_d nhân vào k2, k3 → ổn định Lyapunov
        self.k1 = 0.25
        self.k2 = 1.8
        self.k3 = 1.5

        # ── Sliding Mode (tắt trước khi backstepping ổn) ──
        self.Ks1 = 0.005
        self.Ks2 = 0.008

        # ── Boundary layer ─────────────────────────────────
        self.phi1 = 0.5
        self.phi2 = 1.0

        # ── Coupling e_y vào sliding surface s2 ───────────
        self.c = 0.5

        # ── Velocity limits ────────────────────────────────
        self.MAX_V = 0.32    # m/s
        self.MAX_W = 0.80    # rad/s

        # ── v_L minimum — tránh bánh trái đảo chiều ───────
        self.VL_MIN = 0.03   # m/s

        # ── Singularity guard cho w_d ──────────────────────
        self.V_DENOM_MIN = 1e-3

        # ── Deadband ───────────────────────────────────────
        self.DEADBAND_EY     = 0.005
        self.DEADBAND_ETHETA = 0.008

        # ── Debug counter ──────────────────────────────────
        self.debug_counter = 0

        # ── Control timer ──────────────────────────────────
        self.timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info(
            f"BSMC Figure-8 started. "
            f"A={self.A}, B={self.B}, W={self.W}, "
            f"T={2*math.pi/self.W:.1f}s/vong"
        )

    # ── Utilities ──────────────────────────────────────────
    def sat(self, z):
        return max(-1.0, min(1.0, z))

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def euler_from_quaternion(self, q):
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(t3, t4)

    # ── Odometry callback ───────────────────────────────────
    def odom_callback(self, msg):
        self.current_x     = msg.pose.pose.position.x
        self.current_y     = msg.pose.pose.position.y
        self.current_theta = self.euler_from_quaternion(msg.pose.pose.orientation)

        now_s = self.get_clock().now().nanoseconds / 1e9

        if not self.odom_received:
            self.start_time = now_s
            self.x0         = self.current_x
            self.y0         = self.current_y
            self.theta0     = self.current_theta

            # Heading của trajectory tại t=0
            dx0 = self.A * self.W
            dy0 = 2.0 * self.B * self.W
            theta_traj0 = math.atan2(dy0, dx0)

            # Góc xoay để căn chỉnh trajectory với heading thực
            self.rot = self.normalize_angle(self.theta0 - theta_traj0)

            self.get_logger().info(
                f"Odom received: x0={self.x0:.3f}, y0={self.y0:.3f}, "
                f"theta0={math.degrees(self.theta0):.1f} deg, "
                f"rot={math.degrees(self.rot):.1f} deg"
            )

        self.odom_received  = True
        self.last_odom_time = now_s

    # ── Reference trajectory ────────────────────────────────
    def generate_desired_trajectory(self, t):
        A, B, w = self.A, self.B, self.W
        wt  = w * t
        wt2 = 2.0 * wt

        # Local frame (unrotated)
        x_loc   =  A * math.sin(wt)
        y_loc   =  B * math.sin(wt2)
        dx_loc  =  A * w * math.cos(wt)
        dy_loc  =  2.0 * B * w * math.cos(wt2)
        ddx_loc = -A * w**2 * math.sin(wt)
        ddy_loc = -4.0 * B * w**2 * math.sin(wt2)

        # Rotate by self.rot + offset by (x0, y0)
        cr = math.cos(self.rot)
        sr = math.sin(self.rot)

        x_d   = self.x0 + cr * x_loc   - sr * y_loc
        y_d   = self.y0 + sr * x_loc   + cr * y_loc
        dx_d  = cr * dx_loc  - sr * dy_loc
        dy_d  = sr * dx_loc  + cr * dy_loc
        ddx_d = cr * ddx_loc - sr * ddy_loc
        ddy_d = sr * ddx_loc + cr * ddy_loc

        theta_d = math.atan2(dy_d, dx_d)
        v_d     = math.sqrt(dx_d**2 + dy_d**2)

        # w_d — guard singularity tại điểm giao cắt (v_d → 0)
        denom = max(dx_d**2 + dy_d**2, self.V_DENOM_MIN)
        w_d   = (dx_d * ddy_d - dy_d * ddx_d) / denom
        w_d   = max(-self.MAX_W, min(self.MAX_W, w_d))

        return x_d, y_d, theta_d, v_d, w_d

    # ── Main control loop ───────────────────────────────────
    def control_loop(self):
        if not self.odom_received:
            return

        now_s = self.get_clock().now().nanoseconds / 1e9

        # Timeout odometry
        if self.last_odom_time is not None and (now_s - self.last_odom_time) > 2.0:
            self.get_logger().warn("Odometry timeout — stopping.")
            self.cmd_pub.publish(Twist())
            return

        t = now_s - self.start_time

        # Startup delay — đứng yên chờ hệ thống ổn định
        if t < self.STARTUP_DELAY:
            self.cmd_pub.publish(Twist())
            return

        t_track = t - self.STARTUP_DELAY

        # ── Trajectory reference ───────────────────────────
        x_d, y_d, theta_d, v_d, w_d = self.generate_desired_trajectory(t_track)

        # ── Sai số trong body frame ────────────────────────
        dx = x_d - self.current_x
        dy = y_d - self.current_y
        ct = math.cos(self.current_theta)
        st = math.sin(self.current_theta)

        e_x     =  ct * dx + st * dy
        e_y     = -st * dx + ct * dy
        e_theta = self.normalize_angle(theta_d - self.current_theta)

        # Deadband
        if abs(e_y)     < self.DEADBAND_EY:     e_y     = 0.0
        if abs(e_theta) < self.DEADBAND_ETHETA: e_theta = 0.0

        # ── Sliding surfaces ───────────────────────────────
        s1 = e_x
        s2 = e_theta + self.c * e_y

        sat_s1 = self.sat(s1 / self.phi1)
        sat_s2 = self.sat(s2 / self.phi2)

        # ── Control law (Kanayama-stable) ──────────────────
        # v_cmd = v_d·cos(eθ) + k1·ex + Ks1·sat(s1)
        # w_cmd = w_d + v_d·(k2·ey + k3·sin(eθ)) + Ks2·sat(s2)
        #                ^^^^ v_d nhân vào k2, k3 — bắt buộc!
        v_cmd = (
            v_d * math.cos(e_theta)
            + self.k1 * e_x
            + self.Ks1 * sat_s1
        )

        w_cmd = (
            w_d
            + v_d * (self.k2 * e_y + self.k3 * math.sin(e_theta))
            + self.Ks2 * sat_s2
        )

        # ── Clamp v và w ───────────────────────────────────
        v_cmd = max(0.005, min(self.MAX_V, v_cmd))

        # Clamp w_cmd để v_L = v - w*(L/2) >= VL_MIN
        # tránh bánh trái đảo chiều → odometry sai
        w_max_safe = (v_cmd - self.VL_MIN) / (self.L / 2.0)
        w_limit    = min(self.MAX_W, max(0.0, w_max_safe))
        w_cmd      = max(-w_limit, min(w_limit, w_cmd))

        # ── Publish ────────────────────────────────────────
        err_msg   = Point()
        err_msg.x = float(e_x)
        err_msg.y = float(e_y)
        err_msg.z = float(e_theta)
        self.err_pub.publish(err_msg)

        cmd_msg = Twist()
        cmd_msg.linear.x  = float(v_cmd)
        cmd_msg.angular.z = float(w_cmd)
        self.cmd_pub.publish(cmd_msg)

        # ── Debug log (mỗi 1s) ────────────────────────────
        self.debug_counter += 1
        if self.debug_counter >= 20:
            self.debug_counter = 0
            v_L = v_cmd - w_cmd * (self.L / 2.0)
            v_R = v_cmd + w_cmd * (self.L / 2.0)
            self.get_logger().info(
                f"t={t_track:.1f}s | "
                f"ex={e_x:+.3f} ey={e_y:+.3f} eth={math.degrees(e_theta):+.1f}° | "
                f"v={v_cmd:.3f} w={w_cmd:+.3f} | "
                f"vL={v_L:.3f} vR={v_R:.3f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = BSMCEight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()