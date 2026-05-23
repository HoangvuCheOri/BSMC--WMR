import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import math
import sys
import select
import threading
from std_msgs.msg import Bool


class BSMCCircle(Node):
    def __init__(self):
        super().__init__('bsmc_circle')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.err_pub = self.create_publisher(Point, '/tracking_error', 10)

        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10
        )

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        self.odom_received = False
        self.last_odom_time = None
        self.start_time = None

        self.x0 = 0.0
        self.y0 = 0.0
        self.theta0 = 0.0

        self.timer_period = 0.3
        self.timer = self.create_timer(self.timer_period, self.control_loop)

        self.STARTUP_DELAY = 2.0
        
        # Pause functionality
        self.is_paused = False
        self.total_paused_time = 0.0
        self.pause_start_time = None
        self.pause_sub = self.create_subscription(Bool, '/pause_control', self.pause_cb, 10)
        
        self.kb_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.kb_thread.start()

        # Circle trajectory
        self.R = 0.5
        self.W = 0.25          # giảm từ 0.4 để dễ bám hơn
        self.VD = self.R * self.W

        # Backstepping gains
        self.k1 = 1.1
        self.k2 = 1.2
        self.k3 = 1.1

        # Weak SMC
        self.Ks1 = 0.005
        self.Ks2 = 0.015

        self.phi1 = 0.45
        self.phi2 = 1.2

        # Coupling lateral error to heading sliding surface
        self.c = 1.0

        # Velocity limits
        self.MAX_V = 0.35
        self.MAX_W = 0.7

        # Deadband
        self.DEADBAND_EX = 0.003
        self.DEADBAND_EY = 0.004
        self.DEADBAND_ETHETA = 0.006

        # Nếu robot càng chạy càng lệch ngang, đổi thành True
        self.INVERT_EY = False

        self.debug_counter = 0

        self.get_logger().info(
            f"BSMC Circle stable version started. "
            f"R={self.R}, W={self.W}, vd={self.VD:.2f}"
        )
        self.get_logger().info(">>> Nhấn 'p' rồi Enter trên terminal này để TẠM DỪNG / CHẠY TIẾP <<<")

    def pause_cb(self, msg):
        self.toggle_pause(msg.data)

    def keyboard_loop(self):
        while rclpy.ok():
            i, o, e = select.select([sys.stdin], [], [], 0.5)
            if i:
                key = sys.stdin.readline().strip().lower()
                if key == 'p':
                    self.toggle_pause(not self.is_paused)

    def toggle_pause(self, state):
        if self.is_paused != state:
            self.is_paused = state
            if self.is_paused:
                self.get_logger().warn(">>> PAUSED! Gửi lệnh dừng robot. Nhấn 'p'+Enter để chạy tiếp. <<<")
            else:
                self.get_logger().info(">>> RESUMED! Tiếp tục bám quỹ đạo. <<<")

    def sat(self, z):
        return max(-1.0, min(1.0, z))

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def euler_from_quaternion(self, q):
        x, y, z, w = q.x, q.y, q.z, q.w
        t3 = 2.0 * (w * z + x * y)
        t4 = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        self.current_theta = self.euler_from_quaternion(
            msg.pose.pose.orientation
        )

        now_s = self.get_clock().now().nanoseconds / 1e9

        if not self.odom_received:
            self.start_time = now_s

            self.x0 = self.current_x
            self.y0 = self.current_y
            self.theta0 = self.current_theta

            self.get_logger().info(
                f"Odometry received. Initial pose: "
                f"x0={self.x0:.3f}, y0={self.y0:.3f}, "
                f"theta0={math.degrees(self.theta0):.1f} deg"
            )

        self.odom_received = True
        self.last_odom_time = now_s

    def generate_desired_trajectory(self, t):
        wt = self.W * t

        # Circle in local frame
        x_local = self.R * math.sin(wt)
        y_local = self.R * (1.0 - math.cos(wt))
        theta_local = wt

        # Rotate trajectory by initial robot heading
        cos0 = math.cos(self.theta0)
        sin0 = math.sin(self.theta0)

        x_d = self.x0 + cos0 * x_local - sin0 * y_local
        y_d = self.y0 + sin0 * x_local + cos0 * y_local
        theta_d = self.normalize_angle(self.theta0 + theta_local)

        v_d = self.VD
        w_d = self.W

        return x_d, y_d, theta_d, v_d, w_d

    def control_loop(self):
        if not self.odom_received:
            return

        now_s = self.get_clock().now().nanoseconds / 1e9

        if self.last_odom_time is not None:
            if now_s - self.last_odom_time > 2.0:
                self.get_logger().warn("Odometry timeout. Stopping robot.")
                self.cmd_pub.publish(Twist())
                return

        # Tính toán thời gian Pause
        if self.is_paused:
            if self.pause_start_time is None:
                self.pause_start_time = now_s
            self.cmd_pub.publish(Twist()) # Liên tục gửi lệnh dừng
            return
        else:
            if self.pause_start_time is not None:
                self.total_paused_time += (now_s - self.pause_start_time)
                self.pause_start_time = None

        # Trừ đi khoảng thời gian đã dừng để quỹ đạo ảo không chạy mất
        t = now_s - self.start_time - self.total_paused_time

        if t < self.STARTUP_DELAY:
            self.cmd_pub.publish(Twist())
            return

        t_track = t - self.STARTUP_DELAY

        x_d, y_d, theta_d, v_d, w_d = self.generate_desired_trajectory(t_track)

        dx = x_d - self.current_x
        dy = y_d - self.current_y

        cos_th = math.cos(self.current_theta)
        sin_th = math.sin(self.current_theta)

        e_x = cos_th * dx + sin_th * dy
        e_y = -sin_th * dx + cos_th * dy

        if self.INVERT_EY:
            e_y = -e_y

        e_theta = self.normalize_angle(theta_d - self.current_theta)

        if abs(e_x) < self.DEADBAND_EX:
            e_x = 0.0

        if abs(e_y) < self.DEADBAND_EY:
            e_y = 0.0

        if abs(e_theta) < self.DEADBAND_ETHETA:
            e_theta = 0.0

        s1 = e_x
        s2 = e_theta + self.c * e_y

        sat_s1 = self.sat(s1 / self.phi1)
        sat_s2 = self.sat(s2 / self.phi2)

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

        v_cmd = max(0.0, min(self.MAX_V, v_cmd))
        w_cmd = max(-self.MAX_W, min(self.MAX_W, w_cmd))

        err_msg = Point()
        err_msg.x = float(e_x)
        err_msg.y = float(e_y)
        err_msg.z = float(e_theta)
        self.err_pub.publish(err_msg)

        cmd_msg = Twist()
        cmd_msg.linear.x = float(v_cmd)
        cmd_msg.angular.z = float(w_cmd)
        self.cmd_pub.publish(cmd_msg)

        self.debug_counter += 1
        if self.debug_counter >= 20:
            self.debug_counter = 0
            self.get_logger().info(
                f"t={t_track:.1f}s | "
                f"ex={e_x:+.3f}, ey={e_y:+.3f}, eth={e_theta:+.3f} | "
                f"v={v_cmd:.3f}, w={w_cmd:.3f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = BSMCCircle()

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
