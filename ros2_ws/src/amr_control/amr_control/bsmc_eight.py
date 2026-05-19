import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
import math

class BSMCEight(Node):
    def __init__(self):
        super().__init__('bsmc_eight')

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

        self.timer_period = 0.2
        self.timer = self.create_timer(self.timer_period, self.control_loop)
        self.start_time = None

        # ── Backstepping gains (Tuned for Figure-8) ────────
        self.k1 = 0.3      
        self.k2 = 3.5      # Tăng k2 để bám các khúc cua gấp ở tâm số 8
        self.k3 = 1.5      

        # ── Sliding Mode gains ─────────────────────────────
        self.Ks1 = 0.02
        self.Ks2 = 0.05

        # ── Boundary layer ─────────────────────────────────
        self.phi1 = 0.2
        self.phi2 = 0.4

        self.c = 1.0

        self.MAX_V = 0.35  # Tăng nhẹ Vmax vì số 8 có lúc cần tăng tốc
        self.MAX_W = 1.2   # nới rộng để tránh bão hòa tại điểm giao

        self.V_DENOM_MIN = 1e-6

        self.DEADBAND_EY    = 0.005
        self.DEADBAND_ETHETA = 0.01

        self.get_logger().info("BSMC Figure-8 Trajectory Tracking started.")

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

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        _, _, yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.current_theta = yaw

        if not self.odom_received:
            self.start_time = self.get_clock().now().nanoseconds / 1e9
            
            # ── Lưu pose ban đầu ──────────────────────────────
            self.x0     = self.current_x
            self.y0     = self.current_y
            self.theta0 = self.current_theta
            
            # Heading của quỹ đạo tại t=0
            A, B, w = 0.5, 0.25, 0.10
            theta_traj0 = math.atan2(2 * B * w, A * w)
            
            # Góc xoay để align trajectory với heading thực của robot
            self.rot = self.normalize_angle(self.theta0 - theta_traj0)

        self.odom_received = True
        self.last_odom_time = self.get_clock().now().nanoseconds / 1e9

    def generate_desired_trajectory(self, t):
        A = 0.5
        B = 0.25
        w = 0.10
        wt  = w * t
        wt2 = 2.0 * wt

        # Local trajectory
        x_local  =  A * math.sin(wt)
        y_local  =  B * math.sin(wt2)
        dx_local =  A * w * math.cos(wt)
        dy_local =  2.0 * B * w * math.cos(wt2)
        ddx_local = -A * (w**2) * math.sin(wt)
        ddy_local = -4.0 * B * (w**2) * math.sin(wt2)

        # ── Rotate + offset theo pose ban đầu ─────────────────
        cos_r = math.cos(self.rot)
        sin_r = math.sin(self.rot)

        x_d = self.x0 + cos_r * x_local - sin_r * y_local
        y_d = self.y0 + sin_r * x_local + cos_r * y_local

        dx_d = cos_r * dx_local - sin_r * dy_local
        dy_d = sin_r * dx_local + cos_r * dy_local

        ddx_d = cos_r * ddx_local - sin_r * ddy_local
        ddy_d = sin_r * ddx_local + cos_r * ddy_local

        theta_d = math.atan2(dy_d, dx_d)
        v_d     = math.sqrt(dx_d**2 + dy_d**2)

        denom = max(dx_d**2 + dy_d**2, self.V_DENOM_MIN)
        w_d   = (dx_d * ddy_d - dy_d * ddx_d) / denom
        w_d   = max(-self.MAX_W, min(self.MAX_W, w_d))

        return x_d, y_d, theta_d, v_d, w_d

    def control_loop(self):
        if not self.odom_received:
            return

        now_s = self.get_clock().now().nanoseconds / 1e9
        if self.last_odom_time is not None and (now_s - self.last_odom_time) > 2.0:
            return

        t = now_s - self.start_time
        x_d, y_d, theta_d, v_d, w_d = self.generate_desired_trajectory(t)

        dx = x_d - self.current_x
        dy = y_d - self.current_y

        e_x = math.cos(self.current_theta) * dx + math.sin(self.current_theta) * dy
        e_y = -math.sin(self.current_theta) * dx + math.cos(self.current_theta) * dy
        e_theta = self.normalize_angle(theta_d - self.current_theta)

        if abs(e_y) < self.DEADBAND_EY: e_y = 0.0
        if abs(e_theta) < self.DEADBAND_ETHETA: e_theta = 0.0

        s1 = e_x
        s2 = e_theta + self.c * e_y

        sat_s1 = self.sat(s1 / self.phi1)
        sat_s2 = self.sat(s2 / self.phi2)

        v_cmd = v_d * math.cos(e_theta) + self.k1 * e_x + self.Ks1 * sat_s1
        w_cmd = w_d + self.k2 * e_y + self.k3 * math.sin(e_theta) + self.Ks2 * sat_s2

        v_cmd = max(-self.MAX_V, min(self.MAX_V, v_cmd))
        w_cmd = max(-self.MAX_W, min(self.MAX_W, w_cmd))

        err_msg = Point()
        err_msg.x = float(e_x)
        err_msg.y = float(e_y)
        err_msg.z = float(e_theta)
        self.err_pub.publish(err_msg)

        cmd_msg = Twist()
        cmd_msg.linear.x  = float(v_cmd)
        cmd_msg.angular.z = float(w_cmd)
        self.cmd_pub.publish(cmd_msg)

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