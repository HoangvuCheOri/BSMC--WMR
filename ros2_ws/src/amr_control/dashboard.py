import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading
import math

class DashboardNode(Node):
    def __init__(self):
        super().__init__('dashboard_node')
        self.sub_odom = self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)
        self.sub_state = self.create_subscription(Float32MultiArray, '/robot_state', self.state_cb, 10)
        self.sub_err = self.create_subscription(Point, '/tracking_error', self.err_cb, 10)
        
        self.x_hist = deque(maxlen=50000)
        self.y_hist = deque(maxlen=50000)
        self.xd_hist = deque(maxlen=50000)
        self.yd_hist = deque(maxlen=50000)
        
        self.time_hist = deque(maxlen=10000)
        self.rpmL_hist = deque(maxlen=10000)
        self.rpmR_hist = deque(maxlen=10000)
        
        self.err_time_hist = deque(maxlen=10000)
        self.ex_hist = deque(maxlen=10000)
        self.ey_hist = deque(maxlen=10000)
        
        self.start_time = None
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False
        
    def odom_cb(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.x_hist.append(self.current_x)
        self.y_hist.append(self.current_y)
        self.odom_received = True
        
    def state_cb(self, msg):
        if self.start_time is None:
            self.start_time = self.get_clock().now().nanoseconds / 1e9
        
        t = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        self.time_hist.append(t)
        self.rpmL_hist.append(msg.data[0])
        self.rpmR_hist.append(msg.data[1])

    def err_cb(self, msg):
        if self.start_time is None:
            return
        t = self.get_clock().now().nanoseconds / 1e9 - self.start_time
        self.err_time_hist.append(t)
        self.ex_hist.append(msg.x)
        self.ey_hist.append(msg.y)
        
        # Tái tạo lại tọa độ desired từ sai số và pose hiện tại
        if self.odom_received:
            dx = msg.x * math.cos(self.current_yaw) - msg.y * math.sin(self.current_yaw)
            dy = msg.x * math.sin(self.current_yaw) + msg.y * math.cos(self.current_yaw)
            self.xd_hist.append(self.current_x + dx)
            self.yd_hist.append(self.current_y + dy)

node = None

def spin_thread():
    rclpy.spin(node)

def main():
    global node
    rclpy.init()
    node = DashboardNode()
    
    t = threading.Thread(target=spin_thread, daemon=True)
    t.start()
    
    # Thiết lập giao diện biểu đồ
    fig = plt.figure(figsize=(12, 8))
    try:
        plt.style.use('seaborn-v0_8-darkgrid')
    except OSError:
        try:
            plt.style.use('seaborn-darkgrid')
        except OSError:
            plt.style.use('ggplot')
    
    # 1. Biểu đồ Quỹ đạo (X-Y)
    ax1 = fig.add_subplot(2, 2, 1)
    ax1.set_title("Trajectory (X - Y)")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    line_traj_d, = ax1.plot([], [], 'r--', linewidth=2, label='Desired Trajectory', alpha=0.7)
    line_traj, = ax1.plot([], [], 'b-', linewidth=2, label='Actual Trajectory')
    ax1.legend()
    # Dùng adjustable='datalim' kết hợp với autoscale_view để lấp đầy toàn bộ khoảng trắng 2 bên
    ax1.set_aspect('equal', adjustable='datalim')
    
    # 2. Biểu đồ Sai số (e_x, e_y)
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.set_title("Tracking Errors (ex, ey)")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Meters")
    line_ex, = ax2.plot([], [], 'g-', label='e_x (Longitudinal)')
    line_ey, = ax2.plot([], [], 'm-', label='e_y (Lateral)')
    ax2.axhline(0, color='k', linestyle='--', alpha=0.5)
    ax2.legend()
    
    # 3. Biểu đồ RPM 2 bánh
    ax3 = fig.add_subplot(2, 1, 2)
    ax3.set_title("Wheel Velocities (RPM)")
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("RPM")
    line_rpmL, = ax3.plot([], [], 'r-', label='Left RPM', linewidth=1.5)
    line_rpmR, = ax3.plot([], [], 'b-', label='Right RPM', linewidth=1.5, alpha=0.7)
    ax3.legend()
    
    def update(frame):
        # Update Trajectory
        if len(node.x_hist) > 0:
            line_traj.set_data(node.x_hist, node.y_hist)
            if len(node.xd_hist) > 0:
                line_traj_d.set_data(node.xd_hist, node.yd_hist)
                
            all_x = list(node.x_hist) + list(node.xd_hist)
            all_y = list(node.y_hist) + list(node.yd_hist)
            
            if all_x and all_y:
                min_x, max_x = min(all_x), max(all_x)
                min_y, max_y = min(all_y), max(all_y)
                # Để lấp đầy khoảng trắng và làm đồ thị to nhất có thể, ta phó thác cho Matplotlib tự tính Limits
                if ax1.get_navigate_mode() is None:
                    # Reset data limits và đưa bounding box của trajectory vào
                    ax1.ignore_existing_data_limits = True
                    ax1.update_datalim([[min_x, min_y], [max_x, max_y]])
                    # Tự động scale khung nhìn (nó sẽ tự động mở rộng trục X để lấp đầy khoảng trắng 2 bên)
                    ax1.autoscale_view()
            
        # Update Errors
        if len(node.err_time_hist) > 0:
            line_ex.set_data(node.err_time_hist, node.ex_hist)
            line_ey.set_data(node.err_time_hist, node.ey_hist)
            ax2.set_xlim(0, max(10, node.err_time_hist[-1] + 1))
            
            all_errs = list(node.ex_hist) + list(node.ey_hist)
            if all_errs:
                margin = max(abs(max(all_errs)), abs(min(all_errs))) + 0.05
                ax2.set_ylim(-margin, margin)

        # Update RPM
        if len(node.time_hist) > 0:
            line_rpmL.set_data(node.time_hist, node.rpmL_hist)
            line_rpmR.set_data(node.time_hist, node.rpmR_hist)
            ax3.set_xlim(0, max(10, node.time_hist[-1] + 1))
            
            all_rpms = list(node.rpmL_hist) + list(node.rpmR_hist)
            if all_rpms:
                ax3.set_ylim(min(all_rpms) - 10, max(all_rpms) + 10)
                
        return line_traj, line_traj_d, line_ex, line_ey, line_rpmL, line_rpmR
        
    ani = animation.FuncAnimation(fig, update, interval=100, cache_frame_data=False)
    plt.tight_layout()
    plt.show()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
