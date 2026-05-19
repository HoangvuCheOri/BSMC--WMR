#!/usr/bin/env python3
import os
# Đặt cấu hình FFMPEG TRƯỚC KHI import cv2 để đảm bảo triệt tiêu lag RTSP
os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp|fflags;nobuffer|flags;low_delay"

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
import cv2
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
import time
import os
import threading
import queue
import pupil_apriltags as apriltag
import yaml

class Kalman1D:
    def __init__(self, Q=1e-2, R=1e-5, P=1.0, x0=0.0):
        self.Q, self.R, self.P, self.x = Q, R, P, x0

    def update(self, measurement):
        self.P += self.Q
        K = self.P / (self.P + self.R)
        self.x += K * (measurement - self.x)
        self.P *= (1 - K)
        return self.x

def normalize_angle_deg(angle):
    """Đưa góc về [-180, 180]"""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

def angle_diff_deg(a, b):
    """Tính a - b nhưng kết quả nằm trong [-180, 180]"""
    d = a - b
    while d < -180: d += 360
    while d > 180: d -= 360
    return d

def euler_from_quaternion(x, y, z, w):
    r, p, y_euler = R.from_quat([x, y, z, w]).as_euler('xyz', degrees=False)
    return r, p, y_euler

class CameraPoseEstimator(Node):
    def __init__(self):
        super().__init__('pose_estimation_publisher')
        
        # Publisher chuyển sang hệ Odometry để EKF có thể đọc được
        self.pose_pub = self.create_publisher(Odometry, '/odom_camera', 10)
        
        # Đổi đường dẫn yaml mặc định về thư mục hiện tại để tránh lỗi hardcode cũ
        self.declare_parameter('yaml_path', 'position.yaml')
        self.yaml_path = self.get_parameter('yaml_path').value

        self.ip_url = f"rtsp://{os.getenv('CAMERA_USERNAME', 'admin')}:{os.getenv('CAMERA_PASSWORD', 'lab208b3')}@" \
                 f"{os.getenv('CAMERA_IP', '192.168.100.56')}:{os.getenv('CAMERA_PORT', '554')}/cam/realmonitor?channel=1&subtype=1"

        # Calibration & detector (Đã calibrate ở độ phân giải 1280x720)
        self.base_camera_matrix = np.array([[767.6786, 0., 637.4356],
                                  [0., 765.5082, 357.2588],
                                  [0., 0., 1.]], dtype=np.float32)
        self.dist_coeffs = np.array([-0.2374, 0.0734, 0.00345, -0.00824, -0.0514], dtype=np.float32)
        self.detector = apriltag.Detector(families='tag36h11', nthreads=3, refine_edges=1)

        # Sửa lại kích thước tag cho khớp thực tế (15cm = 0.15m)
        self.marker_size = 0.150
        self.marker_id = 0
        self.marker_3D = np.array([
            [-self.marker_size/2, self.marker_size/2, 0],
            [self.marker_size/2, self.marker_size/2, 0],
            [self.marker_size/2, -self.marker_size/2, 0],
            [-self.marker_size/2, -self.marker_size/2, 0]
        ], dtype=np.float32)
        self.Rz_90 = np.array([[0, 1, 0], [-1, 0, 0], [0, 0, 1]], dtype=np.float32)

        self.kalman_x = Kalman1D(Q=0.005, R=0.01)
        self.kalman_y = Kalman1D(Q=0.005, R=0.01)
        self.kalman_yaw = Kalman1D(Q=0.5, R=2.0)

        self.pose = {'x': None, 'y': None, 'yaw': None}
        self.pose_saved = False

        self.q = queue.Queue(maxsize=1)
        
        # Tạo cửa sổ OpenCV cho phép phóng to/thu nhỏ (WINDOW_NORMAL)
        cv2.namedWindow("AprilTag Navigation Monitor", cv2.WINDOW_NORMAL)

        # Bắt đầu luồng đọc camera
        self.camera_thread = threading.Thread(target=self.camera_stream_thread, daemon=True)
        self.camera_thread.start()

        # Timer loop (thay thế cho while loop & rate.sleep() của ROS 1)
        timer_period = 1.0 / 30.0  # 30 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def camera_stream_thread(self):
        cap = None
        while rclpy.ok():
            if cap is None or not cap.isOpened():
                cap = cv2.VideoCapture(self.ip_url, cv2.CAP_FFMPEG)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                if not cap.isOpened():
                    self.get_logger().warn("Cannot connect to camera, retrying...")
                    time.sleep(1)
                    continue

            ret, frame = cap.read()
            if not ret:
                cap.release()
                cap = None
                continue

            if not self.q.full():
                self.q.put_nowait(frame)
            else:
                try:
                    self.q.get_nowait()
                except queue.Empty:
                    pass
                self.q.put_nowait(frame)
        if cap: cap.release()

    def write_pose_to_yaml(self):
        clean_data = {k: float(v) for k, v in self.pose.items() if k in ['x', 'y', 'yaw']}
        try:
            with open(self.yaml_path, "w") as f:
                yaml.dump(clean_data, f)
            self.get_logger().info(f"Initial position saved to {self.yaml_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to save pose to yaml: {e}")

    def timer_callback(self):
        try:
            # Lấy frame mới nhất, nếu không có thì bỏ qua lượt này
            frame = self.q.get(timeout=0.05)
        except queue.Empty:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Tự động scale camera_matrix theo độ phân giải thực tế của luồng video (VD: 640x480)
        # để tọa độ mét không bị sai lệch khi dùng subtype=1
        h, w = gray.shape
        scale_x = w / 1280.0
        scale_y = h / 720.0
        camera_matrix = self.base_camera_matrix.copy()
        camera_matrix[0, 0] *= scale_x
        camera_matrix[1, 1] *= scale_y
        camera_matrix[0, 2] *= scale_x
        camera_matrix[1, 2] *= scale_y

        detections = self.detector.detect(gray)

        for det in detections:
            if det.tag_id != self.marker_id:
                continue

            img_pts = det.corners.astype(np.float32)
            success, rvec, tvec = cv2.solvePnP(self.marker_3D, img_pts, camera_matrix, self.dist_coeffs,
                                               flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not success:
                continue

            R_orig, _ = cv2.Rodrigues(rvec)
            R_corr = R_orig @ self.Rz_90
            quat = R.from_matrix(R_corr).as_quat()
            _, _, yaw = euler_from_quaternion(*quat)
            yaw_deg = normalize_angle_deg(math.degrees(-yaw - math.pi / 2))

            # Kalman filtering
            self.pose['x'] = self.kalman_x.update(tvec[0][0])
            self.pose['y'] = self.kalman_y.update(-tvec[1][0])

            # Wrap-around fix for yaw
            if 'yaw_unwrapped' not in self.pose:
                self.pose['yaw_unwrapped'] = yaw_deg
                self.pose['yaw'] = yaw_deg
            else:
                diff = angle_diff_deg(yaw_deg, self.pose['yaw'])
                self.pose['yaw_unwrapped'] += diff

                filtered_unwrapped = self.kalman_yaw.update(self.pose['yaw_unwrapped'])
                self.pose['yaw'] = normalize_angle_deg(filtered_unwrapped)

            # Lưu vị trí ban đầu
            if not self.pose_saved:
                self.write_pose_to_yaml()
                self.pose_saved = True

            # Publish dữ liệu chuẩn Odometry cho EKF
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'

            # Tọa độ X, Y
            odom_msg.pose.pose.position.x = float(self.pose['x'])
            odom_msg.pose.pose.position.y = float(self.pose['y'])
            odom_msg.pose.pose.position.z = 0.0

            # Chuyển đổi Yaw sang Quaternion cho Orientation
            quat = R.from_euler('z', self.pose['yaw'], degrees=True).as_quat()
            odom_msg.pose.pose.orientation.x = quat[0]
            odom_msg.pose.pose.orientation.y = quat[1]
            odom_msg.pose.pose.orientation.z = quat[2]
            odom_msg.pose.pose.orientation.w = quat[3]

            # Ma trận hiệp phương sai (Covariance) - Cho EKF biết độ tin cậy của Camera
            # Để giá trị nhỏ (0.01) để EKF tin camera hơn encoder về vị trí
            P = odom_msg.pose.covariance
            P[0] = 0.01  # x
            P[7] = 0.01  # y
            P[35] = 0.05 # yaw

            self.pose_pub.publish(odom_msg)

            # --- THÊM PHẦN VISUALIZATION ---
            # 1. Vẽ khung vuông bao quanh AprilTag (màu xanh lá) - Tăng độ đậm lên 4
            pts = img_pts.astype(int)
            for i in range(4):
                cv2.line(frame, tuple(pts[i]), tuple(pts[(i+1)%4]), (0, 255, 0), 4)
            
            # Ghi ID của Tag
            cv2.putText(frame, f"AprilTag ID: {det.tag_id}", tuple(pts[3] + [0, 25]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # 2. Vẽ trục tọa độ XYZ lên tâm của Tag (Trục X đỏ, Y xanh lá, Z xanh dương)
            # Tăng chiều dài trục lên 0.2m (20cm) và độ dày (thickness) lên 5
            cv2.drawFrameAxes(frame, camera_matrix, self.dist_coeffs, rvec, tvec, 0.2, 5)

            # 3. Vẽ khung nền đen mờ và ghi text tọa độ lên góc trái màn hình
            overlay = frame.copy()
            # Tăng chiều rộng của khung đen lên 400 để chữ không bị tràn ra ngoài
            cv2.rectangle(overlay, (10, 10), (420, 160), (0, 0, 0), -1)
            frame = cv2.addWeighted(overlay, 0.5, frame, 0.5, 0) # Làm mờ nền đen 50%
            
            cv2.putText(frame, f"X: {self.pose['x']:.3f} m", (20, 50), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (255, 255, 255), 2)
            cv2.putText(frame, f"Y: {self.pose['y']:.3f} m", (20, 95), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (255, 255, 255), 2)
            cv2.putText(frame, f"Yaw: {self.pose['yaw']:.2f} deg", (20, 140), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 255, 0), 2)

            break

        # Show cửa sổ hình ảnh (Hiển thị liên tục kể cả khi không thấy tag)
        cv2.imshow("AprilTag Navigation Monitor", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = CameraPoseEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
