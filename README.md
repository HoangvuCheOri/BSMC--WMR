# DA1_BSMC_WMR
Đồ án nghiên cứu thiết kế bộ điều khiển robot di động vi sai 2 bánh (Wheeled Mobile Robot - WMR) sử dụng thuật toán Backstepping Sliding Mode Control (BSMC)
# Tổng quan hệ thống

Hệ thống gồm 2 tầng chính:
Tầng điều khiển cấp thấp (Low-Level Control):
  - STM32 điều khiển động cơ bằng PID.
  - Đọc Encoder và IMU/Gyro.
  - Giao tiếp UART với ROS2.

Tầng điều khiển cấp cao (High-Level Control)
- Máy tính nhúng sử dụng ROS2 để thực hiện bài toán điều khiển bám quỹ đạo cho robot bằng thuật toán Backstepping Sliding Mode Control (BSMC).
- Sử dụng bộ lọc EKF (Extended Kalman Filter) để kết hợp dữ liệu Encoder và IMU nhằm tạo Odometry ổn định và giảm nhiễu.
- Dashboard được sử dụng để hiển thị quỹ đạo di chuyển, vận tốc và sai số bám quỹ đạo của robot theo thời gian thực.
# Các câu lệnh

## 1. Build ROS2 Workspace

```bash
cd ~/DA1_BSMC_WMR/ros2_ws

colcon build

source install/setup.bash
```

---

## 2. Chạy UART Bridge với STM32

```bash
ros2 run amr_control robot_serial_bridge \
--ros-args \
-p port:=/dev/ttyUSB0 \
-p baud:=115200
```

Kiểm tra cổng Serial:

```bash
ls /dev/ttyUSB*
ls /dev/ttyACM*
```

---

## 3. Chạy State Bridge

```bash
ros2 run amr_control state_bridge
```

---

## 4. Chạy EKF

```bash
ros2 run robot_localization ekf_node \
--ros-args \
--params-file src/amr_control/config/ekf.yaml
```

---

# Chạy bộ điều khiển BSMC

## Quỹ đạo đường thẳng

```bash
ros2 run amr_control bsmc_controller
```

## Quỹ đạo tròn

```bash
ros2 run amr_control bsmc_circle
```

## Quỹ đạo số 8

```bash
ros2 run amr_control bsmc_eight
```

---

# Chạy Dashboard

```bash
python3 dashboard.py
```

---

# Kiểm tra Topic ROS2

## Danh sách topic

```bash
ros2 topic list
```

## Dữ liệu trạng thái robot

```bash
ros2 topic echo /robot_state
```

## Odometry thô

```bash
ros2 topic echo /odom_raw
```

## Odometry sau EKF

```bash
ros2 topic echo /odometry/filtered
```

## Lệnh vận tốc

```bash
ros2 topic echo /cmd_vel
```

## Sai số bám quỹ đạo

```bash
ros2 topic echo /tracking_error
```
