# DA1_BSMC_WMR
Đồ án nghiên cứu thiết kế bộ điều khiển robot di động vi sai 2 bánh (Wheeled Mobile Robot - WMR) sử dụng thuật toán Backstepping Sliding Mode Control (BSMC)
# Tổng quan hệ thống
Hệ thống gồm 3 tầng chính:
## Tầng điều khiển cấp cao (High-Level Control)

- Máy tính nhúng sử dụng ROS2 để thực hiện bài toán điều khiển bám quỹ đạo cho robot bằng thuật toán Backstepping Sliding Mode Control (BSMC).
- Sử dụng bộ lọc EKF (Extended Kalman Filter) để kết hợp dữ liệu Encoder và IMU nhằm tạo Odometry ổn định và giảm nhiễu.
- Dashboard được sử dụng để hiển thị quỹ đạo di chuyển, vận tốc và sai số bám quỹ đạo của robot theo thời gian thực.
---
## Tầng truyền thông không dây (Wireless Communication Layer)
- ESP32 được sử dụng làm cầu nối truyền thông không dây giữa ROS2 và STM32 bằng giao thức ESP-NOW.
- ESP32 phía máy tính nhận dữ liệu điều khiển từ ROS2 qua USB Serial và truyền không dây sang robot.
- ESP32 phía robot nhận dữ liệu điều khiển và gửi xuống STM32 qua UART.
- Dữ liệu trạng thái của robot được gửi ngược từ STM32 về ROS2 thông qua ESP32.
- Hệ thống giúp loại bỏ dây nối trực tiếp giữa robot và máy tính điều khiển.
---
## Tầng điều khiển cấp thấp (Low-Level Control)

- STM32 đảm nhiệm điều khiển động cơ DC bằng PID.
- Đọc Encoder để tính tốc độ bánh xe.
- Đọc IMU/Gyro để xác định vận tốc góc của robot.
- Nhận lệnh vận tốc từ ROS2 thông qua ESP32.
- Gửi dữ liệu trạng thái robot về ROS2.

---

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

### Quỹ đạo đường thẳng

```bash
ros2 run amr_control bsmc_controller
```

#### Quỹ đạo tròn

```bash
ros2 run amr_control bsmc_circle
```

#### Quỹ đạo số 8

```bash
ros2 run amr_control bsmc_eight
```

---

#### Chạy Dashboard

```bash
python3 dashboard.py
```
