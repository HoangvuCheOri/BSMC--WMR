/**
 * @file    esp32_bridge.ino
 * @brief   ESP32 micro-ROS WiFi Bridge + TCP Server cho Hercules
 * @version FINAL-AP: ESP32 tự phát WiFi (AP mode)
 *
 * Topology:
 *   ESP32 = Access Point  → IP cố định 192.168.4.1
 *   PC    = Station       → kết nối vào, nhận IP 192.168.4.2
 *   micro-ROS agent chạy trên PC, lắng nghe cổng AGENT_PORT
 *   ESP32 gửi UDP đến PC (192.168.4.2)
 */

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <string.h>
#include <stdio.h>

// ═══════════════════════════════════════════════════════════════
//   CAU HINH — CHINH SUA TRUOC KHI NAN
// ═══════════════════════════════════════════════════════════════
// ESP32 sẽ phát WiFi với tên và mật khẩu này
#define AP_SSID        "AMR_Robot"
#define AP_PASS        "amr12345"

// IP của PC khi kết nối vào AP (client đầu tiên luôn nhận .2)
#define AGENT_IP       "192.168.4.2"
#define AGENT_PORT     8888

#define HERC_TCP_PORT  5000

#define STM32_SERIAL   Serial2
#define STM32_BAUD     115200
#define STM32_RX_PIN   16
#define STM32_TX_PIN   17

#define DATA_FIELDS         3
#define RECONNECT_MS        2000
#define CMD_VEL_TIMEOUT_MS  500
#define PUB_INTERVAL_MS     50    // 20Hz

#define PING_TIMEOUT_MS     50
#define PING_ATTEMPTS       1
#define MAX_PING_FAIL       10
// ═══════════════════════════════════════════════════════════════

// ─── Custom UDP transport (bypass set_microros_wifi_transports) ─
// set_microros_wifi_transports() gọi WiFi.begin() nội bộ → không dùng được
// trong AP mode. Thay bằng custom transport callbacks dùng WiFiUDP trực tiếp.
WiFiUDP _udp;
IPAddress _agent_ip;
uint16_t  _agent_port;

bool wifi_transport_open(struct uxrCustomTransport *t) {
  _udp.begin(_agent_port);
  return true;
}
bool wifi_transport_close(struct uxrCustomTransport *t) {
  _udp.stop();
  return true;
}
size_t wifi_transport_write(struct uxrCustomTransport *t,
                            const uint8_t *buf, size_t len, uint8_t *err) {
  _udp.beginPacket(_agent_ip, _agent_port);
  size_t sent = _udp.write(buf, len);
  _udp.endPacket();
  return sent;
}
size_t wifi_transport_read(struct uxrCustomTransport *t,
                           uint8_t *buf, size_t len, int timeout_ms, uint8_t *err) {
  unsigned long t0 = millis();
  while (millis() - t0 < (unsigned long)timeout_ms) {
    int pkt = _udp.parsePacket();
    if (pkt > 0) return _udp.read(buf, len);
    delay(1);
  }
  return 0;
}
static void setup_microros_transport() {
  _agent_ip.fromString(AGENT_IP);
  _agent_port = AGENT_PORT;
  rmw_uros_set_custom_transport(
    true, NULL,
    wifi_transport_open, wifi_transport_close,
    wifi_transport_write, wifi_transport_read
  );
}

// ─── FreeRTOS Queue: uart_task → loop() ───────────────────────
typedef struct {
  float v[DATA_FIELDS];
} DataPacket;

QueueHandle_t dataQueue;

// ─── micro-ROS objects ─────────────────────────────────────────
rcl_publisher_t    robot_state_pub;
rcl_publisher_t    wifi_status_pub;
rcl_subscription_t cmd_vel_sub;
rclc_executor_t    executor;
rclc_support_t     support;
rcl_allocator_t    allocator;
rcl_node_t         node;

// ─── Messages ──────────────────────────────────────────────────
std_msgs__msg__Float32MultiArray robot_state_msg;
std_msgs__msg__Int32             wifi_status_msg;
geometry_msgs__msg__Twist        cmd_vel_msg;
float state_data[DATA_FIELDS];

// ─── TCP Server cho Hercules ───────────────────────────────────
WiFiServer hercServer(HERC_TCP_PORT);
WiFiClient hercClient;

// ─── State machine micro-ROS ───────────────────────────────────
typedef enum {
  STATE_WAITING_AGENT,
  STATE_AGENT_AVAILABLE,
  STATE_AGENT_CONNECTED,
  STATE_AGENT_DISCONNECTED
} MicroROSState;

MicroROSState     ros_state   = STATE_WAITING_AGENT;
volatile bool     ros_active  = false;   // volatile: đọc từ nhiều context
int               ping_fail_count = 0;
unsigned long     last_ping_ms    = 0;
unsigned long     last_pub_ms     = 0;
unsigned long     last_cmdvel_ms  = 0;
bool              cmd_vel_watchdog_triggered = false;

TaskHandle_t uartTaskHandle = NULL;

// ───────────────────────────────────────────────────────────────
static void send_to_stm32(const char *s) { STM32_SERIAL.print(s); }

// ───────────────────────────────────────────────────────────────
//  CALLBACK: /cmd_vel — chạy trong loop() khi executor spin
// ───────────────────────────────────────────────────────────────
void cmd_vel_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist *msg =
      (const geometry_msgs__msg__Twist *)msgin;

  char cmd_str[64];
  snprintf(cmd_str, sizeof(cmd_str),
           "CMD,%.4f,%.4f\r\n",
           msg->linear.x, msg->angular.z);

  send_to_stm32(cmd_str);
  last_cmdvel_ms = millis();
  cmd_vel_watchdog_triggered = false;

  if (hercClient && hercClient.connected())
    hercClient.print(cmd_str);
}

// ───────────────────────────────────────────────────────────────
//  WATCHDOG cmd_vel
// ───────────────────────────────────────────────────────────────
static void check_cmdvel_watchdog()
{
  if (!ros_active || last_cmdvel_ms == 0) return;

  if (!cmd_vel_watchdog_triggered &&
      (millis() - last_cmdvel_ms) > CMD_VEL_TIMEOUT_MS)
  {
    send_to_stm32("CMD,0.0000,0.0000\r\n");
    cmd_vel_watchdog_triggered = true;
    Serial.println("[WDG] cmd_vel timeout → STOP");
    if (hercClient && hercClient.connected())
      hercClient.println("[WDG] cmd_vel timeout → STOP");
  }
}

// ───────────────────────────────────────────────────────────────
//  micro-ROS: Tạo entities
// ───────────────────────────────────────────────────────────────
bool create_entities()
{
  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "esp32_bridge", "", &support) != RCL_RET_OK) return false;

  if (rclc_publisher_init_default(
        &robot_state_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/robot_state") != RCL_RET_OK) return false;

  if (rclc_publisher_init_default(
        &wifi_status_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "/wifi_status") != RCL_RET_OK) return false;

  if (rclc_subscription_init_default(
        &cmd_vel_sub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel") != RCL_RET_OK) return false;

  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) return false;

  if (rclc_executor_add_subscription(
        &executor, &cmd_vel_sub, &cmd_vel_msg,
        &cmd_vel_callback, ON_NEW_DATA) != RCL_RET_OK) return false;

  return true;
}

// ───────────────────────────────────────────────────────────────
//  micro-ROS: Hủy entities
// ───────────────────────────────────────────────────────────────
void destroy_entities()
{
  rmw_context_t *rmw_ctx = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_ctx, 0);
  rcl_publisher_fini(&robot_state_pub, &node);
  rcl_publisher_fini(&wifi_status_pub, &node);
  rcl_subscription_fini(&cmd_vel_sub, &node);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
  ros_active = false;
}

// ═══════════════════════════════════════════════════════════════
//  UART TASK — CHỈ đọc Serial + gửi vào Queue
//  KHÔNG gọi bất kỳ hàm micro-ROS nào tại đây
// ═══════════════════════════════════════════════════════════════
void uart_task(void *pv)
{
  char buf[128];
  int  idx = 0;

  for (;;)
  {
    while (STM32_SERIAL.available())
    {
      char c = (char)STM32_SERIAL.read();

      if (c == '\n')
      {
        buf[idx] = '\0';

        if (strncmp(buf, "DATA,", 5) == 0) {
          int v[DATA_FIELDS];
          if (sscanf(buf + 5, "%d,%d,%d", &v[0], &v[1], &v[2]) == DATA_FIELDS) {
            // Sanity check: loại giá trị vô lý do nhiễu motor
            if (abs(v[0]) < 10000 && abs(v[1]) < 10000 && abs(v[2]) < 100000) {
              DataPacket pkt;
              pkt.v[0] = (float)v[0] / 10.0f;
              pkt.v[1] = (float)v[1] / 10.0f;
              pkt.v[2] = (float)v[2] / 1000.0f;
              // Ghi đè: chỉ giữ data mới nhất, không tích lũy backlog
              xQueueOverwrite(dataQueue, &pkt);
            }
          }
        }
        idx = 0;
      }
      else if (c != '\r' && idx < 127)
      {
        buf[idx++] = c;
      }
    }
    vTaskDelay(1);
  }
}

// ═══════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════
void setup()
{
  Serial.begin(115200);
  STM32_SERIAL.begin(STM32_BAUD, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);

  // Queue size=1: chỉ giữ data mới nhất
  dataQueue = xQueueCreate(1, sizeof(DataPacket));

  robot_state_msg.data.data     = state_data;
  robot_state_msg.data.size     = DATA_FIELDS;
  robot_state_msg.data.capacity = DATA_FIELDS;

  // Khởi động AP mode — ESP32 tự phát WiFi
  WiFi.mode(WIFI_AP);
  delay(100);  // Chờ WiFi module sẵn sàng

  // QUAN TRỌNG: softAPConfig phải gọi TRƯỚC softAP
  IPAddress apIP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(apIP, gateway, subnet);

  bool ap_ok = WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 4);
  // channel=1, ssid_hidden=0 (broadcast), max_connection=4
  if (!ap_ok) {
    Serial.println("[AP] FAILED to start! Check SSID/PASS (pass >= 8 chars)");
    while (true) { delay(1000); }  // Dừng lại, không chạy tiếp
  }
  delay(200);  // Chờ AP ổn định

  Serial.printf("[AP] SSID: %s | Pass: %s | IP: %s\n",
                AP_SSID, AP_PASS,
                WiFi.softAPIP().toString().c_str());

  // Chờ PC kết nối vào AP
  Serial.print("[AP] Waiting for PC to connect");
  while (WiFi.softAPgetStationNum() == 0) {
    delay(500);
    Serial.print(".");
  }
  Serial.println(" OK!");

  // Khởi tạo micro-ROS UDP transport (không qua WiFi.begin)
  setup_microros_transport();

  hercServer.begin();
  hercServer.setNoDelay(true);
  Serial.printf("[HERC] TCP server on port %d\n", HERC_TCP_PORT);
  Serial.printf("[ROS]  Agent → %s:%d\n", AGENT_IP, AGENT_PORT);

  // uart_task pinned Core 1 (cùng core với loop() trên Arduino ESP32)
  // FreeRTOS chia thời gian giữa 2 task, Queue đảm bảo an toàn dữ liệu
  xTaskCreatePinnedToCore(
    uart_task, "uartTask",
    4096, NULL, 2,
    &uartTaskHandle, 1
  );
}

// ═══════════════════════════════════════════════════════════════
//  LOOP — Toàn bộ micro-ROS + Hercules TCP ở đây
//  KHÔNG có hàm micro-ROS nào chạy ngoài loop()
// ═══════════════════════════════════════════════════════════════
void loop()
{
  // ─── A. Chấp nhận Hercules TCP client ──────────────────────
  if (hercServer.hasClient()) {
    if (!hercClient || !hercClient.connected()) {
      hercClient = hercServer.available();
      Serial.println("[HERC] Client connected");
      hercClient.println("=== ESP32 Bridge | CMD,v,w / STOP / RESET_ODOM ===");
    } else {
      hercServer.available().stop();
    }
  }

  // ─── B. Watchdog cmd_vel ────────────────────────────────────
  check_cmdvel_watchdog();

  // ─── C. Lấy data từ UART task (non-blocking) ───────────────
  DataPacket pkt;
  if (xQueueReceive(dataQueue, &pkt, 0) == pdTRUE)
  {
    // Mirror sang Hercules
    if (hercClient && hercClient.connected()) {
      char line[64];
      snprintf(line, sizeof(line), "DATA,%d,%d,%d\r\n",
               (int)(pkt.v[0] * 10.0f),
               (int)(pkt.v[1] * 10.0f),
               (int)(pkt.v[2] * 1000.0f));
      hercClient.print(line);
    }

    // Cập nhật state_data để publish (chỉ trong loop())
    if (ros_active && millis() - last_pub_ms >= PUB_INTERVAL_MS) {
      state_data[0] = pkt.v[0];
      state_data[1] = pkt.v[1];
      state_data[2] = pkt.v[2];
      rcl_publish(&robot_state_pub, &robot_state_msg, NULL);
      last_pub_ms = millis();
    }
  }

  // ─── D. Hercules → STM32 ───────────────────────────────────
  if (hercClient && hercClient.connected()) {
    while (hercClient.available())
      STM32_SERIAL.write((char)hercClient.read());
  }


  // ─── E. micro-ROS state machine ────────────────────────────
  // (AP mode: không cần reconnect WiFi, ESP32 luôn là base station)


  switch (ros_state)
  {
    case STATE_WAITING_AGENT:
      if (millis() - last_ping_ms >= 500UL) {
        last_ping_ms = millis();
        if (RMW_RET_OK == rmw_uros_ping_agent(PING_TIMEOUT_MS, PING_ATTEMPTS)) {
          Serial.println("[ROS] Agent found");
          ros_state = STATE_AGENT_AVAILABLE;
        }
      }
      break;

    case STATE_AGENT_AVAILABLE:
      if (create_entities()) {
        Serial.println("[ROS] Connected");
        ros_active = true;
        ros_state  = STATE_AGENT_CONNECTED;
      } else {
        destroy_entities();
        ros_state = STATE_WAITING_AGENT;
      }
      break;

    case STATE_AGENT_CONNECTED:
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));

      if (millis() - last_ping_ms >= 1000UL) {
        last_ping_ms = millis();
        wifi_status_msg.data = WiFi.RSSI();
        rcl_publish(&wifi_status_pub, &wifi_status_msg, NULL);

        if (RMW_RET_OK != rmw_uros_ping_agent(PING_TIMEOUT_MS, PING_ATTEMPTS)) {
          ping_fail_count++;
          Serial.printf("[ROS] Ping fail %d/%d\n", ping_fail_count, MAX_PING_FAIL);
          if (ping_fail_count >= MAX_PING_FAIL) {
            ping_fail_count = 0;
            Serial.println("[ROS] Agent lost — retrying...");
            destroy_entities();
            ros_state = STATE_AGENT_DISCONNECTED;
          }
        } else {
          ping_fail_count = 0;
        }
      }
      break;

    case STATE_AGENT_DISCONNECTED:
      if (millis() - last_ping_ms >= RECONNECT_MS) {
        last_ping_ms = millis();
        ros_state = STATE_WAITING_AGENT;
      }
      break;

    default:
      ros_state = STATE_WAITING_AGENT;
      break;
  }

  delay(1);
}
