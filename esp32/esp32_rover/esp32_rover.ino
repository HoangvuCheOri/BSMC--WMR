/**
 * @file    esp32_rover.ino
 * @brief   ESP32 trên xe — ESP-NOW Rover Node
 * @version 1.0
 *
 * Nhiệm vụ:
 *   1. Nhận DATA từ STM32 qua UART → gửi qua ESP-NOW → Base Station (laptop)
 *   2. Nhận CMD từ ESP-NOW ← Base Station → forward xuống STM32 qua UART
 *
 * Kết nối phần cứng:
 *   ESP32 RX2 (GPIO16) ← STM32 TX (USART2)
 *   ESP32 TX2 (GPIO17) → STM32 RX (USART2)
 *
 * QUAN TRỌNG: Trước khi nạp, sửa BASE_MAC bằng MAC của ESP32-USB
 *   Cách lấy MAC: nạp sketch GetMacAddress.ino vào ESP32-USB, đọc Serial Monitor
 */

#include <esp_now.h>
#include <WiFi.h>
#include <string.h>

// ═══════════════════════════════════════════════════════════════
//  CẤU HÌNH — SỬA TRƯỚC KHI NẠP
// ═══════════════════════════════════════════════════════════════
// MAC address của ESP32-USB (base station cắm laptop)
// Ví dụ: {0x24, 0x6F, 0x28, 0xAB, 0xCD, 0xEF}
uint8_t BASE_MAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}; // ← SỬA Ở ĐÂY

#define STM32_SERIAL   Serial2
#define STM32_BAUD     115200
#define STM32_RX_PIN   16
#define STM32_TX_PIN   17

#define MAX_PKT_LEN    80
// ═══════════════════════════════════════════════════════════════

// ─── ESP-NOW Packet ────────────────────────────────────────────
// Dùng struct cố định để tránh lỗi alignment
typedef struct __attribute__((packed)) {
  uint8_t  type;         // 'D' = DATA, 'C' = CMD
  char     payload[72];  // nội dung raw string
} EspNowPacket;

EspNowPacket tx_pkt;
EspNowPacket rx_pkt;

esp_now_peer_info_t peer_info;

// ─── UART buffer ──────────────────────────────────────────────
char uart_buf[MAX_PKT_LEN];
int  uart_idx = 0;

// ─── Trạng thái kết nối ESP-NOW ───────────────────────────────
volatile bool peer_added = false;

// ───────────────────────────────────────────────────────────────
//  Callback: Gửi ESP-NOW xong
// ───────────────────────────────────────────────────────────────
// ESP32 Arduino Core 3.x: send callback dùng wifi_tx_info_t thay vì uint8_t *mac
void on_data_sent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  // Có thể log nếu cần debug
  // Serial.printf("[ESPNOW] Send %s\n", status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

// ───────────────────────────────────────────────────────────────
//  Callback: Nhận ESP-NOW (CMD từ Base)
//  *** Chạy trong ISR context — chỉ làm việc nhẹ ***
//  *** ESP32 Arduino Core 3.x: dùng esp_now_recv_info_t ***
// ───────────────────────────────────────────────────────────────
void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  if (len < (int)sizeof(EspNowPacket)) return;
  const EspNowPacket *pkt = (const EspNowPacket *)data;

  if (pkt->type == 'C') {
    // Forward CMD xuống STM32 ngay lập tức
    STM32_SERIAL.print(pkt->payload);
    STM32_SERIAL.print("\r\n");
  }
}

// ═══════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(115200);
  STM32_SERIAL.begin(STM32_BAUD, SERIAL_8N1, STM32_RX_PIN, STM32_TX_PIN);

  // ESP-NOW yêu cầu WiFi mode = STA (không cần connect vào AP)
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100); // FIX: Đợi WiFi khởi tạo xong

  Serial.print("[ROVER] MAC: ");
  Serial.println(WiFi.macAddress());

  // Kiểm tra BASE_MAC đã được cấu hình chưa
  bool mac_valid = false;
  for (int i = 0; i < 6; i++) {
    if (BASE_MAC[i] != 0xFF) { mac_valid = true; break; }
  }
  if (!mac_valid) {
    Serial.println("[ERROR] BASE_MAC chua duoc cau hinh! Sua BASE_MAC trong code.");
    while (true) delay(1000);
  }

  // Khởi tạo ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW init FAIL");
    while (true) delay(1000);
  }

  esp_now_register_send_cb(on_data_sent);
  esp_now_register_recv_cb(on_data_recv);

  // Thêm peer (Base Station)
  memset(&peer_info, 0, sizeof(peer_info));
  memcpy(peer_info.peer_addr, BASE_MAC, 6);
  peer_info.channel = 0;   // 0 = dùng channel hiện tại
  peer_info.encrypt = false;

  if (esp_now_add_peer(&peer_info) == ESP_OK) {
    peer_added = true;
    Serial.println("[ROVER] ESP-NOW peer added OK");
  } else {
    Serial.println("[ERROR] ESP-NOW add peer FAIL");
  }

  Serial.println("[ROVER] Ready — waiting for STM32 DATA...");
}

// ═══════════════════════════════════════════════════════════════
//  LOOP — Đọc UART từ STM32, gửi ESP-NOW khi có DATA
// ═══════════════════════════════════════════════════════════════
void loop() {
  while (STM32_SERIAL.available()) {
    char c = (char)STM32_SERIAL.read();

    if (c == '\n') {
      uart_buf[uart_idx] = '\0';

      // Chỉ forward "DATA,..." — bỏ qua các message khác (MPU OK, SYSTEM READY,...)
      if (strncmp(uart_buf, "DATA,", 5) == 0 && peer_added) {
        tx_pkt.type = 'D';
        // Copy an toàn vào payload
        strncpy(tx_pkt.payload, uart_buf, sizeof(tx_pkt.payload) - 1);
        tx_pkt.payload[sizeof(tx_pkt.payload) - 1] = '\0';

        esp_now_send(BASE_MAC, (uint8_t *)&tx_pkt, sizeof(EspNowPacket));
      }
      uart_idx = 0;

    } else if (c != '\r' && uart_idx < MAX_PKT_LEN - 1) {
      uart_buf[uart_idx++] = c;
    }
  }
}
