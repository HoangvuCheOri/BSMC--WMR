#include <esp_now.h>
#include <WiFi.h>
#include <string.h>

uint8_t ROVER_MAC[] = {0xB4, 0xBF, 0xE9, 0x14, 0x21, 0xA4}; 

#define USB_BAUD    115200

typedef struct __attribute__((packed)) {
  uint8_t  type;        
  char     payload[72];
} EspNowPacket;

EspNowPacket tx_pkt;
esp_now_peer_info_t peer_info;

// Buffer đọc USB Serial (CMD từ ROS2)
char usb_buf[80];
int  usb_idx = 0;

volatile bool peer_added = false;

void on_data_recv(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
  if (len < (int)sizeof(EspNowPacket)) return;
  const EspNowPacket *pkt = (const EspNowPacket *)data;

  if (pkt->type == 'D') {
    Serial.print(pkt->payload);
    Serial.print("\r\n");
  }
}

void on_data_sent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {

}

void setup() {
  Serial.begin(USB_BAUD);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100); 
  Serial.print("[BASE] My MAC: ");
  Serial.println(WiFi.macAddress());

  bool mac_valid = false;
  for (int i = 0; i < 6; i++) {
    if (ROVER_MAC[i] != 0xFF) { mac_valid = true; break; }
  }
  if (!mac_valid) {
    Serial.println("[WARNING] ROVER_MAC chua duoc cau hinh. Chi co the nhan DATA, khong gui CMD duoc.");
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ERROR] ESP-NOW init FAIL");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(on_data_recv);
  esp_now_register_send_cb(on_data_sent);

  if (mac_valid) {
    memset(&peer_info, 0, sizeof(peer_info));
    memcpy(peer_info.peer_addr, ROVER_MAC, 6);
    peer_info.channel = 0;
    peer_info.encrypt = false;

    if (esp_now_add_peer(&peer_info) == ESP_OK) {
      peer_added = true;
      Serial.println("[BASE] ESP-NOW peer (Rover) added OK");
    } else {
      Serial.println("[ERROR] ESP-NOW add peer FAIL");
    }
  }

  Serial.println("[BASE] Ready — forwarding ESP-NOW <-> USB Serial");
}
void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\n') {
      usb_buf[usb_idx] = '\0';
      if (strncmp(usb_buf, "CMD,", 4) == 0 && peer_added) {
        tx_pkt.type = 'C';
        strncpy(tx_pkt.payload, usb_buf, sizeof(tx_pkt.payload) - 1);
        tx_pkt.payload[sizeof(tx_pkt.payload) - 1] = '\0';

        esp_now_send(ROVER_MAC, (uint8_t *)&tx_pkt, sizeof(EspNowPacket));
      }
      else if ((strncmp(usb_buf, "STOP", 4) == 0 ||
                strncmp(usb_buf, "RESET_ODOM", 10) == 0) && peer_added) {
        tx_pkt.type = 'C';
        strncpy(tx_pkt.payload, usb_buf, sizeof(tx_pkt.payload) - 1);
        tx_pkt.payload[sizeof(tx_pkt.payload) - 1] = '\0';
        esp_now_send(ROVER_MAC, (uint8_t *)&tx_pkt, sizeof(EspNowPacket));
      }

      usb_idx = 0;

    } else if (c != '\r' && usb_idx < 79) {
      usb_buf[usb_idx++] = c;
    }
  }
}
