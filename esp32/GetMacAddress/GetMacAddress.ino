#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(1000);  // Chờ Serial ổn định

  WiFi.mode(WIFI_STA);
  delay(500);   // Chờ WiFi module khởi tạo xong

  Serial.println("=== ESP32 MAC Address ===");
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.println();
  Serial.println("Dán MAC này vào:");
  Serial.println("  - BASE_MAC[]  trong esp32_rover.ino  (nếu đây là ESP32-USB/Base)");
  Serial.println("  - ROVER_MAC[] trong esp32_base.ino   (nếu đây là ESP32-Rover)");
  Serial.println();

  // In dạng array C
  uint8_t mac[6];
  WiFi.macAddress(mac);
  Serial.print("Dạng C array: {");
  for (int i = 0; i < 6; i++) {
    Serial.printf("0x%02X", mac[i]);
    if (i < 5) Serial.print(", ");
  }
  Serial.println("}");
}

void loop() {
  delay(5000);
  Serial.print("MAC: ");
  Serial.println(WiFi.macAddress());
}
