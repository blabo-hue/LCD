#include "LCDNow.h"
#include <esp_now.h>
#include <WiFi.h>

uint8_t baseMAC[] = {0xB0, 0xCB, 0xD8, 0xE3, 0x0E, 0x80}; // MAC address of the base station, change this to match your base station's MAC address

typedef struct struct_message {
  int x;
  int y;
} struct_message;

struct_message data;

void onSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sent OK" : "Send Fail");
}

void onReceive(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  memcpy(&data, incomingData, sizeof(data));
  Serial.print("From base: ");
  Serial.print(data.x);
  Serial.print(",");
  Serial.println(data.y);
}


void initLCDNow() {

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onReceive);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, baseMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  esp_now_add_peer(&peerInfo);

}

void LCDNow_send(uint16_t x, uint16_t y) {

  data.x = x;
  data.y = y;
  esp_now_send(baseMAC, (uint8_t *)&data, sizeof(data));
}
