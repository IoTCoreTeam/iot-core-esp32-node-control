#include <WiFi.h>
#include <esp_now.h>
#include "esp_wifi.h"

#ifndef NODE_ID
#define NODE_ID "node-control-001"
#endif

#ifndef DEVICE_ID
#define DEVICE_ID "actuator-control-001"
#endif

// Update MAC to your gateway MAC address.
#ifndef GATEWAY_MAC_0
#define GATEWAY_MAC_0 0x00
#endif
#ifndef GATEWAY_MAC_1
#define GATEWAY_MAC_1 0x70
#endif
#ifndef GATEWAY_MAC_2
#define GATEWAY_MAC_2 0x07
#endif
#ifndef GATEWAY_MAC_3
#define GATEWAY_MAC_3 0x7E
#endif
#ifndef GATEWAY_MAC_4
#define GATEWAY_MAC_4 0x7D
#endif
#ifndef GATEWAY_MAC_5
#define GATEWAY_MAC_5 0x3C
#endif

// Must match gateway channel.
#if defined(WIFI_PROFILE_HOME)
#define WIFI_SSID "Tien Thuat"
#define WIFI_PASSWORD "07112004tien"
#elif defined(WIFI_PROFILE_OFFICE)
#define WIFI_SSID "OrsCorp"
#define WIFI_PASSWORD "Tamchiduc68"
#endif

#ifndef WIFI_SSID
#define WIFI_SSID "OrsCorp"
#endif
#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "Tamchiduc68"
#endif

// Relay pins: GPIO23 -> Pump, GPIO22 -> Light.
const int PUMP_RELAY_PIN = 23;
const int LIGHT_RELAY_PIN = 22;
const bool RELAY_ACTIVE_LOW = true;

const uint8_t MSG_TYPE_HEARTBEAT = 2;
const uint32_t HEARTBEAT_INTERVAL_MS = 5000;

uint8_t gatewayAddress[] = {
  GATEWAY_MAC_0,
  GATEWAY_MAC_1,
  GATEWAY_MAC_2,
  GATEWAY_MAC_3,
  GATEWAY_MAC_4,
  GATEWAY_MAC_5
};

typedef struct struct_message {
  char device_id[32];
  char node_id[32];
  float temperature;
  float humidity;
  int light_raw;
  float light_percent;
  int rain_raw;
  float rain_percent;
  int soil_raw;
  float soil_percent;
  unsigned long sensor_timestamp;
  int rssi;
  bool dht_error;
  uint8_t message_type;
  uint32_t uptime_sec;
  uint32_t heartbeat_seq;
} struct_message;

typedef struct control_command_message {
  char gateway_id[32];
  char node_id[32];
  char action_type[24];
  char device[16];
  char state[8];
  uint32_t command_seq;
} control_command_message;

struct_message heartbeatData;
uint32_t heartbeatSeq = 0;

uint8_t resolveWifiChannel() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 8000) {
    delay(200);
  }
  uint8_t channel = WiFi.channel();
  if (WiFi.status() != WL_CONNECTED || channel == 0) {
    int n = WiFi.scanNetworks(false, true);
    for (int i = 0; i < n; i++) {
      if (WiFi.SSID(i) == WIFI_SSID) {
        channel = WiFi.channel(i);
        break;
      }
    }
  }
  if (channel == 0) {
    channel = 1;
  }
  WiFi.disconnect(true);
  delay(100);
  return channel;
}

void setRelay(int pin, bool on) {
  if (RELAY_ACTIVE_LOW) {
    digitalWrite(pin, on ? LOW : HIGH);
  } else {
    digitalWrite(pin, on ? HIGH : LOW);
  }
}

void applyCommand(const control_command_message &command) {
  bool turnOn = strcmp(command.state, "on") == 0;

  if (strcmp(command.device, "pump") == 0) {
    setRelay(PUMP_RELAY_PIN, turnOn);
    Serial.printf("Pump %s (seq=%lu)\n", turnOn ? "ON" : "OFF", (unsigned long)command.command_seq);
    return;
  }

  if (strcmp(command.device, "light") == 0) {
    setRelay(LIGHT_RELAY_PIN, turnOn);
    Serial.printf("Light %s (seq=%lu)\n", turnOn ? "ON" : "OFF", (unsigned long)command.command_seq);
    return;
  }

  Serial.printf("Unknown device: %s\n", command.device);
}

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len < (int)sizeof(control_command_message)) {
    Serial.printf("Ignored payload len=%d\n", len);
    return;
  }

  control_command_message command = {};
  memcpy(&command, incomingData, sizeof(command));

  if (strcmp(command.node_id, NODE_ID) != 0) {
    Serial.printf("Command for other node: %s\n", command.node_id);
    return;
  }

  if (strcmp(command.action_type, "relay_control") != 0) {
    Serial.printf("Unsupported action_type: %s\n", command.action_type);
    return;
  }

  applyCommand(command);
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    Serial.println("Heartbeat sent");
  } else {
    Serial.println("Heartbeat send failed");
  }
}

void sendHeartbeat() {
  heartbeatData.sensor_timestamp = millis();
  heartbeatData.rssi = WiFi.RSSI();
  heartbeatData.message_type = MSG_TYPE_HEARTBEAT;
  heartbeatData.uptime_sec = millis() / 1000;
  heartbeatData.heartbeat_seq = ++heartbeatSeq;

  esp_err_t result = esp_now_send(
    gatewayAddress,
    reinterpret_cast<const uint8_t *>(&heartbeatData),
    sizeof(heartbeatData)
  );

  if (result != ESP_OK) {
    Serial.printf("Heartbeat enqueue failed: %d\n", result);
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\nESP32 Control Node booting...");

  pinMode(PUMP_RELAY_PIN, OUTPUT);
  pinMode(LIGHT_RELAY_PIN, OUTPUT);
  setRelay(PUMP_RELAY_PIN, false);
  setRelay(LIGHT_RELAY_PIN, false);

  memset(&heartbeatData, 0, sizeof(heartbeatData));
  strncpy(heartbeatData.device_id, DEVICE_ID, sizeof(heartbeatData.device_id) - 1);
  strncpy(heartbeatData.node_id, NODE_ID, sizeof(heartbeatData.node_id) - 1);

  uint8_t wifiChannel = resolveWifiChannel();
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(wifiChannel, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(onDataRecv);
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, gatewayAddress, 6);
  peerInfo.channel = wifiChannel;
  peerInfo.encrypt = false;
  peerInfo.ifidx = WIFI_IF_STA;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add gateway peer");
    return;
  }

  esp_wifi_set_channel(wifiChannel, WIFI_SECOND_CHAN_NONE);

  Serial.print("Node ID: ");
  Serial.println(NODE_ID);
  Serial.print("Device ID: ");
  Serial.println(DEVICE_ID);
  Serial.print("Control node MAC: ");
  Serial.println(WiFi.macAddress());

  sendHeartbeat();
}

void loop() {
  static unsigned long lastHeartbeat = 0;
  unsigned long now = millis();

  if (now - lastHeartbeat >= HEARTBEAT_INTERVAL_MS) {
    lastHeartbeat = now;
    sendHeartbeat();
  }

  delay(100);
}
