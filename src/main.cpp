#include <Arduino.h>
#include "addpeers.h"
#include "message.h"
#include <WiFi.h>
#include <esp_now.h>
#include <Adafruit_NeoPixel.h>

//VERSION
String fwversion = "1.0";

#define BUTTON1_PIN 1  // GPIO1 -> PEDAL1  [R1]
#define BUTTON2_PIN 2  // GPIO2 -> PEDAL2  [R3]

//PEDAL INFO STRUCT
struct PedalInfo {
  uint8_t id1;
  uint8_t id2;
};
PedalInfo currentPedal;


// debounce kezelés a gombokra
unsigned long lastPress1 = 0;
unsigned long lastPress2 = 0;
const unsigned long debounceDelay = 200;  // 200 ms

// LED pinek és darabszám
#define NUM_LEDS 1  // minden LED csík 1 pixel
#define LED1_PIN 3  // GPIO1 -> PEDAL1  [R1]
#define LED2_PIN 10 // GPIO1 -> PEDAL2  [R1]

// LED csíkok objektumai
Adafruit_NeoPixel led1(NUM_LEDS, LED1_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel led2(NUM_LEDS, LED2_PIN, NEO_GRB + NEO_KHZ800);

// --- FUNKCIÓK ---
PedalInfo getPedalInfo() {
  String mac = WiFi.macAddress();
  if(mac == "EC:DA:3B:BF:E9:A8") {
    return {1, 2};
  }
  if(mac == "EC:DA:3B:BF:C6:B8") {
    return {3, 4};
  }
  // Alapértelmezett, 'ismeretlen' érték
  return {0, 0};
}

// Segédfüggvény LED szín állításra
void setColor(Adafruit_NeoPixel &led, uint8_t r, uint8_t g, uint8_t b) {
  led.setPixelColor(0, led.Color(r, g, b));
  led.show();
}

//SendNOW(cimzett mac cim, üzenet);
void SendNOW(const uint8_t *mac, const Message &msg) {
  esp_err_t result = esp_now_send(mac, (uint8_t *)&msg, sizeof(msg));
  (result == ESP_OK) ? Serial.println("✅ Message sent successfully") : Serial.printf("❌ Send error (%d)\n", result);
}

//ellenorzés cimzett megkapta-e
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Send status: OK, " + String(macStr) : "Send status: FAIL, " + String(macStr));
  if(status != ESP_NOW_SEND_SUCCESS){
      //setColor(led1, 255, 0, 0);
      //setColor(led2, 255, 0, 0);
  }
}

void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

  if (len != sizeof(Message)) {
    Serial.printf("⚠️ Invalid message size: %d\n", len);
    return;
  }

  Message msg;
  memcpy(&msg, incomingData, sizeof(msg));
  Serial.printf("📩 Received msg: from=%d, type=%d, index=%d, value=%d\n", macStr, msg.type, msg.index, msg.value);

  switch (msg.type) {
    case 9: // 9. tipusú reset üzenet
          setColor(led1, 255, 0, 0);  // piros
          setColor(led2, 255, 0, 0);  //
    break;
    default:
      Serial.println("❓ Unknown message type");
  }

}

//FŐ programkód
void setup() {
  delay(3000);
  Serial.begin(115200);

  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);

  // LED-ek inicializálása és alapból piros
  led1.begin(); setColor(led1, 255, 0, 0);
  led2.begin(); setColor(led2, 255, 0, 0);

  Serial.print("ESP32 MAC: ");
  Serial.println(WiFi.macAddress());

  WiFi.mode(WIFI_STA);

  currentPedal = getPedalInfo();

  WiFi.disconnect();

  //ESP-NOW inicializálás
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }

  esp_now_register_recv_cb(OnDataRecv);
  esp_now_register_send_cb(OnDataSent);

  addPeers();// <-- ide tedd be, hogy hozzáadja a peer-eket

}

void loop() {
  static bool prev1 = HIGH;
  static bool prev2 = HIGH;

  bool curr1 = digitalRead(BUTTON1_PIN);
  bool curr2 = digitalRead(BUTTON2_PIN);

  unsigned long now = millis();

  if (prev1 == HIGH && curr1 == LOW && now - lastPress1 > debounceDelay) {
    lastPress1 = now;
    Message msg = {1, currentPedal.id1, 0};  // verseny uzenet, első versenyző (R1), kész=0
    SendNOW(receiver1, msg);
    SendNOW(receiver2, msg);
    setColor(led1, 0, 255, 0);  // zöld
  }

  if (prev2 == HIGH && curr2 == LOW && now - lastPress2 > debounceDelay) {
    lastPress2 = now;
    Message msg = {1, currentPedal.id2, 0}; // verseny uzenet, masodik versenyző (R3), kész=0
    SendNOW(receiver1, msg);
    SendNOW(receiver2, msg);
    setColor(led2, 0, 255, 0);  // zöld
  }

  prev1 = curr1;
  prev2 = curr2;
}