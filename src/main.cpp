#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <WiFiClientSecure.h>

// WiFi Credentials
const char *ssid = "HimalPixel";
const char *password = "abcdef11";

// Firebase
const char *firebaseHost = "stroke-rehab-arm-default-rtdb.asia-southeast1.firebasedatabase.app";
const char *firebaseAuth = "AIzaSyBfr5aBDUGnIKaOS62m9za4smyFWQae6tk";
const String rehabManus = "rehab123";

// Pins
#define Relay_1 26
#define Relay_2 27
#define Motor_PWM1 14
#define Motor_PWM2 25

// PWM Channels and Config
#define PWM_CHANNEL1 0
#define PWM_CHANNEL2 1
#define PWM_FREQ 50
#define PWM_RESOLUTION 16
#define PWM_MIN 1000
#define PWM_MAX 2000
#define PWM_NEUTRAL 1500
#define PWM_PERIOD_US 20000

// Shared Variables (volatile for dual-core access)
volatile int PWMSignal1 = 0;
volatile int PWMSignal2 = 0;
volatile bool switch1 = false;
volatile bool switch2 = true;

TaskHandle_t TaskPWM1;
TaskHandle_t TaskPWM2;

void connectToWiFi() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println(" Connected!");
}

void readDataFromFirebase() {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    WiFiClientSecure client;
    client.setInsecure();

    String url = String("https://") + firebaseHost + "/rehabManus/" + rehabManus + ".json?auth=" + firebaseAuth;
    http.begin(client, url);
    int httpResponseCode = http.GET();

    if (httpResponseCode > 0) {
      String payload = http.getString();
      DynamicJsonDocument doc(1024);
      DeserializationError error = deserializeJson(doc, payload);

      if (!error) {
        switch1 = doc["switch1"] | false;
        switch2 = doc["switch2"] | false;
        PWMSignal1 = constrain((int)doc["PWMSignal1"], 0, 100);
        PWMSignal2 = constrain((int)doc["PWMSignal2"], 0, 100);
      } else {
        Serial.print("JSON error: ");
        Serial.println(error.f_str());
      }
    } else {
      Serial.print("HTTP error: ");
      Serial.println(httpResponseCode);
      // Fallback to safe state
      PWMSignal1 = 0;
      PWMSignal2 = 0;
    }

    http.end();
  }
}

void writePWM(int channel, int percent) {
  int pulse = map(percent, 0, 100, PWM_MIN, PWM_MAX);
  int duty = (pulse * 65535UL) / PWM_PERIOD_US;
  ledcWrite(channel, duty);
  Serial.printf("PWM Ch%d: %d%% → Pulse: %dus → Duty: %d\n", channel, percent, pulse, duty);
}

// Core 0 Task
void TaskPWM1Handler(void *parameter) {
  int lastValue = -1;
  while (true) {
    if (PWMSignal1 != lastValue) {
      lastValue = PWMSignal1;
      writePWM(PWM_CHANNEL1, PWMSignal1);
    }
    delay(100);
  }
}

// Core 1 Task
void TaskPWM2Handler(void *parameter) {
  int lastValue = -1;
  while (true) {
    if (PWMSignal2 != lastValue) {
      lastValue = PWMSignal2;
      writePWM(PWM_CHANNEL2, PWMSignal2);
    }
    delay(100);
  }
}

void setup() {
  Serial.begin(115200);
  connectToWiFi();

  pinMode(Relay_1, OUTPUT);
  pinMode(Relay_2, OUTPUT);

  ledcSetup(PWM_CHANNEL1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(Motor_PWM1, PWM_CHANNEL1);
  ledcWrite(PWM_CHANNEL1, (PWM_NEUTRAL * 65535UL) / PWM_PERIOD_US); // Initial neutral

  ledcSetup(PWM_CHANNEL2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(Motor_PWM2, PWM_CHANNEL2);
  ledcWrite(PWM_CHANNEL2, (PWM_NEUTRAL * 65535UL) / PWM_PERIOD_US); // Initial neutral

  xTaskCreatePinnedToCore(TaskPWM1Handler, "PWM1Task", 10000, NULL, 1, &TaskPWM1, 0);
  xTaskCreatePinnedToCore(TaskPWM2Handler, "PWM2Task", 10000, NULL, 1, &TaskPWM2, 1);

  Serial.println("PWM tasks initialized on both cores.");
}

void loop() {
  readDataFromFirebase();

  digitalWrite(Relay_1, switch1 ? HIGH : LOW);
  digitalWrite(Relay_2, switch2 ? HIGH : LOW);

  delay(500); // Firebase polling interval
}