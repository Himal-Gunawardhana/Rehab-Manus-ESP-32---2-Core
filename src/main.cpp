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

// PWM Channels
#define PWM_CHANNEL1 0
#define PWM_CHANNEL2 1
#define PWM_FREQ 50
#define PWM_RESOLUTION 16

// Shared Variables (volatile)
volatile int PWMSignal1 = 0;
volatile int PWMSignal2 = 0;
volatile bool switch1 = false;
volatile bool switch2 = true;

// Tasks
TaskHandle_t TaskPWM1;
TaskHandle_t TaskPWM2;

void connectToWiFi()
{
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
  }
  Serial.println(" Connected to WiFi!");
}

// Firebase fetch
void readDataFromFirebase()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    HTTPClient http;
    WiFiClientSecure client;
    client.setInsecure();

    String url = String("https://") + firebaseHost + "/rehabManus/" + rehabManus + ".json?auth=" + firebaseAuth;
    http.begin(client, url);
    int httpResponseCode = http.GET();

    if (httpResponseCode > 0)
    {
      String payload = http.getString();
      DynamicJsonDocument doc(1024);
      DeserializationError error = deserializeJson(doc, payload);

      if (!error)
      {
        switch1 = doc["switch1"] | false;
        switch2 = doc["switch2"] | false;
        PWMSignal1 = constrain((int)doc["PWMSignal1"], 0, 100);
        PWMSignal2 = constrain((int)doc["PWMSignal2"], 0, 100);
      }
      else
      {
        Serial.print("JSON error: ");
        Serial.println(error.f_str());
      }
    }
    else
    {
      Serial.print("HTTP Error: ");
      Serial.println(httpResponseCode);
    }

    http.end();
  }
}

// Core 0 Task – PWM1 control
void TaskPWM1Handler(void *parameter)
{
  while (true)
  {
    int pulse1 = map(PWMSignal1, 0, 100, 1000, 2000);
    int duty1 = (pulse1 * 65535) / 20000;
    ledcWrite(PWM_CHANNEL1, duty1);

    Serial.printf("[Core0] PWM1: %d%% | Pulse: %dus | Duty: %d\n", PWMSignal1, pulse1, duty1);
    delay(200); // Run every 200ms
  }
}

// Core 1 Task – PWM2 control
void TaskPWM2Handler(void *parameter)
{
  while (true)
  {
    int pulse2 = map(PWMSignal2, 0, 100, 1000, 2000);
    int duty2 = (pulse2 * 65535) / 20000;
    ledcWrite(PWM_CHANNEL2, duty2);

    Serial.printf("[Core1] PWM2: %d%% | Pulse: %dus | Duty: %d\n", PWMSignal2, pulse2, duty2);
    delay(200); // Run every 200ms
  }
}

void setup()
{
  Serial.begin(115200);
  connectToWiFi();

  pinMode(Relay_1, OUTPUT);
  pinMode(Relay_2, OUTPUT);

  ledcSetup(PWM_CHANNEL1, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(Motor_PWM1, PWM_CHANNEL1);

  ledcSetup(PWM_CHANNEL2, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(Motor_PWM2, PWM_CHANNEL2);

  // Create Tasks for two cores
  xTaskCreatePinnedToCore(TaskPWM1Handler, "PWM1Task", 10000, NULL, 1, &TaskPWM1, 0); // Core 0
  xTaskCreatePinnedToCore(TaskPWM2Handler, "PWM2Task", 10000, NULL, 1, &TaskPWM2, 1); // Core 1

  Serial.println("Tasks created on both cores.");
}

void loop()
{
  readDataFromFirebase();

  // Relay control
  digitalWrite(Relay_1, switch1 ? HIGH : LOW);
  digitalWrite(Relay_2, switch2 ? HIGH : LOW);

  delay(500); // Poll Firebase every 500ms
}