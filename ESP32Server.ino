// !!! На данный момент барабанная установка начинает крутится лишь тогда,
// !!! когда на пин управления подается значение ~ 150.

const uint8_t CENTRIFUGE_SPEED_SHIFT = 154;


// Для создания web-приложения.
#include <WiFi.h>
// github: ESP32Async/ESPAsyncWebServer скачать, распаковать и положить в папку Arduino/libraries
#include <ESPAsyncWebServer.h>

// github: ESP32Async/AsyncTCP скачать, распаковать и положить в папку Arduino/libraries
// github: me-no-dev/arduino-esp32fs-plugin скачать, распаковать и положить в папку Arduino/tools/ (Arduino/tools/ESP32FS/tool/...).
// Положить все файлы html, css и тд. в папку data, расположенную в папке проекта. 
// Затем в Arduino IDE 1.* (На данный момент (16.07.24) не работает с 2.*) нажать tools -> ESP32 Sketch Data Upload

// Либо mkspiffs.py
// mkspiffs -c ./data -b 4096 -p 256 -s 0x1E0000 spiffs.bin
// esptool.py --port /dev/ttyUSB0 write_flash 0x210000 spiffs.bin


// Библиотека WebSockets by Markus Sattler
#include <WebSocketsServer.h>

// Библиотека ArduinoJson by Benoit Blanchon
#include <ArduinoJson.h>
#include <SPIFFS.h>

// Пользовательский файл, содержащий классы для управления мотором, сенсором и таймером.
#include <Utils.h>

// Используется для дебага.
// Чтобы перейти к собственной точке доступа ESP закомментируйте #define строку
// #define ESP_DEBUG
const char* LOCAL_SSID = "DancingCow";
const char* LOCAL_PASS = "perfectwe1";

// С этими данными будет создана точка доступа.
const char* APP_SSID = "Rotor-Control";
const char* APP_PASS = "1234567890";

// Задаёт время, через которое будет обнавляться информация с датчика тока.
const uint16_t SensorUpdateTime = 1000;
uint32_t SensorUpdate = 0;

C_L298N_Motor Motor;
C_INA219_Sensor Sensor;
C_Timer Timer;

// Используется для отправки сообщений между сервером и клиентом по веб-сокетам.
JsonDocument doc_tx; // Отправка сообщений клиенту
JsonDocument doc_rx; // Получение сообщений от клиента

// IP адрес для дебага
IPAddress Actual_IP;

// Информация для точки доступа ESP32.
IPAddress PageIP(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress ip;

AsyncWebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

#include "Debug.h"

void setup() {
  Serial.begin(115200);
  Serial.println("Starting setup");

  // Отключить перезапуск ESP32 при отсутствии сигнала.
  disableCore1WDT();

  if (!SPIFFS.begin(true)) {
    Serial.println("SPIFFS Mount Failed. Attempting to format...");
    if (SPIFFS.format()) {
      Serial.println("SPIFFS Formatted Successfully!");
      ESP.restart();
    } else {
      Serial.println("SPIFFS Format Failed!");
    }
  } else {
    Serial.println("SPIFFS Mounted Successfully!");
  }

  Serial.println("Motor setup");
  Motor.SetupPins();
  Serial.println("Done");

  Serial.println("Sensor setup");
  Sensor.SetupINA219();
  Serial.println("Done");

#ifdef ESP_DEBUG
  Serial.println("Connecting to access point. debug");
  WiFi.begin(LOCAL_SSID, LOCAL_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }
  Serial.println("Connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
#endif

#ifndef ESP_DEBUG
  Serial.println("Starting access point");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(APP_SSID, APP_PASS);
  delay(500);
  WiFi.softAPConfig(PageIP, gateway, subnet);
  delay(100);
  Serial.println("Done");
  Serial.print("SSID: ");
  Serial.println(APP_SSID);
  Serial.print("PASSWORD: ");
  Serial.println(APP_PASS);
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());
#endif

  server.on("/", HTTP_GET, SendWebsite);
  server.serveStatic("/", SPIFFS, "/"); // Разрешение серверу доступа к внутренним файлам ESP32.

  webSocket.begin();
  webSocket.onEvent(WebSocketEvent);
  server.begin();
  Serial.println("Setup done");

  printPartitionTable();
  printSpiffsSFiles();
}

void loop() {
  webSocket.loop();
  uint32_t now = millis();
  if ((now - SensorUpdate) >= SensorUpdateTime) {
    SensorUpdate = now;
    Timer.CheckTime(Motor);
    Sensor.UpdateSensor();
    SendJson();
  }
}

void SendWebsite(AsyncWebServerRequest* request) {
  request->send(SPIFFS, "/webpage.html", "text/html");
}

void SendJson() {
  doc_tx["RPM"] = (Motor.GetSpeed() != 0 ? Motor.GetSpeed() - CENTRIFUGE_SPEED_SHIFT : 0);
  auto [voltage, current] = Sensor.GetSensorData();
  doc_tx["VOLT"] = voltage;
  doc_tx["CURR"] = current;
  doc_tx["MOTOR_STATE"] = (Motor.GetState() ? "ON" : "OFF");
  doc_tx["TIMER_STATE"] = (Timer.GetState() ? "ON" : "OFF");
  doc_tx["TIME_LEFT"] = Timer.GetTimeRemaining();

  String jsonString = "";
  serializeJson(doc_tx, jsonString);
  webSocket.broadcastTXT(jsonString);
}

void WebSocketEvent(byte num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {

  case WStype_DISCONNECTED:
    Serial.println("Client Disconnected");
    Motor.SetSpeed(0);
    break;

  case WStype_CONNECTED:
    Serial.println("Client Connected");
    break;

  case WStype_TEXT:
    DeserializationError error = deserializeJson(doc_rx, payload);
    if (error) {
      Serial.println("deserializeJson() failed!");
      return;
    }
    String messageType = String(doc_rx["type"]);

    if (messageType == "CHANGE-STATE") {
      // Serial.println("Got CHANGE-STATE message");
      Motor.SwitchState();
      if (Motor.GetState()) {
        Timer.RefreshStartTime();
      } else {
        Timer.SetState(false);
        Timer.SetTime(0, 0, 0);
      }
    }
    if (messageType == "CHANGE-ROTATION") {
      // Serial.println("Got CHANGE-ROTATION message");
      Motor.ProcessReverse();
    }
    if (messageType == "CHANGE-SPEED") {
      // Serial.println("Got CHANGE-SPEED message");
      uint8_t newSpeed = int(doc_rx["speed"]) + CENTRIFUGE_SPEED_SHIFT;
      Motor.SetSpeed(newSpeed);
    }
    if (messageType == "UPDATE-TIMER") {
      // Serial.println("Got UPDATE-TIMER message");
      Timer.SetTime(doc_rx["hours"], doc_rx["minutes"], doc_rx["seconds"]);
      Timer.RefreshStartTime();
      Timer.SetState(true);
    }
    if (messageType == "DISABLE-TIMER") {
      // Serial.println("Got DISABLE-TIMER message");
      Timer.SetState(false);
      Timer.SetTime(0, 0, 0);
    }
  }
}
