/*
 * ESP32 + BNO085 即時姿態 Demo
 * --------------------------------------------------------------
 * 功能:
 *   - BNO085 全功能讀取(四元數 / 加速度 / 陀螺儀 / 磁力計 /
 *     線性加速度 / 重力 / 計步 / 敲擊 / 搖晃 / 穩定度分類)
 *   - WebSocket 即時串流(~50Hz),網頁反應快速
 *   - 3D 姿態 + 指南針(網頁端渲染)
 *   - 網頁 OTA 韌體更新
 *   - WiFi 設定頁面,連不到 AP 時自動切換為 AP 直連 + Captive Portal
 *
 * 硬體接線 (SPI):
 *   BNO085 VIN  -> 3V3
 *   BNO085 GND  -> GND
 *   BNO085 SCK  -> GPIO18  (SPI CLK)
 *   BNO085 MISO/SDA/DO -> GPIO19  (SPI MISO)
 *   BNO085 MOSI/DI      -> GPIO23  (SPI MOSI)
 *   BNO085 CS   -> GPIO5   (晶片選擇,函式庫自動控制)
 *   BNO085 INT  -> GPIO4   (HINTN,資料就緒中斷,低態有效,必接)
 *   BNO085 RST  -> GPIO16  (NRST,硬體重置)
 *   BNO085 P0/PS0 -> GPIO17  (協定選擇,SPI 需拉高)
 *   BNO085 P1/PS1 -> GPIO25  (協定選擇,SPI 需拉高)
 *
 *   ※ PS1=HIGH 且 PS0=HIGH 才會在 reset 當下鎖定 SPI 模式,
 *     由 ESP32 於 begin_SPI 前先拉高並保持。
 *   ※ 函式庫固定使用 SPI_MODE3 @ 1MHz。
 */

#include <Arduino.h>
#include <SPI.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <Update.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <Adafruit_BNO08x.h>

// ---------- 接腳設定 (SPI) ----------
#define PIN_SCK   18   // SPI CLK
#define PIN_MISO  19   // SPI MISO (BNO085 的 SDA/DO)
#define PIN_MOSI  23   // SPI MOSI (BNO085 的 DI)
#define PIN_CS     5   // 晶片選擇
#define PIN_INT    4   // HINTN 資料就緒(低態有效,必接)
#define PIN_RST   16   // NRST 硬體重置
#define PIN_PS0   17   // 協定選擇 P0(SPI 需拉高)
#define PIN_PS1   25   // 協定選擇 P1(SPI 需拉高)

// ---------- AP 直連模式參數 ----------
static const char *AP_SSID     = "BNO085-Setup";
static const char *AP_PASSWORD = "12345678";   // 至少 8 碼;留空字串則為開放 AP
static const byte  DNS_PORT    = 53;
static const IPAddress AP_IP(192, 168, 4, 1);

// ---------- 全域物件 ----------
Adafruit_BNO08x bno(PIN_RST);
sh2_SensorValue_t sensorValue;
Preferences prefs;
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");
DNSServer dnsServer;

bool apMode = false;              // 目前是否為 AP 直連模式
uint32_t lastWsPush = 0;          // 上次推送時間
uint32_t wsPushMs = 10;           // 推送間隔(ms),可由網頁調整,預設 100Hz
const uint32_t WS_PUSH_MIN_MS = 10;   // 最快 100Hz(對齊感測器姿態上限)
const uint32_t WS_PUSH_MAX_MS = 200;  // 最慢 5Hz

// ---------- 感測器最新數值快取 ----------
struct SensorData {
  // 旋轉向量(含地磁,絕對方位)
  float rot_i = 0, rot_j = 0, rot_k = 0, rot_r = 1;
  float rot_acc = 0;                       // 準確度(弧度)
  // 遊戲旋轉向量(不含磁力,無漂移抖動,適合 3D)
  float grv_i = 0, grv_j = 0, grv_k = 0, grv_r = 1;
  // 地磁旋轉向量(指南針用)
  float geo_i = 0, geo_j = 0, geo_k = 0, geo_r = 1;
  // 原始三軸
  float acc_x = 0, acc_y = 0, acc_z = 0;   // 加速度 m/s^2
  float gyr_x = 0, gyr_y = 0, gyr_z = 0;   // 陀螺儀 rad/s
  float mag_x = 0, mag_y = 0, mag_z = 0;   // 磁力 uT
  float lin_x = 0, lin_y = 0, lin_z = 0;   // 線性加速度
  float grav_x = 0, grav_y = 0, grav_z = 0;// 重力向量
  // 校正狀態(0~3)
  uint8_t acc_cal = 0, gyr_cal = 0, mag_cal = 0, rot_cal = 0;
  // 事件 / 分類
  uint32_t steps = 0;
  uint8_t stability = 0;    // 0:Unknown 1:On Table 2:Stationary 3:Stable 4:Motion
  uint32_t tapCount = 0;
  uint32_t shakeCount = 0;
  uint32_t lastTapMs = 0;
  uint32_t lastShakeMs = 0;
} data;

// ================================================================
//  BNO085 初始化與報告設定
// ================================================================
bool enableReport(sh2_SensorId_t id, uint32_t interval_us) {
  if (!bno.enableReport(id, interval_us)) {
    Serial.printf("[BNO] 無法啟用報告 0x%02X\n", id);
    return false;
  }
  return true;
}

void setBnoReports() {
  // 姿態類:高更新率
  enableReport(SH2_ROTATION_VECTOR,            10000);  // 100Hz
  enableReport(SH2_GAME_ROTATION_VECTOR,       10000);  // 100Hz
  enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR,20000);  // 50Hz
  // 原始三軸:50Hz
  enableReport(SH2_ACCELEROMETER,              20000);
  enableReport(SH2_GYROSCOPE_CALIBRATED,       20000);
  enableReport(SH2_MAGNETIC_FIELD_CALIBRATED,  20000);
  enableReport(SH2_LINEAR_ACCELERATION,        20000);
  enableReport(SH2_GRAVITY,                    20000);
  // 事件 / 分類:低頻
  enableReport(SH2_STEP_COUNTER,              200000);
  enableReport(SH2_STABILITY_CLASSIFIER,      200000);
  enableReport(SH2_TAP_DETECTOR,              100000);
  enableReport(SH2_SHAKE_DETECTOR,            100000);
  Serial.println("[BNO] 報告啟用完成");
}

bool initBNO() {
  // 先拉高 PS0/PS1 以在 reset 當下鎖定 SPI 模式
  pinMode(PIN_PS0, OUTPUT); digitalWrite(PIN_PS0, HIGH);
  pinMode(PIN_PS1, OUTPUT); digitalWrite(PIN_PS1, HIGH);
  delay(10);

  // 自訂 VSPI 腳位
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_CS);

  // begin_SPI 內部會透過建構子的 RST 腳做硬體重置並鎖定協定
  if (!bno.begin_SPI(PIN_CS, PIN_INT, &SPI)) {
    Serial.println("[BNO] 找不到 BNO085 (SPI),請檢查接線 / PS0-PS1 是否拉高");
    return false;
  }
  Serial.println("[BNO] BNO085 已就緒 (SPI @ 1MHz, MODE3)");
  setBnoReports();
  return true;
}

// 逐一取出感測事件,更新快取
void pollBNO() {
  if (bno.wasReset()) {
    Serial.println("[BNO] 偵測到感測器重置,重新啟用報告");
    setBnoReports();
  }

  // 一次盡量讀完佇列。SPI HAL 在無資料時會空等最多 500ms,
  // 因此先用 INT(低態=資料就緒)把關,沒資料就直接離開。
  for (int i = 0; i < 40; i++) {
    if (digitalRead(PIN_INT) == HIGH) break;   // INT 未拉低 -> 無資料
    if (!bno.getSensorEvent(&sensorValue)) break;

    switch (sensorValue.sensorId) {
      case SH2_ROTATION_VECTOR:
        data.rot_i = sensorValue.un.rotationVector.i;
        data.rot_j = sensorValue.un.rotationVector.j;
        data.rot_k = sensorValue.un.rotationVector.k;
        data.rot_r = sensorValue.un.rotationVector.real;
        data.rot_acc = sensorValue.un.rotationVector.accuracy;
        data.rot_cal = sensorValue.status & 0x03;
        break;
      case SH2_GAME_ROTATION_VECTOR:
        data.grv_i = sensorValue.un.gameRotationVector.i;
        data.grv_j = sensorValue.un.gameRotationVector.j;
        data.grv_k = sensorValue.un.gameRotationVector.k;
        data.grv_r = sensorValue.un.gameRotationVector.real;
        break;
      case SH2_GEOMAGNETIC_ROTATION_VECTOR:
        data.geo_i = sensorValue.un.geoMagRotationVector.i;
        data.geo_j = sensorValue.un.geoMagRotationVector.j;
        data.geo_k = sensorValue.un.geoMagRotationVector.k;
        data.geo_r = sensorValue.un.geoMagRotationVector.real;
        break;
      case SH2_ACCELEROMETER:
        data.acc_x = sensorValue.un.accelerometer.x;
        data.acc_y = sensorValue.un.accelerometer.y;
        data.acc_z = sensorValue.un.accelerometer.z;
        data.acc_cal = sensorValue.status & 0x03;
        break;
      case SH2_GYROSCOPE_CALIBRATED:
        data.gyr_x = sensorValue.un.gyroscope.x;
        data.gyr_y = sensorValue.un.gyroscope.y;
        data.gyr_z = sensorValue.un.gyroscope.z;
        data.gyr_cal = sensorValue.status & 0x03;
        break;
      case SH2_MAGNETIC_FIELD_CALIBRATED:
        data.mag_x = sensorValue.un.magneticField.x;
        data.mag_y = sensorValue.un.magneticField.y;
        data.mag_z = sensorValue.un.magneticField.z;
        data.mag_cal = sensorValue.status & 0x03;
        break;
      case SH2_LINEAR_ACCELERATION:
        data.lin_x = sensorValue.un.linearAcceleration.x;
        data.lin_y = sensorValue.un.linearAcceleration.y;
        data.lin_z = sensorValue.un.linearAcceleration.z;
        break;
      case SH2_GRAVITY:
        data.grav_x = sensorValue.un.gravity.x;
        data.grav_y = sensorValue.un.gravity.y;
        data.grav_z = sensorValue.un.gravity.z;
        break;
      case SH2_STEP_COUNTER:
        data.steps = sensorValue.un.stepCounter.steps;
        break;
      case SH2_STABILITY_CLASSIFIER:
        data.stability = sensorValue.un.stabilityClassifier.classification;
        break;
      case SH2_TAP_DETECTOR:
        data.tapCount++;
        data.lastTapMs = millis();
        break;
      case SH2_SHAKE_DETECTOR:
        if (sensorValue.un.shakeDetector.shake) {
          data.shakeCount++;
          data.lastShakeMs = millis();
        }
        break;
      default:
        break;
    }
  }
}

// ================================================================
//  將感測資料序列化成緊湊 JSON 並推送給所有 WS 用戶端
// ================================================================
void pushSensorData() {
  if (ws.count() == 0) return;

  static char buf[900];
  uint32_t now = millis();
  int n = snprintf(buf, sizeof(buf),
    "{\"t\":%lu,"
    "\"rot\":[%.4f,%.4f,%.4f,%.4f],\"rotAcc\":%.3f,"
    "\"grv\":[%.4f,%.4f,%.4f,%.4f],"
    "\"geo\":[%.4f,%.4f,%.4f,%.4f],"
    "\"acc\":[%.3f,%.3f,%.3f],"
    "\"gyr\":[%.3f,%.3f,%.3f],"
    "\"mag\":[%.2f,%.2f,%.2f],"
    "\"lin\":[%.3f,%.3f,%.3f],"
    "\"grav\":[%.3f,%.3f,%.3f],"
    "\"cal\":[%u,%u,%u,%u],"
    "\"steps\":%lu,\"stab\":%u,"
    "\"tap\":%lu,\"shake\":%lu,"
    "\"tapAgo\":%lu,\"shakeAgo\":%lu}",
    (unsigned long)now,
    data.rot_i, data.rot_j, data.rot_k, data.rot_r, data.rot_acc,
    data.grv_i, data.grv_j, data.grv_k, data.grv_r,
    data.geo_i, data.geo_j, data.geo_k, data.geo_r,
    data.acc_x, data.acc_y, data.acc_z,
    data.gyr_x, data.gyr_y, data.gyr_z,
    data.mag_x, data.mag_y, data.mag_z,
    data.lin_x, data.lin_y, data.lin_z,
    data.grav_x, data.grav_y, data.grav_z,
    data.acc_cal, data.gyr_cal, data.mag_cal, data.rot_cal,
    (unsigned long)data.steps, data.stability,
    (unsigned long)data.tapCount, (unsigned long)data.shakeCount,
    (unsigned long)(now - data.lastTapMs), (unsigned long)(now - data.lastShakeMs));

  if (n > 0 && n < (int)sizeof(buf)) {
    ws.textAll(buf, n);
  }
}

// ================================================================
//  WiFi:嘗試連線,失敗則開啟 AP 直連
// ================================================================
bool connectSTA(const String &ssid, const String &pass, uint32_t timeoutMs = 12000) {
  if (ssid.isEmpty()) return false;
  Serial.printf("[WiFi] 連線至 \"%s\" ...\n", ssid.c_str());
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), pass.c_str());
  uint32_t start = millis();
  while (millis() - start < timeoutMs) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("[WiFi] 已連線,IP = %s\n", WiFi.localIP().toString().c_str());
      return true;
    }
    delay(200);
  }
  Serial.println("[WiFi] 連線逾時");
  return false;
}

void startAP() {
  apMode = true;
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(AP_IP, AP_IP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID, strlen(AP_PASSWORD) >= 8 ? AP_PASSWORD : nullptr);
  Serial.printf("[WiFi] 已開啟 AP:\"%s\"  IP = %s\n",
                AP_SSID, WiFi.softAPIP().toString().c_str());
  // Captive Portal:所有網域都導向本機
  dnsServer.start(DNS_PORT, "*", AP_IP);
}

void setupWiFi() {
  prefs.begin("wifi", true);
  String ssid = prefs.getString("ssid", "");
  String pass = prefs.getString("pass", "");
  prefs.end();

  if (!connectSTA(ssid, pass)) {
    startAP();
  }
}

// ================================================================
//  WebSocket 事件
// ================================================================
// 依 Hz 設定推送間隔(夾在 5~100Hz 內)
void applyRate(float hz) {
  if (hz < 1) hz = 1;
  uint32_t ms = (uint32_t)(1000.0f / hz + 0.5f);
  if (ms < WS_PUSH_MIN_MS) ms = WS_PUSH_MIN_MS;
  if (ms > WS_PUSH_MAX_MS) ms = WS_PUSH_MAX_MS;
  wsPushMs = ms;
  Serial.printf("[WS] 推送率設為 %.0f Hz (%lu ms)\n", 1000.0f / ms, (unsigned long)ms);
}

// 送出目前設定給單一用戶端,讓網頁滑桿同步(min=最慢,max=最快)
void sendConfig(AsyncWebSocketClient *client) {
  char cfg[96];
  int n = snprintf(cfg, sizeof(cfg),
    "{\"cfg\":{\"rateHz\":%.0f,\"min\":%.0f,\"max\":%.0f}}",
    1000.0f / wsPushMs, 1000.0f / WS_PUSH_MAX_MS, 1000.0f / WS_PUSH_MIN_MS);
  if (n > 0) client->text(cfg, n);
}

void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *payload, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.printf("[WS] 用戶端 #%u 連線\n", client->id());
    sendConfig(client);
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.printf("[WS] 用戶端 #%u 離線\n", client->id());
  } else if (type == WS_EVT_DATA) {
    // 只處理單一封包、完整、文字型訊息(控制指令都很短)
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len &&
        info->opcode == WS_TEXT && len < 128) {
      JsonDocument doc;
      if (deserializeJson(doc, payload, len) == DeserializationError::Ok &&
          !doc["rateHz"].isNull()) {
        applyRate(doc["rateHz"].as<float>());
      }
    }
  }
}

// ================================================================
//  Web 伺服器路由
// ================================================================
String stabilityName(uint8_t s) {
  switch (s) {
    case 1: return "On Table";
    case 2: return "Stationary";
    case 3: return "Stable";
    case 4: return "Motion";
    default: return "Unknown";
  }
}

void setupServer() {
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);

  // ---- 系統資訊 ----
  server.on("/api/info", HTTP_GET, [](AsyncWebServerRequest *req) {
    JsonDocument doc;
    doc["mode"]    = apMode ? "AP" : "STA";
    doc["ssid"]    = apMode ? AP_SSID : WiFi.SSID();
    doc["ip"]      = apMode ? WiFi.softAPIP().toString() : WiFi.localIP().toString();
    doc["rssi"]    = apMode ? 0 : WiFi.RSSI();
    doc["mac"]     = WiFi.macAddress();
    doc["heap"]    = ESP.getFreeHeap();
    doc["uptime"]  = millis() / 1000;
    doc["chip"]    = ESP.getChipModel();
    String out; serializeJson(doc, out);
    req->send(200, "application/json", out);
  });

  // ---- WiFi 掃描 ----
  server.on("/api/wifi/scan", HTTP_GET, [](AsyncWebServerRequest *req) {
    int n = WiFi.scanComplete();
    if (n == WIFI_SCAN_FAILED) {
      WiFi.scanNetworks(true);   // 非同步掃描
      req->send(202, "application/json", "{\"scanning\":true}");
      return;
    }
    if (n == WIFI_SCAN_RUNNING) {
      req->send(202, "application/json", "{\"scanning\":true}");
      return;
    }
    JsonDocument doc;
    JsonArray arr = doc["networks"].to<JsonArray>();
    for (int i = 0; i < n; i++) {
      JsonObject o = arr.add<JsonObject>();
      o["ssid"] = WiFi.SSID(i);
      o["rssi"] = WiFi.RSSI(i);
      o["enc"]  = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
    }
    WiFi.scanDelete();
    String out; serializeJson(doc, out);
    req->send(200, "application/json", out);
  });

  // ---- 儲存 WiFi 設定 ----
  server.on("/api/wifi/save", HTTP_POST, [](AsyncWebServerRequest *req) {
    String ssid, pass;
    if (req->hasParam("ssid", true)) ssid = req->getParam("ssid", true)->value();
    if (req->hasParam("pass", true)) pass = req->getParam("pass", true)->value();
    if (ssid.isEmpty()) {
      req->send(400, "application/json", "{\"ok\":false,\"msg\":\"SSID 不可空白\"}");
      return;
    }
    prefs.begin("wifi", false);
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    prefs.end();
    req->send(200, "application/json",
              "{\"ok\":true,\"msg\":\"已儲存,裝置將重新啟動並連線\"}");
    delay(300);
    ESP.restart();
  });

  // ---- 重新開機 ----
  server.on("/api/reboot", HTTP_POST, [](AsyncWebServerRequest *req) {
    req->send(200, "application/json", "{\"ok\":true}");
    delay(300);
    ESP.restart();
  });

  // ---- OTA 韌體更新 ----
  server.on("/api/update", HTTP_POST,
    [](AsyncWebServerRequest *req) {
      bool ok = !Update.hasError();
      AsyncWebServerResponse *resp = req->beginResponse(
        ok ? 200 : 500, "application/json",
        ok ? "{\"ok\":true,\"msg\":\"更新成功,重新啟動中\"}"
           : "{\"ok\":false,\"msg\":\"更新失敗\"}");
      resp->addHeader("Connection", "close");
      req->send(resp);
      if (ok) { delay(500); ESP.restart(); }
    },
    [](AsyncWebServerRequest *req, String filename, size_t index,
       uint8_t *data, size_t len, bool final) {
      if (index == 0) {
        Serial.printf("[OTA] 開始更新:%s\n", filename.c_str());
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
          Update.printError(Serial);
        }
      }
      if (Update.write(data, len) != len) {
        Update.printError(Serial);
      }
      if (final) {
        if (Update.end(true)) {
          Serial.printf("[OTA] 完成,共 %u bytes\n", index + len);
        } else {
          Update.printError(Serial);
        }
      }
    });

  // ---- 靜態網頁(LittleFS)----
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  // ---- Captive Portal / 404 ----
  server.onNotFound([](AsyncWebServerRequest *req) {
    if (apMode) {
      req->redirect("/");   // AP 模式一律導向設定頁
    } else {
      req->send(404, "text/plain", "Not Found");
    }
  });

  server.begin();
  Serial.println("[HTTP] 伺服器已啟動");
}

// ================================================================
//  setup / loop
// ================================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== ESP32 + BNO085 Demo 啟動 ===");

  if (!LittleFS.begin(true)) {
    Serial.println("[FS] LittleFS 掛載失敗(是否已執行 uploadfs?)");
  }

  initBNO();
  setupWiFi();
  setupServer();
}

void loop() {
  if (apMode) dnsServer.processNextRequest();

  pollBNO();

  uint32_t now = millis();
  if (now - lastWsPush >= wsPushMs) {
    lastWsPush = now;
    pushSensorData();
    ws.cleanupClients();
  }
}
