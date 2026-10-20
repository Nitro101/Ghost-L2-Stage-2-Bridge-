#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoOTA.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ---------------- OLED ----------------

#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_ADDRESS 0x3C
#define PIN_OLED_SDA 21
#define PIN_OLED_SCL 22

Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

// ---------------- WIFI ----------------

const char *AP_SSID = "Ghost-L2-Bridge";
const char *AP_PASS = "ghost12345";

WebServer server(80);
Preferences prefs;

String webMessage = "";
unsigned long webMessageUntil = 0;

// ---------------- LIMITS ----------------

#define MAX_CYDS 8
#define MAX_PENDING 8
#define MAX_MESHES 5
#define MAX_NODES 32
#define MAX_SENSORS 16

// ---------------- CYD DEVICES ----------------

struct CydDevice {
  uint8_t mac[6];
  char name[24];
  unsigned long lastSeen;
};

struct PendingCyd {
  uint8_t mac[6];
  char name[24];
  unsigned long receivedAt;
};

CydDevice cyds[MAX_CYDS];
int cydCount = 0;

PendingCyd pending[MAX_PENDING];
int pendingCount = 0;

// ---------------- MESH CONFIG ----------------

struct MeshConfig {
  char name[21];
  char key[33];
  bool enabled;
  bool alarmEnabled;
  char expectedIds[100];
  uint16_t lossTimeout;
};

MeshConfig meshes[MAX_MESHES];

// ---------------- NODE STATE ----------------

struct NodeState {
  bool seen;
  bool lossAlarm;
  bool radarAlarm;
  bool tamperAlarm;
  bool sensorFault;
  bool lowBattery;
  bool attackAlarm;
  bool healthAlarm;
  bool attentionAlarm;
  bool fallAlarm;
  bool presence;
  bool heartRateAlarm;

  uint8_t heartRate;
  uint8_t breathingRate;
  uint8_t alarmMode;

  uint8_t sensorMask;
  float sensors[MAX_SENSORS];

  uint32_t lastSequence;
  unsigned long lastSeenMs;
  uint32_t packets;
};

NodeState nodes[MAX_MESHES][MAX_NODES + 1];

// ---------------- PACKETS ----------------

struct BridgePairingRequest {
  char cydName[24];
};

struct BridgeAck {
  char bridgeName[24];
  bool accepted;
};

struct BridgeRadarSummary {
  char cydName[24];
  uint8_t meshId;
  uint8_t nodeId;
  uint8_t online;
  uint8_t radarAlarm;
  uint8_t tamperAlarm;
  uint8_t lossAlarm;
  uint8_t targetCount;
  uint16_t distanceCm;
  int16_t speed;
  int16_t angle10;
  uint8_t attackAlarm;
  uint8_t healthAlarm;
  uint8_t attentionAlarm;
  uint8_t fallAlarm;
  uint8_t presence;
  uint8_t heartRate;
  uint8_t breathingRate;
  uint8_t heartRateAlarm;
  uint8_t alarmMode;
};

struct Stage1Packet {
  uint32_t magic;
  uint8_t version;
  uint8_t meshId;
  uint8_t nodeId;
  uint32_t sequence;
  uint32_t auth;
  uint8_t sensorMask;
  uint8_t alarmFlags;
  float sensors[MAX_SENSORS];
};

#define STAGE1_MAGIC 0x314C4748UL
#define STAGE1_VERSION 1

#define ALARM_RADAR 0x01
#define ALARM_TAMPER 0x02
#define ALARM_SENSOR_FAULT 0x04
#define ALARM_LOW_BATTERY 0x08
#define ALARM_ATTACK 0x10
#define ALARM_HEALTH 0x20
#define ALARM_ATTENTION 0x20
#define ALARM_FALL 0x40
#define ALARM_PRESENCE 0x80
#define ALARM_HEART_RATE 0x100

// ---------------- STAGE 3 ----------------

uint8_t stage3Mac[6] = {0, 0, 0, 0, 0, 0};
bool stage3Configured = false;
bool stage3Connected = false;
unsigned long stage3LastAck = 0;

// ---------------- SYSTEM STATE ----------------

bool systemArmed = false;
bool globalAlarm = false;
String globalAlarmText = "SYSTEM OK";

char controlPin[13] = "2468";

// ---------------- HELPERS ----------------

void macToString(const uint8_t mac[6], char *out) {
  sprintf(
    out,
    "%02X:%02X:%02X:%02X:%02X:%02X",
    mac[0], mac[1], mac[2],
    mac[3], mac[4], mac[5]
  );
}

bool parseMac(String text, uint8_t mac[6]) {
  text.trim();

  int value[6];

  if (
    sscanf(
      text.c_str(),
      "%x:%x:%x:%x:%x:%x",
      &value[0], &value[1], &value[2],
      &value[3], &value[4], &value[5]
    ) != 6
  ) {
    return false;
  }

  for (int i = 0; i < 6; i++) {
    mac[i] = value[i];
  }

  return true;
}

bool expectedNode(uint8_t meshIndex, uint8_t nodeId) {
  if (meshIndex >= MAX_MESHES || nodeId < 1 || nodeId > MAX_NODES) {
    return false;
  }

  String list = String(meshes[meshIndex].expectedIds);
  list.trim();

  if (list.length() == 0) {
    return false;
  }

  int start = 0;

  while (start < list.length()) {
    int comma = list.indexOf(",", start);

    if (comma < 0) {
      comma = list.length();
    }

    String item = list.substring(start, comma);
    item.trim();

    if (item.toInt() == nodeId) {
      return true;
    }

    start = comma + 1;
  }

  return false;
}

uint8_t countExpected(uint8_t meshIndex) {
  if (meshIndex >= MAX_MESHES) {
    return 0;
  }

  uint8_t count = 0;

  for (int n = 1; n <= MAX_NODES; n++) {
    if (expectedNode(meshIndex, n)) {
      count++;
    }
  }

  return count;
}

uint16_t onlineCount(uint8_t meshIndex) {
  if (meshIndex >= MAX_MESHES) {
    return 0;
  }

  uint16_t count = 0;

  for (int n = 1; n <= MAX_NODES; n++) {
    if (
      expectedNode(meshIndex, n) &&
      nodes[meshIndex][n].seen &&
      !nodes[meshIndex][n].lossAlarm
    ) {
      count++;
    }
  }

  return count;
}

// ---------------- ALARMS ----------------

void setAlarm(String text, bool critical) {
  globalAlarm = true;
  globalAlarmText = text;

  Serial.println("ALARM: " + text);
}

void clearAlarms() {
  globalAlarm = false;
  globalAlarmText = "SYSTEM OK";

  for (int m = 0; m < MAX_MESHES; m++) {
    for (int n = 1; n <= MAX_NODES; n++) {
      nodes[m][n].radarAlarm = false;
      nodes[m][n].tamperAlarm = false;
      nodes[m][n].sensorFault = false;
      nodes[m][n].lowBattery = false;
      nodes[m][n].attackAlarm = false;
      nodes[m][n].healthAlarm = false;
      nodes[m][n].attentionAlarm = false;
      nodes[m][n].fallAlarm = false;
      nodes[m][n].heartRateAlarm = false;
    }
  }
}

// ---------------- OLED ----------------

void drawOLED() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.println("GHOST L2 BRIDGE");

  display.setCursor(0, 14);
  display.println(systemArmed ? "SYSTEM: ARMED" : "SYSTEM: DISARMED");

  display.setCursor(0, 26);

  if (globalAlarm) {
    display.println("ALARM ACTIVE");
  } else {
    display.println("SYSTEM OK");
  }

  display.setCursor(0, 38);

  if (stage3Configured && stage3Connected) {
    display.println("STAGE3: CONNECTED");
  } else if (stage3Configured) {
    display.println("STAGE3: WAITING");
  } else {
    display.println("STAGE3: NOT SET");
  }

  display.setCursor(0, 52);
  display.printf("CYDS:%d PENDING:%d", cydCount, pendingCount);

  display.display();
}

// ---------------- WEB HELPERS ----------------

String css() {
  String x;

  x += "<style>";
  x += "*{box-sizing:border-box}";
  x += "body{font-family:Arial;background:#0b1220;color:#e5e7eb;margin:0;padding:20px}";
  x += ".box{background:#121b2c;padding:20px;margin-bottom:20px;border-radius:12px;border:1px solid #23324a}";
  x += "h1,h2,h3{color:#22d3ee}";
  x += "h1{margin-top:0}";
  x += "label{display:block;margin-top:12px}";
  x += "input,select{width:100%;padding:12px;margin-top:5px;box-sizing:border-box;background:#1a2438;color:white;border:1px solid #2f425e;border-radius:8px}";
  x += "button{padding:12px 20px;margin-top:15px;border:0;border-radius:8px;background:#22d3ee;color:#06202a;font-weight:bold}";
  x += ".danger{background:#ef4444;color:white}";
  x += ".good{color:#34d399}";
  x += ".bad{color:#f87171}";
  x += ".warn{color:#fbbf24}";
  x += ".small{font-size:13px;color:#94a3b8}";
  x += "table{width:100%;border-collapse:collapse;margin-top:10px}";
  x += "td,th{border:1px solid #23324a;padding:8px;text-align:left}";
  x += "</style>";

  return x;
}

// ---------------- DASHBOARD ----------------

void dashboardPage() {
  String x;

  x += "<!DOCTYPE html><html><head>";
  x += "<meta charset='UTF-8'>";
  x += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  x += "<title>Ghost L2 Bridge</title>";
  x += css();
  x += "</head><body>";

  x += "<h1>Ghost L2 Bridge</h1>";

  x += "<div class='box'>";
  x += "<h2>Bridge Status</h2>";

  if (webMessage.length() && millis() < webMessageUntil) {
    x += "<p><b>" + webMessage + "</b></p>";
  }

  uint8_t bridgeMacBytes[6];
  WiFi.macAddress(bridgeMacBytes);

  char bridgeMacStr[18];
  macToString(bridgeMacBytes, bridgeMacStr);

  x += "<p>Bridge MAC address:</p>";
  x += "<p><b>" + String(bridgeMacStr) + "</b></p>";

  x += "<p>Wi-Fi: <span class='good'>READY</span></p>";
  x += "<p>ESP-NOW: <span class='good'>READY</span></p>";

  x += "<p>System: ";

  if (systemArmed) {
    x += "<span class='good'>ARMED</span>";
  } else {
    x += "<span class='warn'>DISARMED</span>";
  }

  x += "</p>";

  x += "<p>";

  if (globalAlarm) {
    x += "<span class='bad'>ALARM: " + globalAlarmText + "</span>";
  } else {
    x += "<span class='good'>SYSTEM OK</span>";
  }

  x += "</p>";

  x += "<p>Stage 3: ";

  if (!stage3Configured) {
    x += "<span class='warn'>NOT CONFIGURED</span>";
  } else if (stage3Connected) {
    x += "<span class='good'>CONNECTED</span>";
  } else {
    x += "<span class='bad'>WAITING</span>";
  }

  x += "</p>";
  x += "</div>";

  x += "<div class='box'>";
  x += "<h2>System Control</h2>";

  x += "<form method='POST' action='/system/activate'>";
  x += "<label>CONTROL PIN</label>";
  x += "<input type='password' name='pin'>";
  x += "<button>ACTIVATE SYSTEM</button>";
  x += "</form>";

  x += "<form method='POST' action='/system/deactivate'>";
  x += "<label>CONTROL PIN</label>";
  x += "<input type='password' name='pin'>";
  x += "<button class='danger'>DEACTIVATE SYSTEM</button>";
  x += "</form>";

  x += "<form method='POST' action='/alarm/clear'>";
  x += "<button>ACKNOWLEDGE / CLEAR ALARMS</button>";
  x += "</form>";
  x += "</div>";

  x += "<div class='box'>";
  x += "<h2>Stage 3 Controller</h2>";

  char stage3MacStr[18];
  macToString(stage3Mac, stage3MacStr);

  x += "<form method='POST' action='/stage3/save'>";
  x += "<label>Stage 3 MAC Address</label>";
  x += "<input name='stage3mac' value='" + String(stage3MacStr) + "'>";
  x += "<button>SAVE STAGE 3 MAC</button>";
  x += "</form>";
  x += "</div>";

  x += "<div class='box'>";
  x += "<h2>Mesh Settings</h2>";

  for (int m = 0; m < MAX_MESHES; m++) {
    x += "<h3>MESH " + String(m + 1) + "</h3>";

    x += "<form method='POST' action='/mesh/save'>";
    x += "<input type='hidden' name='mesh' value='" + String(m) + "'>";

    x += "<label>MESH NAME</label>";
    x += "<input name='name' value='" + String(meshes[m].name) + "'>";

    x += "<label>MESH SECURITY KEY</label>";
    x += "<input name='key' value='" + String(meshes[m].key) + "'>";

    x += "<label>ENABLED</label>";
    x += "<select name='enabled'>";
    x += "<option value='0'";

    if (!meshes[m].enabled) {
      x += " selected";
    }

    x += ">OFF</option>";
    x += "<option value='1'";

    if (meshes[m].enabled) {
      x += " selected";
    }

    x += ">ON</option>";
    x += "</select>";

    x += "<label>EXPECTED NODE IDS</label>";
    x += "<input name='ids' value='" + String(meshes[m].expectedIds) + "'>";
    x += "<p class='small'>Example: 1,2,3,4</p>";

    x += "<label>NODE LOSS TIMEOUT SECONDS</label>";
    x += "<input type='number' name='timeout' min='2' max='3600' value='" + String(meshes[m].lossTimeout) + "'>";

    x += "<button>SAVE MESH</button>";
    x += "</form>";
  }

  x += "</div>";

  x += "<div class='box'>";
  x += "<h2>Node Status</h2>";

  x += "<table>";
  x += "<tr><th>MESH</th><th>NODE</th><th>STATUS</th><th>ALARMS</th></tr>";

  for (int m = 0; m < MAX_MESHES; m++) {
    if (!meshes[m].enabled) {
      continue;
    }

    for (int n = 1; n <= MAX_NODES; n++) {
      if (!expectedNode(m, n)) {
        continue;
      }

      x += "<tr>";
      x += "<td>" + String(meshes[m].name) + "</td>";
      x += "<td>" + String(n) + "</td>";

      if (nodes[m][n].seen && !nodes[m][n].lossAlarm) {
        x += "<td class='good'>ONLINE</td>";
      } else {
        x += "<td class='bad'>OFFLINE / LOST</td>";
      }

      String alarms = "";

      if (nodes[m][n].radarAlarm) alarms += "RADAR ";
      if (nodes[m][n].tamperAlarm) alarms += "TAMPER ";
      if (nodes[m][n].sensorFault) alarms += "SENSOR ";
      if (nodes[m][n].lowBattery) alarms += "BATTERY ";
      if (nodes[m][n].attackAlarm) alarms += "ATTACK ";
      if (nodes[m][n].healthAlarm) alarms += "HEALTH ";
      if (nodes[m][n].attentionAlarm) alarms += "ATTENTION ";
      if (nodes[m][n].fallAlarm) alarms += "FALL ";
      if (nodes[m][n].heartRateAlarm) alarms += "HEART ";

      if (alarms.length() == 0) {
        alarms = "CLEAR";
      }

      x += "<td>" + alarms + "</td>";
      x += "</tr>";
    }
  }

  x += "</table>";
  x += "</div>";

  x += "<div class='box'>";
  x += "<h2>Sensor Nodes</h2>";

  bool anySensor = false;

  for (int m = 0; m < MAX_MESHES; m++) {
    if (!meshes[m].enabled) {
      continue;
    }

    for (int n = 1; n <= MAX_NODES; n++) {
      NodeState node = nodes[m][n];

      if (!node.seen || node.sensorMask == 0) {
        continue;
      }

      anySensor = true;

      x += "<h3>" + String(meshes[m].name) + " NODE " + String(n) + "</h3>";
      x += "<table>";

      for (int s = 0; s < MAX_SENSORS; s++) {
        if (!(node.sensorMask & (1 << s))) {
          continue;
        }

        x += "<tr>";
        x += "<td>SENSOR " + String(s + 1) + "</td>";
        x += "<td>" + String(node.sensors[s], 2) + "</td>";
        x += "</tr>";
      }

      x += "</table>";
    }
  }

  if (!anySensor) {
    x += "<p class='small'>No sensor nodes reporting yet.</p>";
  }

  x += "</div>";

  x += "<div class='box'>";
  x += "<h2>Pairing Requests</h2>";

  if (pendingCount == 0) {
    x += "<p class='small'>No pending CYD pairing requests.</p>";
  }

  for (int i = 0; i < pendingCount; i++) {
    char macStr[18];
    macToString(pending[i].mac, macStr);

    x += "<div class='box'>";
    x += "<p><b>" + String(pending[i].name) + "</b></p>";
    x += "<p class='small'>" + String(macStr) + "</p>";

    x += "<form method='POST' action='/pair/accept'>";
    x += "<input type='hidden' name='index' value='" + String(i) + "'>";
    x += "<button>ACCEPT</button>";
    x += "</form>";

    x += "<form method='POST' action='/pair/reject'>";
    x += "<input type='hidden' name='index' value='" + String(i) + "'>";
    x += "<button class='danger'>REJECT</button>";
    x += "</form>";

    x += "</div>";
  }

  x += "</div>";

  x += "<div class='box'>";
  x += "<h2>Connected CYD Consoles</h2>";

  if (cydCount == 0) {
    x += "<p class='small'>No CYD consoles connected. The Bridge still works independently.</p>";
  }

  for (int i = 0; i < cydCount; i++) {
    char macStr[18];
    macToString(cyds[i].mac, macStr);

    x += "<div class='box'>";
    x += "<p><b>" + String(cyds[i].name) + "</b></p>";
    x += "<p class='small'>" + String(macStr) + "</p>";

    x += "<form method='POST' action='/cyd/remove'>";
    x += "<input type='hidden' name='index' value='" + String(i) + "'>";
    x += "<button class='danger'>REMOVE</button>";
    x += "</form>";

    x += "</div>";
  }

  x += "</div>";

  x += "</body></html>";

  server.send(200, "text/html", x);
}

// ---------------- CYD FUNCTIONS ----------------

void acceptPairing() {
  int index = server.arg("index").toInt();

  if (index < 0 || index >= pendingCount || cydCount >= MAX_CYDS) {
    server.sendHeader("Location", "/");
    server.send(303);
    return;
  }

  memcpy(cyds[cydCount].mac, pending[index].mac, 6);
  strncpy(cyds[cydCount].name, pending[index].name, sizeof(cyds[cydCount].name) - 1);
  cyds[cydCount].name[sizeof(cyds[cydCount].name) - 1] = 0;
  cyds[cydCount].lastSeen = millis();
  cydCount++;

  for (int i = index; i < pendingCount - 1; i++) {
    pending[i] = pending[i + 1];
  }

  pendingCount--;

  prefs.begin("bridge", false);
  prefs.putBytes("cyds", cyds, sizeof(cyds));
  prefs.putInt("cydcount", cydCount);
  prefs.end();

  webMessage = "CYD ACCEPTED";
  webMessageUntil = millis() + 3000;

  drawOLED();

  server.sendHeader("Location", "/");
  server.send(303);
}

void rejectPairing() {
  int index = server.arg("index").toInt();

  if (index < 0 || index >= pendingCount) {
    server.sendHeader("Location", "/");
    server.send(303);
    return;
  }

  for (int i = index; i < pendingCount - 1; i++) {
    pending[i] = pending[i + 1];
  }

  pendingCount--;

  webMessage = "CYD PAIRING REJECTED";
  webMessageUntil = millis() + 3000;

  drawOLED();

  server.sendHeader("Location", "/");
  server.send(303);
}

void removeCyd() {
  int index = server.arg("index").toInt();

  if (index < 0 || index >= cydCount) {
    server.sendHeader("Location", "/");
    server.send(303);
    return;
  }

  for (int i = index; i < cydCount - 1; i++) {
    cyds[i] = cyds[i + 1];
  }

  cydCount--;

  prefs.begin("bridge", false);
  prefs.putBytes("cyds", cyds, sizeof(cyds));
  prefs.putInt("cydcount", cydCount);
  prefs.end();

  webMessage = "CYD REMOVED";
  webMessageUntil = millis() + 3000;

  drawOLED();

  server.sendHeader("Location", "/");
  server.send(303);
}

// ---------------- STAGE 3 ----------------

void saveStage3() {
  String mac = server.arg("stage3mac");

  if (!parseMac(mac, stage3Mac)) {
    webMessage = "INVALID STAGE 3 MAC";
    webMessageUntil = millis() + 3000;

    server.sendHeader("Location", "/");
    server.send(303);
    return;
  }

  stage3Configured = true;
  stage3Connected = false;

  prefs.begin("bridge", false);
  prefs.putString("stage3mac", mac);
  prefs.end();

  webMessage = "STAGE 3 MAC SAVED";
  webMessageUntil = millis() + 3000;

  drawOLED();

  server.sendHeader("Location", "/");
  server.send(303);
}

void loadStage3() {
  prefs.begin("bridge", true);

  String savedMac = prefs.getString("stage3mac", "");
  prefs.end();

  stage3Configured = false;

  if (savedMac.length() == 17) {
    if (parseMac(savedMac, stage3Mac)) {
      stage3Configured = true;
    }
  }
}

// ---------------- MESH SETTINGS ----------------

void saveMesh() {
  int m = server.arg("mesh").toInt();

  if (m < 0 || m >= MAX_MESHES) {
    server.sendHeader("Location", "/");
    server.send(303);
    return;
  }

  String name = server.arg("name");
  String key = server.arg("key");
  String ids = server.arg("ids");

  name.trim();
  key.trim();
  ids.trim();

  name.toCharArray(meshes[m].name, 20);
  key.toCharArray(meshes[m].key, 32);
  ids.toCharArray(meshes[m].expectedIds, 99);

  meshes[m].enabled = server.arg("enabled") == "1";
  meshes[m].lossTimeout = server.arg("timeout").toInt();

  if (meshes[m].lossTimeout < 2) {
    meshes[m].lossTimeout = 10;
  }

  char keyName[24];
  char keyKey[24];
  char keyIds[24];
  char keyEnabled[24];
  char keyTimeout[24];

  snprintf(keyName, sizeof(keyName), "mesh%dname", m);
  snprintf(keyKey, sizeof(keyKey), "mesh%dkey", m);
  snprintf(keyIds, sizeof(keyIds), "mesh%dids", m);
  snprintf(keyEnabled, sizeof(keyEnabled), "mesh%denabled", m);
  snprintf(keyTimeout, sizeof(keyTimeout), "mesh%dtimeout", m);

  prefs.begin("bridge", false);

  prefs.putString(keyName, name);
  prefs.putString(keyKey, key);
  prefs.putString(keyIds, ids);
  prefs.putBool(keyEnabled, meshes[m].enabled);
  prefs.putUShort(keyTimeout, meshes[m].lossTimeout);

  prefs.end();

  webMessage = "MESH " + String(m + 1) + " SAVED";
  webMessageUntil = millis() + 3000;

  server.sendHeader("Location", "/");
  server.send(303);
}

void loadMeshes() {
  prefs.begin("bridge", true);

  for (int m = 0; m < MAX_MESHES; m++) {
    char keyName[24];
    char keyKey[24];
    char keyIds[24];
    char keyEnabled[24];
    char keyTimeout[24];

    snprintf(keyName, sizeof(keyName), "mesh%dname", m);
    snprintf(keyKey, sizeof(keyKey), "mesh%dkey", m);
    snprintf(keyIds, sizeof(keyIds), "mesh%dids", m);
    snprintf(keyEnabled, sizeof(keyEnabled), "mesh%denabled", m);
    snprintf(keyTimeout, sizeof(keyTimeout), "mesh%dtimeout", m);

    String defaultName = "MESH " + String(m + 1);

    String name = prefs.getString(keyName, defaultName);
    String key = prefs.getString(keyKey, "CHANGE-MESH-KEY");
    String ids = prefs.getString(keyIds, "");

    name.toCharArray(meshes[m].name, 20);
    key.toCharArray(meshes[m].key, 32);
    ids.toCharArray(meshes[m].expectedIds, 99);

    meshes[m].enabled = prefs.getBool(keyEnabled, m == 0);
    meshes[m].alarmEnabled = true;
    meshes[m].lossTimeout = prefs.getUShort(keyTimeout, 10);
  }

  prefs.end();
}

// ---------------- SYSTEM CONTROL ----------------

void activateSystem() {
  String pin = server.arg("pin");

  if (pin != String(controlPin)) {
    webMessage = "INCORRECT PIN";
    webMessageUntil = millis() + 3000;

    server.sendHeader("Location", "/");
    server.send(303);
    return;
  }

  systemArmed = true;

  prefs.begin("bridge", false);
  prefs.putBool("armed", true);
  prefs.end();

  webMessage = "SYSTEM ACTIVATED";
  webMessageUntil = millis() + 3000;

  drawOLED();

  server.sendHeader("Location", "/");
  server.send(303);
}

void deactivateSystem() {
  String pin = server.arg("pin");

  if (pin != String(controlPin)) {
    webMessage = "INCORRECT PIN";
    webMessageUntil = millis() + 3000;

    server.sendHeader("Location", "/");
    server.send(303);
    return;
  }

  systemArmed = false;

  prefs.begin("bridge", false);
  prefs.putBool("armed", false);
  prefs.end();

  webMessage = "SYSTEM DEACTIVATED";
  webMessageUntil = millis() + 3000;

  drawOLED();

  server.sendHeader("Location", "/");
  server.send(303);
}

void acknowledgeAlarms() {
  clearAlarms();

  webMessage = "ALARMS CLEARED";
  webMessageUntil = millis() + 3000;

  drawOLED();

  server.sendHeader("Location", "/");
  server.send(303);
}

// ---------------- NODE PROCESSING ----------------

void processNodePacket(
  uint8_t meshId,
  uint8_t nodeId,
  uint32_t sequence,
  uint8_t sensorMask,
  uint8_t alarmFlags,
  float *sensors
) {
  if (meshId < 1 || meshId > MAX_MESHES) {
    return;
  }

  if (nodeId < 1 || nodeId > MAX_NODES) {
    return;
  }

  uint8_t m = meshId - 1;

  if (!meshes[m].enabled) {
    return;
  }

  NodeState &node = nodes[m][nodeId];

  if (node.seen && node.lastSequence == sequence) {
    return;
  }

  node.seen = true;
  node.lossAlarm = false;
  node.lastSequence = sequence;
  node.lastSeenMs = millis();
  node.packets++;

  node.sensorMask = sensorMask;

  for (int s = 0; s < MAX_SENSORS; s++) {
    node.sensors[s] = sensors[s];
  }

  node.radarAlarm = alarmFlags & ALARM_RADAR;
  node.tamperAlarm = alarmFlags & ALARM_TAMPER;
  node.sensorFault = alarmFlags & ALARM_SENSOR_FAULT;
  node.lowBattery = alarmFlags & ALARM_LOW_BATTERY;
  node.attackAlarm = alarmFlags & ALARM_ATTACK;
  node.healthAlarm = alarmFlags & ALARM_HEALTH;
  node.attentionAlarm = alarmFlags & ALARM_ATTENTION;
  node.fallAlarm = alarmFlags & ALARM_FALL;
  node.heartRateAlarm = alarmFlags & ALARM_HEART_RATE;

  if (node.tamperAlarm) {
    setAlarm(String(meshes[m].name) + " NODE " + String(nodeId) + " TAMPER", true);
  }

  if (node.radarAlarm && meshes[m].alarmEnabled) {
    setAlarm(String(meshes[m].name) + " NODE " + String(nodeId) + " RADAR ALARM", false);
  }

  if (node.sensorFault) {
    setAlarm(String(meshes[m].name) + " NODE " + String(nodeId) + " SENSOR FAULT", false);
  }

  if (node.lowBattery) {
    setAlarm(String(meshes[m].name) + " NODE " + String(nodeId) + " LOW BATTERY", false);
  }
}

// ---------------- ESP-NOW ----------------

void onEspNowReceive(
  const esp_now_recv_info_t *info,
  const uint8_t *data,
  int len
) {
  if (len == sizeof(BridgePairingRequest)) {
    BridgePairingRequest request;
    memcpy(&request, data, sizeof(request));

    for (int i = 0; i < cydCount; i++) {
      if (memcmp(cyds[i].mac, info->src_addr, 6) == 0) {
        cyds[i].lastSeen = millis();

        BridgeAck ack;
        strncpy(ack.bridgeName, "GHOST BRIDGE", sizeof(ack.bridgeName) - 1);
        ack.bridgeName[sizeof(ack.bridgeName) - 1] = 0;
        ack.accepted = true;

        esp_now_send(
          info->src_addr,
          (uint8_t *)&ack,
          sizeof(ack)
        );

        return;
      }
    }

    for (int i = 0; i < pendingCount; i++) {
      if (memcmp(pending[i].mac, info->src_addr, 6) == 0) {
        pending[i].receivedAt = millis();
        return;
      }
    }

    if (pendingCount >= MAX_PENDING) {
      return;
    }

    memcpy(pending[pendingCount].mac, info->src_addr, 6);
    strncpy(pending[pendingCount].name, request.cydName, sizeof(pending[pendingCount].name) - 1);
    pending[pendingCount].name[sizeof(pending[pendingCount].name) - 1] = 0;
    pending[pendingCount].receivedAt = millis();
    pendingCount++;

    webMessage = "NEW CYD PAIRING REQUEST";
    webMessageUntil = millis() + 3000;

    drawOLED();
    return;
  }

  if (len == sizeof(BridgeRadarSummary)) {
    BridgeRadarSummary packet;
    memcpy(&packet, data, sizeof(packet));

    for (int i = 0; i < cydCount; i++) {
      if (memcmp(cyds[i].mac, info->src_addr, 6) == 0) {
        cyds[i].lastSeen = millis();
        break;
      }
    }

    float sensors[MAX_SENSORS] = {0};

    processNodePacket(
      packet.meshId,
      packet.nodeId,
      millis(),
      0,
      0,
      sensors
    );

    if (stage3Configured) {
      esp_now_send(
        stage3Mac,
        (uint8_t *)&packet,
        sizeof(packet)
      );
    }

    return;
  }

  if (len == sizeof(Stage1Packet)) {
    Stage1Packet packet;
    memcpy(&packet, data, sizeof(packet));

    if (packet.magic != STAGE1_MAGIC) {
      return;
    }

    if (packet.version != STAGE1_VERSION) {
      return;
    }

    processNodePacket(
      packet.meshId,
      packet.nodeId,
      packet.sequence,
      packet.sensorMask,
      packet.alarmFlags,
      packet.sensors
    );

    if (stage3Configured) {
      esp_now_send(
        stage3Mac,
        (uint8_t *)&packet,
        sizeof(packet)
      );
    }
  }
}

// ---------------- NODE LOSS ----------------

void checkNodeLoss() {
  static unsigned long lastCheck = 0;

  if (millis() - lastCheck < 1000) {
    return;
  }

  lastCheck = millis();

  for (int m = 0; m < MAX_MESHES; m++) {
    if (!meshes[m].enabled) {
      continue;
    }

    unsigned long timeout = meshes[m].lossTimeout * 1000UL;

    for (int n = 1; n <= MAX_NODES; n++) {
      if (!expectedNode(m, n)) {
        continue;
      }

      NodeState &node = nodes[m][n];

      if (!node.seen) {
        continue;
      }

      if (!node.lossAlarm && millis() - node.lastSeenMs > timeout) {
        node.lossAlarm = true;

        setAlarm(
          String(meshes[m].name) + " NODE " + String(n) + " NODE LOSS",
          true
        );
      }
    }
  }
}

// ---------------- LOAD SAVED DATA ----------------

void loadSavedData() {
  prefs.begin("bridge", true);

  cydCount = prefs.getInt("cydcount", 0);

  if (cydCount < 0 || cydCount > MAX_CYDS) {
    cydCount = 0;
  }

  if (cydCount > 0) {
    prefs.getBytes("cyds", cyds, sizeof(cyds));
  }

  systemArmed = prefs.getBool("armed", false);

  String savedPin = prefs.getString("controlpin", "2468");
  savedPin.toCharArray(controlPin, 12);

  prefs.end();
}

// ---------------- SETUP ----------------

void setup() {
  Serial.begin(115200);

  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
    Serial.println("OLED NOT FOUND");
  }

  drawOLED();

  loadSavedData();
  loadMeshes();
  loadStage3();

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASS);

  if (esp_now_init() == ESP_OK) {
    esp_now_register_recv_cb(onEspNowReceive);
  }

  ArduinoOTA.setHostname("Ghost-L2-Bridge");
  ArduinoOTA.begin();

  server.on("/", HTTP_GET, dashboardPage);

  server.on("/pair/accept", HTTP_POST, acceptPairing);
  server.on("/pair/reject", HTTP_POST, rejectPairing);
  server.on("/cyd/remove", HTTP_POST, removeCyd);

  server.on("/stage3/save", HTTP_POST, saveStage3);

  server.on("/mesh/save", HTTP_POST, saveMesh);

  server.on("/system/activate", HTTP_POST, activateSystem);
  server.on("/system/deactivate", HTTP_POST, deactivateSystem);
  server.on("/alarm/clear", HTTP_POST, acknowledgeAlarms);

  server.begin();

  Serial.println("GHOST L2 BRIDGE STARTED");
  Serial.print("BRIDGE MAC: ");
  Serial.println(WiFi.macAddress());
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
}

// ---------------- LOOP ----------------

void loop() {
  server.handleClient();
  ArduinoOTA.handle();
  checkNodeLoss();
}