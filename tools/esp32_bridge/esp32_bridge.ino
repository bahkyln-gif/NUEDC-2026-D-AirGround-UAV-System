/*
 * ESP32 桥接固件（Arduino）
 * 功能：小车 UART(115200) ↔ WiFi UDP 双向透传/协议转换
 *   S,<MODE>         → UDP UAV_START（发往无人机 14552）
 *   C,<mode>,x,y,s,v,state → UDP CAR_TELEMETRY（发往地面站 14551）
 *   F,<lap_time>     → UDP CAR_DONE
 *   UDP GS_CMD(ESTOP/RESUME) → 串口 G,ESTOP / G,RESUME
 *
 * 接线：ESP32 TX → 小车 RX(PA10)，ESP32 RX → 小车 TX(PA9)，共地
 */
#include <WiFi.h>
#include <WiFiUdp.h>

// ================= 配置 =================
const char *SSID = "NUEDC-2026";
const char *PASS = "12345678";
IPAddress GS_IP(192, 168, 1, 100);   // 地面站
IPAddress UAV_IP(192, 168, 1, 101);  // 无人机伴机
const uint16_t GS_PORT = 14551;
const uint16_t UAV_PORT = 14552;

WiFiUDP udp;
String line = "";

void sendUDP(IPAddress ip, uint16_t port, const String &s) {
  udp.beginPacket(ip, port);
  udp.print(s);
  udp.endPacket();
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID, PASS);
  int t = 0;
  while (WiFi.status() != WL_CONNECTED && t < 50) { delay(200); t++; }
  udp.begin(GS_PORT);
}

void handleSerialLine(const String &l) {
  if (l.startsWith("S,")) {
    String mode = l.substring(2);
    mode.trim();
    // UAV_START JSON
    String json = String("{\"type\":\"UAV_START\",\"from\":\"car\",\"data\":{\"mode\":\"") + mode + "\"}}";
    sendUDP(UAV_IP, UAV_PORT, json);
    sendUDP(GS_IP, GS_PORT, json);   // 同步给地面站
  } else if (l.startsWith("C,")) {
    // C,<mode>,<x>,<y>,<s>,<v>,<state>
    int p1 = l.indexOf(',', 2);
    int p2 = l.indexOf(',', p1 + 1);
    int p3 = l.indexOf(',', p2 + 1);
    int p4 = l.indexOf(',', p3 + 1);
    int p5 = l.indexOf(',', p4 + 1);
    int p6 = l.indexOf(',', p5 + 1);
    String mode = l.substring(2, p1);
    String x = l.substring(p1 + 1, p2);
    String y = l.substring(p2 + 1, p3);
    String s = l.substring(p3 + 1, p4);
    String v = l.substring(p4 + 1, p5);
    String state = l.substring(p6 + 1);
    state.trim();
    String json = String("{\"type\":\"CAR_TELEMETRY\",\"from\":\"car\",\"data\":{") +
                  "\"mode\":\"" + mode + "\"," +
                  "\"x\":" + x + ",\"y\":" + y + ",\"s\":" + s + ",\"v\":" + v +
                  ",\"state\":\"" + state + "\"}}";
    sendUDP(GS_IP, GS_PORT, json);
  } else if (l.startsWith("F,")) {
    String lap = l.substring(2);
    lap.trim();
    String json = String("{\"type\":\"CAR_DONE\",\"from\":\"car\",\"data\":{\"lap_time\":") + lap + "}}";
    sendUDP(GS_IP, GS_PORT, json);
  }
}

void handleUDP() {
  int n = udp.parsePacket();
  if (n <= 0) return;
  char buf[256];
  int len = udp.read(buf, sizeof(buf) - 1);
  buf[len] = 0;
  String msg(buf);
  if (msg.indexOf("\"ESTOP\"") >= 0) {
    Serial.println("G,ESTOP");
  } else if (msg.indexOf("\"RESUME\"") >= 0) {
    Serial.println("G,RESUME");
  }
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      handleSerialLine(line);
      line = "";
    } else if (line.length() < 100) {
      line += c;
    }
  }
  handleUDP();
}
