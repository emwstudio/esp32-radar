#include "net_config.h"
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <lvgl.h>
#include <time.h>

// 默认雷达中心：杭州萧山国际机场（配网网页可覆盖）
static double g_lat = 30.2295;
static double g_lon = 120.4344;
static float g_range = 120.0f;

static Preferences prefs;
static WebServer server(80);
static DNSServer dns;
static bool portalActive = false;
static char g_status[96] = "boot";

void net_status(char *buf, size_t n) { snprintf(buf, n, "%s", g_status); }

void net_center(double &lat, double &lon, float &rangeNm)
{
    lat = g_lat;
    lon = g_lon;
    rangeNm = g_range;
}

static const char PORTAL_HTML[] PROGMEM = R"rawliteral(<!doctype html><html><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>HGH Radar Setup</title>
<style>body{font-family:sans-serif;background:#04110e;color:#9fe8c8;max-width:420px;margin:24px auto;padding:0 16px}
h1{color:#6effbe;font-size:22px}label{display:block;margin-top:14px;font-size:13px;color:#4a9484}
input{width:100%;box-sizing:border-box;margin-top:4px;padding:10px;border-radius:8px;border:1px solid #266258;background:#062018;color:#c8ffe0;font-size:16px}
button{margin-top:20px;width:100%;padding:12px;border:0;border-radius:8px;background:#28c78f;color:#02120c;font-size:17px;font-weight:700}
p{color:#4a9484;font-size:12px}</style></head><body>
<h1>HGH RADAR 配网</h1>
<form method="POST" action="/save">
<label>WiFi 名称 (SSID)<input name="ssid" required></label>
<label>WiFi 密码<input name="pass" type="password"></label>
<label>雷达中心纬度（留空 = 杭州萧山机场 30.2295）<input name="lat" placeholder="30.2295"></label>
<label>雷达中心经度（留空 = 120.4344）<input name="lon" placeholder="120.4344"></label>
<label>雷达半径 NM（留空 = 120）<input name="range" placeholder="120"></label>
<button type="submit">保存并重启</button>
</form>
<p>提交后设备会重启并连接 WiFi。连不上时会重新打开本热点。</p>
</body></html>)rawliteral";

static void handleRoot()
{
    server.send_P(200, "text/html", PORTAL_HTML);
}

static void handleSave()
{
    String ssid = server.arg("ssid");
    String pass = server.arg("pass");
    String lat = server.arg("lat");
    String lon = server.arg("lon");
    String range = server.arg("range");
    prefs.begin("radar", false);
    prefs.putString("ssid", ssid);
    prefs.putString("pass", pass);
    if (lat.length()) prefs.putDouble("lat", lat.toDouble());
    if (lon.length()) prefs.putDouble("lon", lon.toDouble());
    if (range.length()) prefs.putFloat("range", range.toFloat());
    prefs.end();
    server.send(200, "text/html",
                "<meta charset='utf-8'><body style='background:#04110e;color:#9fe8c8;font-family:sans-serif;text-align:center;padding-top:60px'>"
                "<h2>已保存，设备重启中…</h2><p>本热点即将关闭，请切回你的 WiFi。</p></body>");
    Serial.println("[net] config saved, rebooting");
    delay(1500);
    ESP.restart();
}

static void handleNotFound() //  captive portal：所有域名都指回配置页
{
    server.sendHeader("Location", "http://192.168.4.1/", true);
    server.send(302, "text/plain", "");
}

static void startPortal()
{
    portalActive = true;
    snprintf(g_status, sizeof(g_status), "SETUP: join HGH-RADAR -> 192.168.4.1");
    WiFi.mode(WIFI_AP);
    WiFi.softAP("HGH-RADAR");
    dns.start(53, "*", IPAddress(192, 168, 4, 1));
    server.on("/", handleRoot);
    server.on("/save", HTTP_POST, handleSave);
    server.onNotFound(handleNotFound);
    server.begin();
    Serial.println("[net] portal up: HGH-RADAR 192.168.4.1");
    while (true) // 直到 handleSave 触发重启
    {
        dns.processNextRequest();
        server.handleClient();
        lv_timer_handler();
        delay(2);
    }
}

void net_setup()
{
    prefs.begin("radar", true);
    String ssid = prefs.getString("ssid", "");
    String pass = prefs.getString("pass", "");
    g_lat = prefs.getDouble("lat", 30.2295);
    g_lon = prefs.getDouble("lon", 120.4344);
    g_range = prefs.getFloat("range", 120.0f);
    prefs.end();

    if (!ssid.length())
    {
        Serial.println("[net] no saved ssid -> portal");
        startPortal(); // 不返回
    }

    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), pass.c_str());
    Serial.printf("[net] connecting to %s", ssid.c_str());
    uint32_t t0 = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(400);
        Serial.print(".");
        lv_timer_handler();
        if (millis() - t0 > 20000)
        {
            Serial.println("\n[net] connect failed -> portal");
            WiFi.disconnect(true);
            startPortal(); // 不返回
        }
    }
    Serial.printf("\n[net] connected, ip=%s rssi=%d\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
    snprintf(g_status, sizeof(g_status), "NET %s", WiFi.localIP().toString().c_str());

    configTzTime("CST-8", "ntp.aliyun.com", "ntp.tencent.com", "pool.ntp.org");
    struct tm tmv;
    uint32_t t1 = millis();
    while (!getLocalTime(&tmv, 500))
    {
        if (millis() - t1 > 12000)
        {
            Serial.println("[net] NTP sync FAILED (继续跑，日期可能对不上)");
            break;
        }
    }
    if (net_time_ok())
    {
        char d[11];
        net_today(d);
        Serial.printf("[net] NTP ok, today=%s\n", d);
    }
}

bool net_time_ok()
{
    time_t now = time(nullptr);
    return now > 1700000000; // 2023 之后即认为已同步
}

bool net_today(char out[11])
{
    if (!net_time_ok())
        return false;
    struct tm tmv;
    if (!getLocalTime(&tmv, 100))
        return false;
    snprintf(out, 11, "%04d-%02d-%02d", tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday);
    return true;
}
