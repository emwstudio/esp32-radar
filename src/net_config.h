#pragma once
#include <Arduino.h>

// 连接 WiFi（NVS 里有已保存的凭据则直连）；失败则开 HGH-RADAR 热点 + 配网网页，
// 用户提交后写入 NVS 并重启。函数在 WiFi 连上前不返回（配网期间持续喂 LVGL）。
void net_setup();

// NTP 是否已同步成功
bool net_time_ok();

// 今天日期 "YYYY-MM-DD"（NTP 未同步时返回 false）
bool net_today(char out[11]);

// 雷达中心配置（NVS 覆盖值或默认 HGH）
void net_center(double &lat, double &lon, float &rangeNm);

// WiFi 状态简述（给字幕条用）
void net_status(char *buf, size_t n);
