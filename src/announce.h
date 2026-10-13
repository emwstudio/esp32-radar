#pragma once
#include <Arduino.h>
#include "flight_mgr.h"

// 启动播报任务（core 0）。需要在 WiFi 连通之后调用。
void announce_start();

// 航班接近时由 flight_mgr 调用（内部做冷却去重，重复调用安全）。
// distKm：距雷达中心的公里数。
void announce_notify(const TrackFlight &f, float distKm);

// 播报状态（空串 = 空闲），给字幕条
void announce_status(char *buf, size_t n);

// 调试：跳过冷却/距离判断，直接触发一次测试播报
void announce_test();
