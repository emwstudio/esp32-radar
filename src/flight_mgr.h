#pragma once
#include <Arduino.h>

#define MAX_TRACKS 12

// 一架被跟踪的飞机（实时数据快照，UI 侧再做坐标换算和平滑）
struct TrackFlight
{
    bool used = false;
    char anum[8] = "";
    char fnum[12] = "";
    char dep[5] = "";
    char arr[5] = "";
    double lat = 0, lon = 0;
    float speedKmh = 0;
    float heightM = 0;
    float angle = 0;      // 航向角 0=北 顺时针
    uint32_t updatetime = 0;
    uint32_t lastPoll = 0;   // millis()，上次轮询时刻
    uint32_t lastSeen = 0;   // millis()，上次成功拿到位置
    uint8_t failCount = 0;
};

void flight_mgr_start();

// 拷贝当前跟踪列表（线程安全），返回条数
int flight_mgr_snapshot(TrackFlight *out, int maxN);

// 状态行（给字幕条）
void flight_mgr_status(char *buf, size_t n);
