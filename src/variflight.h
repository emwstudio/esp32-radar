#pragma once
#include <Arduino.h>

// 航线查询结果里的一条航班（只要四个字段）
struct VfRouteFlight
{
    char fnum[12];  // 航班号 MU5157
    char anum[8];   // 机尾号 B328X（可能为空）
    char state[16]; // FlightState：计划/起飞/到达/取消...（UTF-8 中文）
    char dep[5];
    char arr[5];
};

// 实时位置
struct VfPosition
{
    bool ok = false;
    char fnum[12];
    char dep[5];
    char arr[5];
    double lat = 0, lon = 0;
    float speedKmh = 0;  // 公里/小时
    float heightM = 0;   // 米
    float angle = 0;     // 航向角，0=北，顺时针
    uint32_t updatetime = 0;
};

// 查询某天 dep->arr 航线的全部航班（detail=full，含机尾号）。
// 返回是否成功；out 最多填 maxOut 条，count 为实际条数。
bool vf_search_route(const char *date, const char *dep, const char *arr,
                     VfRouteFlight *out, int maxOut, int *count);

// 按机尾号查实时位置。ok=false 表示查询失败或无数据。
bool vf_get_position(const char *anum, VfPosition &out);

// 最近一次 API 业务错误（isError=true 的文本，纯 ASCII）；无错误返回空串
const char *vf_last_error(void);
