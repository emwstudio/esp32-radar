#include "flight_mgr.h"
#include "variflight.h"
#include "net_config.h"
#include "announce.h"
#include <WiFi.h>

// ================= 演示模式开关 =================
// 1 = 本地模拟航班（不消耗 Variflight 配额，演示用）
// 0 = 真实 Variflight 数据（需要账号有余额）
#define SIM_FLIGHTS 1

#if !SIM_FLIGHTS
// 扫描的航线：以 HGH 为枢纽的热门方向（双向）。
// 注意：Variflight 没有"按地理范围查航班"的接口，只能靠航线表发现飞机。
static const char *CITIES[] = {"PEK", "CAN", "SZX", "CKG", "CTU", "XIY", "KMG", "XMN"};
#define N_CITIES (sizeof(CITIES) / sizeof(CITIES[0]))
static const char *HUB = "HGH";

#define SCAN_INTERVAL_MS (5 * 60 * 1000) // 全航线扫描周期（每次 16 个计费调用）
#define POLL_INTERVAL_MS (45 * 1000)     // 单架飞机位置轮询周期（每次 1 个计费调用）
#define CALL_GAP_MS 400                  // 调用间隔，避免打得太快
#define STALE_MS (10 * 60 * 1000)        // 10 分钟没更新就移除
#endif

static TrackFlight tracks[MAX_TRACKS];
static SemaphoreHandle_t mtx;
static char g_status[128] = "flight mgr init";
static uint32_t lastScan = 0;
static uint32_t lastScanOk = 0;
static int scanCount = 0;

int flight_mgr_snapshot(TrackFlight *out, int maxN)
{
    if (!mtx) // flight_mgr_start 之前（如配网 portal 阶段）直接返回空
        return 0;
    int n = 0;
    if (xSemaphoreTake(mtx, pdMS_TO_TICKS(50)) == pdTRUE)
    {
        for (int i = 0; i < MAX_TRACKS && n < maxN; i++)
            if (tracks[i].used)
                out[n++] = tracks[i];
        xSemaphoreGive(mtx);
    }
    return n;
}

void flight_mgr_status(char *buf, size_t n)
{
    snprintf(buf, n, "%s", g_status);
}

static TrackFlight *find_track(const char *anum)
{
    for (int i = 0; i < MAX_TRACKS; i++)
        if (tracks[i].used && strcmp(tracks[i].anum, anum) == 0)
            return &tracks[i];
    return nullptr;
}

static TrackFlight *alloc_track()
{
    for (int i = 0; i < MAX_TRACKS; i++)
        if (!tracks[i].used)
            return &tracks[i];
    return nullptr;
}

#if !SIM_FLIGHTS
// 全航线扫描：发现"起飞"（在飞）状态的航班，把机尾号纳入跟踪
static void scan_routes(const char *date)
{
    static VfRouteFlight res[48];
    int found = 0;
    for (int ci = 0; ci < (int)N_CITIES; ci++)
    {
        for (int dir = 0; dir < 2; dir++)
        {
            const char *dep = dir == 0 ? HUB : CITIES[ci];
            const char *arr = dir == 0 ? CITIES[ci] : HUB;
            int cnt = 0;
            bool ok = vf_search_route(date, dep, arr, res, 48, &cnt);
            if (!ok)
            {
                Serial.printf("[mgr] search %s-%s failed\n", dep, arr);
                delay(CALL_GAP_MS);
                continue;
            }
            int inAir = 0;
            for (int i = 0; i < cnt; i++)
            {
                if (strcmp(res[i].state, "起飞") != 0 || !res[i].anum[0])
                    continue;
                inAir++;
                xSemaphoreTake(mtx, portMAX_DELAY);
                TrackFlight *t = find_track(res[i].anum);
                if (!t)
                    t = alloc_track();
                if (t)
                {
                    if (!t->used)
                    {
                        memset(t, 0, sizeof(*t));
                        t->used = true;
                        snprintf(t->anum, sizeof(t->anum), "%s", res[i].anum);
                    }
                    snprintf(t->fnum, sizeof(t->fnum), "%s", res[i].fnum);
                    snprintf(t->dep, sizeof(t->dep), "%s", res[i].dep);
                    snprintf(t->arr, sizeof(t->arr), "%s", res[i].arr);
                }
                xSemaphoreGive(mtx);
            }
            found += inAir;
            Serial.printf("[mgr] %s-%s: %d flights, %d in-air\n", dep, arr, cnt, inAir);
            delay(CALL_GAP_MS);
        }
    }
    lastScan = millis();
    scanCount++;
    Serial.printf("[mgr] scan #%d done, %d in-air candidates\n", scanCount, found);
}

// 轮询所有到期飞机的实时位置
static void poll_positions()
{
    for (int i = 0; i < MAX_TRACKS; i++)
    {
        bool due = false;
        char anum[8];
        xSemaphoreTake(mtx, portMAX_DELAY);
        if (tracks[i].used && millis() - tracks[i].lastPoll > POLL_INTERVAL_MS)
        {
            due = true;
            tracks[i].lastPoll = millis();
            snprintf(anum, sizeof(anum), "%s", tracks[i].anum);
        }
        xSemaphoreGive(mtx);
        if (!due)
            continue;

        VfPosition pos;
        bool ok = vf_get_position(anum, pos);
        TrackFlight snapForAnn;
        bool doAnn = false;
        float distKm = 0;
        xSemaphoreTake(mtx, portMAX_DELAY);
        if (ok && pos.ok)
        {
            tracks[i].lat = pos.lat;
            tracks[i].lon = pos.lon;
            tracks[i].speedKmh = pos.speedKmh;
            tracks[i].heightM = pos.heightM;
            tracks[i].angle = pos.angle;
            tracks[i].updatetime = pos.updatetime;
            if (pos.fnum[0])
                snprintf(tracks[i].fnum, sizeof(tracks[i].fnum), "%s", pos.fnum);
            if (pos.dep[0])
                snprintf(tracks[i].dep, sizeof(tracks[i].dep), "%s", pos.dep);
            if (pos.arr[0])
                snprintf(tracks[i].arr, sizeof(tracks[i].arr), "%s", pos.arr);
            tracks[i].lastSeen = millis();
            tracks[i].failCount = 0;
            Serial.printf("[mgr] %s %s %.4f,%.4f h=%.0fm v=%.0fkm/h ang=%.0f\n",
                          tracks[i].anum, tracks[i].fnum, pos.lat, pos.lon,
                          pos.heightM, pos.speedKmh, pos.angle);
            // 落地/在地面（速度很低）就不再显示
            if (pos.speedKmh < 80 && pos.heightM < 200)
            {
                Serial.printf("[mgr] %s on ground, drop\n", tracks[i].anum);
                tracks[i].used = false;
            }
            else
            {
                // 距中心距离（等距圆柱近似），近了交给播报模块（它自己冷却去重）
                double clat, clon;
                float rng;
                net_center(clat, clon, rng);
                double dx = (pos.lon - clon) * cos((pos.lat + clat) * 0.5 * DEG_TO_RAD) * 111.32;
                double dy = (pos.lat - clat) * 110.57;
                distKm = (float)sqrt(dx * dx + dy * dy);
                snapForAnn = tracks[i];
                doAnn = true;
            }
        }
        else
        {
            tracks[i].failCount++;
            Serial.printf("[mgr] %s poll failed (%d)\n", anum, tracks[i].failCount);
            if (tracks[i].failCount >= 4)
                tracks[i].used = false;
        }
        xSemaphoreGive(mtx);
        if (doAnn)
            announce_notify(snapForAnn, distKm); // 内部判断距离阈值和冷却
        delay(CALL_GAP_MS);
    }

    // 清理过期
    xSemaphoreTake(mtx, portMAX_DELAY);
    for (int i = 0; i < MAX_TRACKS; i++)
        if (tracks[i].used && millis() - tracks[i].lastSeen > STALE_MS)
        {
            Serial.printf("[mgr] %s stale, drop\n", tracks[i].anum);
            tracks[i].used = false;
        }
    int n = 0;
    for (int i = 0; i < MAX_TRACKS; i++)
        if (tracks[i].used)
            n++;
    uint32_t nextScan = lastScan + SCAN_INTERVAL_MS - millis();
    if (vf_last_error()[0])
        snprintf(g_status, sizeof(g_status), "API ERR: %.80s", vf_last_error());
    else
        snprintf(g_status, sizeof(g_status), "TRK %d | scan #%d | next %lus",
                 n, scanCount, (unsigned long)(nextScan / 1000));
    xSemaphoreGive(mtx);
}
#endif // !SIM_FLIGHTS

#if SIM_FLIGHTS
// ---------------- 演示模式：本地模拟航班 ----------------
// 航线全部是 HGH 往返真实机场，位置每秒推进，带真实感的高度剖面；
// 不联网、不花钱，飞机进出雷达范围和靠近中心时的播报链路完全真实。
struct SimAirport
{
    const char *code;
    double lat, lon;
};
static const SimAirport SIM_APS[] = {
    {"HGH", 30.2295, 120.4344}, // 0 号固定为枢纽
    {"PEK", 40.0801, 116.5846},
    {"CAN", 23.3924, 113.2988},
    {"SZX", 22.6393, 113.8107},
    {"CKG", 29.7192, 106.6417},
    {"CTU", 30.5785, 103.9471},
    {"XIY", 34.4471, 108.7516},
    {"KMG", 24.9924, 102.7435},
    {"XMN", 24.5440, 118.1277},
};
#define N_SIM_APS (sizeof(SIM_APS) / sizeof(SIM_APS[0]))
#define SIM_COUNT 10           // 同时在空中的模拟飞机数
#define SIM_TIME_SCALE 6.0f    // 演示加速倍率（1 = 真实速度）
#define SIM_STEP_MS 1000       // 位置推进周期

struct SimFlight
{
    bool active = false;
    bool ovr = false;        // 过路航班：穿越本场上空，不在本场起降
    int dep = 0, arr = 0;    // SIM_APS 下标（过路航班时仅用于标签真实性）
    double la1 = 0, lo1 = 0, la2 = 0, lo2 = 0; // 航线端点坐标（过路航班为合成坐标）
    float prog = 0;          // 航程进度 0..1
    float durationS = 3600;  // 全程秒数（真实尺度）
    float routeKm = 1000;    // 航线全长（公里）
    float cruiseM = 10000;   // 巡航高度
    float angle = 0;         // 航向角
    char anum[8] = "";
    char fnum[12] = "";
};
static SimFlight sims[SIM_COUNT];
static const char *SIM_AL[] = {"CA", "MU", "CZ", "HU", "ZH", "MF", "3U", "GJ"};
static uint32_t sim_last = 0;

static float sim_bearing(double la1, double lo1, double la2, double lo2)
{
    double dx = (lo2 - lo1) * cos((la1 + la2) * 0.5 * DEG_TO_RAD);
    double dy = la2 - la1;
    float a = atan2((float)dx, (float)dy) * RAD_TO_DEG;
    if (a < 0)
        a += 360.0f;
    return a;
}

// 生成一架新航班。萧山塔台视角，三种角色：
// 进场机（arr==HGH）出现在距机场 60~215km 的航段上，一路进近、落地后从雷达消失；
// 离场机（dep==HGH）刚离地或爬升中，从屏幕中心附近起飞，飞出雷达范围后换班；
// 过路航班（约 25%）用真实外地航线做标签，坐标是过 HGH 的合成直线，巡航高度穿场而过。
static void sim_spawn(int i)
{
    SimFlight &s = sims[i];
    s.ovr = random(100) < 25;
    if (s.ovr)
    {
        s.dep = 1 + random((long)(N_SIM_APS - 1));
        do
        {
            s.arr = 1 + random((long)(N_SIM_APS - 1));
        } while (s.arr == s.dep);
    }
    else
    {
        s.dep = 0;
        s.arr = 1 + random((long)(N_SIM_APS - 1));
        if (random(2)) // 一半概率反向（飞向 HGH）
        {
            int t = s.dep;
            s.dep = s.arr;
            s.arr = t;
        }
    }
    // 航线全长用两端真实机场间的真实距离，保证速度剖面和全程时长合理
    double rla1 = SIM_APS[s.dep].lat, rlo1 = SIM_APS[s.dep].lon;
    double rla2 = SIM_APS[s.arr].lat, rlo2 = SIM_APS[s.arr].lon;
    double rdx = (rlo2 - rlo1) * cos((rla1 + rla2) * 0.5 * DEG_TO_RAD) * 111.32;
    double rdy = (rla2 - rla1) * 110.57;
    double distKm = sqrt(rdx * rdx + rdy * rdy);

    if (s.ovr)
    {
        // 合成一条过 HGH 的直线航线：随机航向，两端各伸 distKm/2
        float brg = (float)random(0, 360) * DEG_TO_RAD;
        double half = distKm * 0.5;
        double dxk = sin(brg) * half, dyk = cos(brg) * half;
        double hlat = SIM_APS[0].lat, hlon = SIM_APS[0].lon;
        double kx = 111.32 * cos(hlat * DEG_TO_RAD);
        s.la1 = hlat - dyk / 110.57;
        s.lo1 = hlon - dxk / kx;
        s.la2 = hlat + dyk / 110.57;
        s.lo2 = hlon + dxk / kx;
        // 出生时位于过台前后 215km 以内的航段
        float maxDev = (float)(215.0 / distKm);
        if (maxDev > 0.45f)
            maxDev = 0.45f;
        s.prog = 0.5f + ((float)random(-100, 101) / 100.0f) * maxDev;
    }
    else
    {
        s.la1 = rla1;
        s.lo1 = rlo1;
        s.la2 = rla2;
        s.lo2 = rlo2;
        if (s.arr == 0)
        {
            float d = random(60, 215); // 进场：距 HGH 还有 d 公里
            s.prog = 1.0f - d / (float)distKm;
        }
        else
        {
            float d = random(2, 35); // 离场：刚离地/爬升中，必经 30km 播报圈
            s.prog = d / (float)distKm;
        }
    }
    s.prog = constrain(s.prog, 0.001f, 0.98f);
    s.durationS = (float)(distKm / 860.0 * 3600.0); // 按 860km/h 算全程时间
    s.routeKm = (float)distKm;
    s.cruiseM = 8900 + random(0, 9) * 300;
    s.angle = sim_bearing(s.la1, s.lo1, s.la2, s.lo2);
    // 机尾号全局唯一：与其他在飞航班撞号会让 UI 槽位串台（两架共用一个图标）
    for (int tries = 0; tries < 20; tries++)
    {
        snprintf(s.anum, sizeof(s.anum), "B%04d", (int)random(1000, 9999));
        bool dup = false;
        for (int k = 0; k < SIM_COUNT; k++)
            if (k != i && sims[k].active && strcmp(sims[k].anum, s.anum) == 0)
            {
                dup = true;
                break;
            }
        if (!dup)
            break;
    }
    snprintf(s.fnum, sizeof(s.fnum), "%s%d", SIM_AL[random((long)(sizeof(SIM_AL) / sizeof(SIM_AL[0])))],
             (int)random(100, 9999));
    s.active = true;
    Serial.printf("[sim] spawn %s %s %s->%s%s prog=%.2f\n", s.anum, s.fnum,
                  SIM_APS[s.dep].code, SIM_APS[s.arr].code, s.ovr ? " (OVR)" : "", s.prog);
}

static void sim_step()
{
    uint32_t now = millis();
    if (now - sim_last < SIM_STEP_MS)
        return;
    float dt = (now - sim_last) / 1000.0f;
    sim_last = now;

    double clat, clon;
    float rng;
    net_center(clat, clon, rng);

    for (int i = 0; i < SIM_COUNT; i++)
    {
        if (!sims[i].active)
            sim_spawn(i);
        SimFlight &s = sims[i];

        double lat = s.la1 + (s.la2 - s.la1) * s.prog;
        double lon = s.lo1 + (s.lo2 - s.lo1) * s.prog;

        double ddx = (lon - clon) * cos((lat + clat) * 0.5 * DEG_TO_RAD) * 111.32;
        double ddy = (lat - clat) * 110.57;
        float distKm = (float)sqrt(ddx * ddx + ddy * ddy);

        // 速度剖面：进近低速、巡航高速、离场爬升介于中间；过路航班全程巡航
        float v;
        if (s.ovr)
            v = 870.0f;
        else if (distKm < 20.0f)
            v = s.arr == 0 ? 240.0f : 330.0f; // 五边进近 / 刚离地
        else if (distKm < 60.0f)
            v = s.arr == 0 ? 300.0f : 420.0f;
        else if (distKm < 120.0f)
            v = s.arr == 0 ? 460.0f : 600.0f;
        else
            v = 870.0f; // 巡航

        // 按当前真实速度推进航程：雷达上近场飞机移动慢、巡航快
        s.prog += dt * SIM_TIME_SCALE * (v / 3600.0f) / s.routeKm;
        if (s.prog >= 1.0f) // 落地/到达，从雷达消失并换一班新的
        {
            sim_spawn(i);
            continue; // 新航班下一帧再写轨迹，避免新旧数据混搭
        }

        // 进场机触地（距机场 <4km）即从雷达消失——真实雷达在落地后丢失目标。
        // 不做这步的话多架进场机会在圆心叠成一坨：图标重合但标签散开，看起来像"有标签没飞机"
        if (s.arr == 0 && distKm < 4.0f)
        {
            sim_spawn(i);
            continue;
        }

        // 高度剖面：贴着机场由距离决定（落地=0、起飞=0 爬升），远处按航程爬升到巡航；过路航班全程巡航高度
        float h;
        if (s.ovr)
            h = s.cruiseM;
        else if (distKm < 90.0f)
            h = s.arr == 0 ? distKm * 90.0f : distKm * 150.0f;
        else if (s.prog < 0.15f)
            h = s.cruiseM * s.prog / 0.15f;
        else if (s.prog > 0.85f)
            h = s.cruiseM * (1.0f - s.prog) / 0.15f;
        else
            h = s.cruiseM;
        if (h > s.cruiseM)
            h = s.cruiseM;

        if (distKm > 225.0f) // 离场飞出 120NM(222km) 外环即换班，保持屏幕密度且不让图标长期贴边
        {
            sim_spawn(i);
            continue;
        }

        TrackFlight snap;
        xSemaphoreTake(mtx, portMAX_DELAY);
        TrackFlight &t = tracks[i];
        t.used = true;
        snprintf(t.anum, sizeof(t.anum), "%s", s.anum);
        snprintf(t.fnum, sizeof(t.fnum), "%s", s.fnum);
        snprintf(t.dep, sizeof(t.dep), "%s", SIM_APS[s.dep].code);
        snprintf(t.arr, sizeof(t.arr), "%s", SIM_APS[s.arr].code);
        t.lat = lat;
        t.lon = lon;
        t.heightM = h;
        t.speedKmh = v;
        t.angle = s.angle;
        t.lastSeen = now;
        snap = t;
        xSemaphoreGive(mtx);

        announce_notify(snap, distKm); // 内部判断 30km 阈值和冷却
    }

    xSemaphoreTake(mtx, portMAX_DELAY);
    snprintf(g_status, sizeof(g_status), "SIM x%d | DEMO MODE", SIM_COUNT);
    xSemaphoreGive(mtx);
}
#endif // SIM_FLIGHTS

static void mgr_task(void *)
{
#if SIM_FLIGHTS
    randomSeed(esp_random());
    sim_last = millis();
    Serial.println("[mgr] SIM mode: local demo flights, no API cost");
#else
    // 等 NTP（查询要带日期）
    while (!net_time_ok())
        delay(1000);
#endif
    char date[11];
    while (true)
    {
        if (WiFi.status() != WL_CONNECTED)
        {
            snprintf(g_status, sizeof(g_status), "WIFI LOST");
            WiFi.reconnect();
            delay(5000);
            continue;
        }
#if SIM_FLIGHTS
        sim_step();
        delay(200);
#else
        if (!net_today(date))
        {
            delay(2000);
            continue;
        }
        if (lastScan == 0 || millis() - lastScan > SCAN_INTERVAL_MS)
            scan_routes(date);
        poll_positions();
        delay(1000);
#endif
    }
}

void flight_mgr_start()
{
    mtx = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(mgr_task, "flmgr", 12288, nullptr, 1, nullptr, 0);
    Serial.println("[mgr] task started on core 0");
}
