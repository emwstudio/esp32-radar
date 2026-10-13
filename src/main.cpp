// 雷达 UI 静态原型 v3（假数据）
// 画质升级：LVGL 抗锯齿矢量网格/飞机剪影 + canvas 只做扫描渐变与加性辉光
#include <Arduino.h>
#include <lvgl.h>
#include "Arduino_GFX_Library.h"
#include "pin_config.h"
#include "TouchDrvCSTXXX.hpp"
#include "net_config.h"
#include "flight_mgr.h"
#include "audio_player.h"
#include "announce.h"
#include <Wire.h>
#include <WiFi.h>
#include <math.h>
#include <time.h>

// 粗体字体（lv_font_conv 从 Arial Bold 生成，src/fonts/）
LV_FONT_DECLARE(font_bold_44); // 仅 NESW，罗盘字母
LV_FONT_DECLARE(font_bold_28); // ASCII，四角大数值
LV_FONT_DECLARE(font_bold_24); // ASCII，RADIO 角状态字（大写字母视觉补偿）
LV_FONT_DECLARE(font_bold_18); // ASCII，详情卡/跑道/环数值
LV_FONT_DECLARE(font_bold_14); // ASCII，备用
LV_FONT_DECLARE(font_bold_12); // ASCII，飞机数据标签

// ---------- 显示 ----------
Arduino_DataBus *bus = new Arduino_ESP32QSPI(
    LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);
Arduino_CO5300 *gfx = new Arduino_CO5300(
    bus, LCD_RESET, 0, LCD_WIDTH, LCD_HEIGHT, 0, 0, 0, 0);

// ---------- 触摸 ----------
TouchDrvCST92xx touch;
int16_t tx[2], ty[2];

// ---------- LVGL ----------
static lv_disp_draw_buf_t draw_buf;
static const uint32_t BUF_LINES = 30;

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
    lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_drv_t *, lv_indev_data_t *data)
{
    // 直接轮询触摸芯片（15ms 节流）：手指按住期间持续报 PR，
    // 不能靠中断一次性上报，否则 LVGL 看到的就是 1 帧按下即松开，CLICKED 经常丢。
    static uint32_t lastPoll = 0;
    static bool down = false;
    static lv_coord_t lx = 0, ly = 0;
    if (millis() - lastPoll >= 15)
    {
        lastPoll = millis();
        if (touch.getPoint(tx, ty, touch.getSupportTouchPoint()))
        {
            lx = tx[0];
            ly = ty[0];
            down = true;
        }
        else
            down = false;
    }
    data->state = down ? LV_INDEV_STATE_PR : LV_INDEV_STATE_REL;
    if (down)
    {
        data->point.x = lx;
        data->point.y = ly;
    }
}

void lvgl_tick_cb(void *) { lv_tick_inc(2); }

// ---------- 雷达 ----------
#define RADAR_W 480
#define RADAR_H 480
#define RCX 240
#define RCY 240
#define RADIUS 224

// 配色参考 ESP32-Plane-Radar：深藏青底 + 暗绿网格 + 白色字符 + 红色目标
#define COL_BG lv_color_make(4, 10, 28)        // 深藏青底
#define COL_GRID lv_color_make(38, 170, 74)    // 网格绿（三环/十字/统一）
#define COL_TICK lv_color_make(190, 230, 205)  // 刻度
#define COL_TEXT lv_color_make(235, 244, 255)  // 主文字（近白）
#define COL_TEXT_DIM lv_color_make(110, 158, 168) // 次要文字
#define COL_ACCENT lv_color_make(110, 210, 230)   // 跑道标签浅青
#define COL_PLANE lv_color_make(255, 72, 60)   // 红色飞机符号
#define COL_VEC lv_color_make(255, 60, 220)    // 品红速度矢量

static lv_obj_t *canvas;
static uint16_t *cbuf; // PSRAM
static lv_obj_t *card;
static lv_obj_t *card_label;
static lv_obj_t *sel_ring;

// 四角要素（RockBase 风格）：WiFi 信号 / 在跟踪飞机数 / 量程 / 时间
static lv_obj_t *wifi_bars[4];
static lv_obj_t *air_val;
static lv_obj_t *time_val;
static int g_air_count = 0; // radar_frame 里更新
static lv_obj_t *tx_txt;    // 左下角无线电状态（STBY / ON AIR，与其他角同风格）

// 四角要素刷新：WiFi 信号 / 飞机数 / 时间 + TX 指示
static void status_tick(lv_timer_t *)
{
    // 左上 WiFi 信号强度条
    int rssi = WiFi.RSSI(); // 未连接时 0
    int lvl = rssi > -55 ? 4 : rssi > -65 ? 3 : rssi > -75 ? 2 : rssi > -85 ? 1 : 0;
    for (int i = 0; i < 4; i++)
        lv_obj_set_style_bg_color(wifi_bars[i],
                                  i < lvl ? COL_TEXT : lv_color_make(40, 60, 80), 0);
    // 右上在跟踪飞机数
    lv_label_set_text_fmt(air_val, "%d", g_air_count);
    // 右下时间（NTP，CST-8）
    time_t now = time(nullptr);
    struct tm tmv;
    if (localtime_r(&now, &tmv) && tmv.tm_year > 120)
        lv_label_set_text_fmt(time_val, "%02d:%02d", tmv.tm_hour, tmv.tm_min);
}

// 无线电状态走独立快刷（80ms）：和声音严格同步，TX 随功放点亮、播完即回 STBY
static void tx_tick(lv_timer_t *)
{
    if (audio_busy())
    {
        lv_label_set_text(tx_txt, "TX");
        lv_obj_set_style_text_color(tx_txt, lv_color_make(255, 70, 70), 0);
    }
    else
    {
        lv_label_set_text(tx_txt, "STBY");
        lv_obj_set_style_text_color(tx_txt, lv_color_make(90, 210, 140), 0);
    }
}

static float sweep_deg = 90.0f; // 从正北开始

// ---------- 雷达中心配置（默认杭州萧山机场；配网网页可改，boot 时从 NVS 读） ----------
static double CENTER_LAT = 30.2295; // HGH 杭州萧山国际机场
static double CENTER_LON = 120.4344;
static float RANGE_NM = 120.0f; // 外环半径对应的距离（海里）

// UI 侧的飞机槽位：以机尾号为 key，从 FlightManager 快照 reconcile 而来，带平滑插值
struct UiFlight
{
    bool active = false;
    char anum[8] = "";
    float curX = 0, curY = 0, tgtX = 0, tgtY = 0;
    float angle = 0;
    char fnum[12] = "";
    char dep[5] = "";
    char arr[5] = "";
    int alt_ft = 0, spd_kt = 0;
    int opa = 255;    // 淡入淡出：新目标从 0 淡入，消失目标淡出后才释放槽位
    bool dying = false;
};
static UiFlight uiF[MAX_TRACKS];
static char selectedAnum[8] = "";

// 经纬度 → 雷达像素偏移（简易等距圆柱投影，120NM 范围内误差可忽略）
static void geo_to_px(double lat, double lon, float &px, float &py)
{
    float dxNm = (float)((lon - CENTER_LON) * 60.0 * cos(CENTER_LAT * 0.0174533));
    float dyNm = (float)((lat - CENTER_LAT) * 60.0);
    px = dxNm / RANGE_NM * RADIUS;
    py = -dyNm / RANGE_NM * RADIUS;
}

// 每架飞机一个可旋转的 ✈️ 图标（LVGL 图片旋转自带抗锯齿）
static lv_obj_t *plane_imgs[MAX_TRACKS];
// 每架飞机的数据标签（航班号+高度/速度）和速度矢量线
static lv_obj_t *tag_labels[MAX_TRACKS];
static lv_obj_t *vec_lines[MAX_TRACKS];
static lv_point_t vec_pts[MAX_TRACKS][2];
static int tagx[MAX_TRACKS], tagy[MAX_TRACKS]; // 本帧标签目标位置（防重叠后排开）
static bool tagvis[MAX_TRACKS];

// ---------- 飞机图标生成（32x32，带 alpha，机头朝上） ----------
// 造型参考飞常准/航旅地图上的细长喷气客机：尖头 + 细机身 + 大后掠细主翼 + 小尾翼
// 生成时按"到边缘的有符号距离"做 1px 抗锯齿软边
#define PLANE_SZ 40
static uint8_t plane_data[PLANE_SZ * PLANE_SZ * 3];
static lv_img_dsc_t plane_dsc;

// 点 (x,y) 在三角形 (ax,ay)(bx,by)(cx,cy) 内的有符号边距（正=在内，按边长归一化）
static float tri_margin(float x, float y,
                        float ax, float ay, float bx, float by, float cx, float cy)
{
    float e1 = sqrtf((bx - ax) * (bx - ax) + (by - ay) * (by - ay));
    float e2 = sqrtf((cx - bx) * (cx - bx) + (cy - by) * (cy - by));
    float e3 = sqrtf((ax - cx) * (ax - cx) + (ay - cy) * (ay - cy));
    float d1 = ((x - bx) * (ay - by) - (ax - bx) * (y - by)) / (e1 > 0 ? e1 : 1);
    float d2 = ((x - cx) * (by - cy) - (bx - cx) * (y - cy)) / (e2 > 0 ? e2 : 1);
    float d3 = ((x - ax) * (cy - ay) - (cx - ax) * (y - ay)) / (e3 > 0 ? e3 : 1);
    bool neg = (d1 < 0) || (d2 < 0) || (d3 < 0);
    bool pos = (d1 > 0) || (d2 > 0) || (d3 > 0);
    if (neg && pos)
        return -1.0f; // 在外
    float m = fabsf(d1);
    if (fabsf(d2) < m) m = fabsf(d2);
    if (fabsf(d3) < m) m = fabsf(d3);
    return m;
}

static void build_plane_icon()
{
    lv_color_t col = COL_PLANE;
    const float S = PLANE_SZ / 32.0f; // 几何按 32px 设计，放大到 PLANE_SZ 采样
    const float CC = 15.5f;           // 中轴 x（32px 坐标系）
    for (int y = 0; y < PLANE_SZ; y++)
        for (int x = 0; x < PLANE_SZ; x++)
        {
            float ys = (y + 0.5f) / S - 0.5f; // 映射回 32px 设计坐标
            float cx = fabsf((x + 0.5f) / S - 0.5f - CC); // 左右对称，只算半边
            float m = -1.0f;          // 有符号边距，>0 在机身内

            // 机身：细长圆柱 y∈[4,25]，半宽 1.9，尾椎 y 25..28 收成尖
            float fus_half = 1.9f;
            if (ys > 25)
                fus_half = 1.9f - (ys - 25) * 0.63f;
            if (ys >= 4 && ys <= 28)
            {
                float mf = fus_half - cx;              // 横向边距
                float my = (float)min(ys - 4, 28 - ys);  // 纵向边距
                if (my < mf) mf = my;
                if (mf > m) m = mf;
            }
            // 尖头机头：三角形，顶点 (0,-0.5)，底边 y=4 半宽 1.9
            {
                float mt = tri_margin(cx, ys, 0, -0.5f, 1.9f, 4.0f, -1.9f, 4.0f);
                if (mt > m) m = mt;
            }
            // 主翼（大后掠细翼）：(2,10) (14.5,17) (2,15)
            {
                float mw = tri_margin(cx, ys, 2.0f, 10.0f, 14.5f, 17.0f, 2.0f, 15.0f);
                if (mw > m) m = mw;
            }
            // 尾翼（小后掠）：(1.5,22) (7.5,27) (1.5,25.5)
            {
                float mt = tri_margin(cx, ys, 1.5f, 22.0f, 7.5f, 27.0f, 1.5f, 25.5f);
                if (mt > m) m = mt;
            }

            // 1px 抗锯齿软边
            float a = m + 0.5f;
            if (a < 0) a = 0;
            if (a > 1) a = 1;
            uint8_t *p = &plane_data[(y * PLANE_SZ + x) * 3];
            *(lv_color_t *)p = col;
            p[2] = (uint8_t)(a * 255);
        }
    plane_dsc.header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA;
    plane_dsc.header.always_zero = 0;
    plane_dsc.header.reserved = 0;
    plane_dsc.header.w = PLANE_SZ;
    plane_dsc.header.h = PLANE_SZ;
    plane_dsc.data_size = PLANE_SZ * PLANE_SZ * 3;
    plane_dsc.data = plane_data;
}

// ---------- canvas 像素工具 ----------
// 磷光底色（深藏青，与表盘背景一致）：衰减到的下限，整屏有荧光粉底子而不是死黑
#define FLOOR_R 0
#define FLOOR_G 2
#define FLOOR_B 3
#define FLOOR565 ((FLOOR_R << 11) | (FLOOR_G << 5) | FLOOR_B)

// 磷光慢衰减：每帧每通道减 1，衰减到底色为止
static inline void decay_canvas()
{
    uint32_t n = RADAR_W * RADAR_H;
    for (uint32_t i = 0; i < n; i++)
    {
        uint16_t c = cbuf[i];
        if (c != FLOOR565)
        {
            uint16_t r = (c >> 11) & 0x1F;
            uint16_t g = (c >> 5) & 0x3F;
            uint16_t b = c & 0x1F;
            if (r > FLOOR_R) r--;
            if (g > FLOOR_G) g--;
            if (b > FLOOR_B) b--;
            cbuf[i] = (r << 11) | (g << 5) | b;
        }
    }
}

static inline void px(int x, int y, uint16_t c)
{
    if (x >= 0 && x < RADAR_W && y >= 0 && y < RADAR_H)
        cbuf[y * RADAR_W + x] = c;
}

static void draw_line(int x0, int y0, int x1, int y1, uint16_t c)
{
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy;
    while (true)
    {
        px(x0, y0, c);
        if (x0 == x1 && y0 == y1)
            break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

static inline uint16_t sweep565(uint8_t v) // 扫描线青绿
{
    return (((uint16_t)(40 * v / 255) >> 3) << 11) |
           (((uint16_t)v >> 2) << 5) |
           ((uint16_t)(170 * v / 255) >> 3);
}

// 2px 宽线：在主线的垂直方向补画一条偏移 1px 的线。
// 1px Bresenham 线在大半径处相邻角度线之间会漏像素（间距 ~0.6px 但覆盖离散），
// 漏掉的像素衰减后变成余晖里的黑色放射状细线；2px 宽彻底堵住缝隙。
static void draw_line_wide(int x0, int y0, int x1, int y1, uint16_t c)
{
    draw_line(x0, y0, x1, y1, c);
    if (abs(x1 - x0) >= abs(y1 - y0))
        draw_line(x0, y0 + 1, x1, y1 + 1, c);
    else
        draw_line(x0 + 1, y0, x1 + 1, y1, c);
}

// ---------- LVGL 矢量元素 ----------
static lv_obj_t *make_ring(int r, lv_color_t c, lv_opa_t opa, int w)
{
    lv_obj_t *o = lv_obj_create(lv_scr_act());
    lv_obj_set_size(o, 2 * r, 2 * r);
    lv_obj_align(o, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_opa(o, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_color(o, c, 0);
    lv_obj_set_style_border_opa(o, opa, 0);
    lv_obj_set_style_border_width(o, w, 0);
    lv_obj_set_style_radius(o, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_pad_all(o, 0, 0);
    lv_obj_clear_flag(o, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
    return o;
}

static lv_obj_t *make_line_seg(lv_color_t c, lv_opa_t opa, int width)
{
    lv_obj_t *ln = lv_line_create(lv_scr_act());
    lv_obj_set_style_line_color(ln, c, 0);
    lv_obj_set_style_line_opa(ln, opa, 0);
    lv_obj_set_style_line_width(ln, width, 0);
    lv_obj_set_style_line_rounded(ln, true, 0);
    lv_obj_clear_flag(ln, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
    return ln;
}

// ---------- 帧更新 ----------
static void radar_frame(lv_timer_t *)
{
    decay_canvas();

    // 扫描楔形渐变：0.15° 步进（2px 宽线，外圈无缝隙），二次方衰减
    for (int i = 0; i < 120; i++)
    {
        float a = (sweep_deg + i * 0.15f) * 0.0174533f;
        float t = 1.0f - i / 120.0f;
        uint8_t bright = (uint8_t)(225 * t * t);
        draw_line_wide(RCX, RCY, RCX + (int)(RADIUS * cosf(a)), RCY - (int)(RADIUS * sinf(a)),
                       sweep565(bright));
    }
    {
        // 前沿：与渐变同色系，只略亮一档（不再用近白色，避免被误认为飞机的线）
        float a = sweep_deg * 0.0174533f;
        draw_line_wide(RCX, RCY, RCX + (int)(RADIUS * cosf(a)), RCY - (int)(RADIUS * sinf(a)),
                       sweep565(255));
    }
    sweep_deg -= 1.2f;
    if (sweep_deg < 0)
        sweep_deg += 360;

    // 真实航班：从 FlightManager 拿快照 → reconcile 到 UI 槽位 → 平滑插值渲染
    static TrackFlight snap[MAX_TRACKS];
    int ns = flight_mgr_snapshot(snap, MAX_TRACKS);
    g_air_count = ns;
    for (int i = 0; i < ns; i++)
    {
        int slot = -1, freeSlot = -1;
        for (int j = 0; j < MAX_TRACKS; j++)
        {
            if (uiF[j].active && !uiF[j].dying && strcmp(uiF[j].anum, snap[i].anum) == 0)
                slot = j;
            if (!uiF[j].active && freeSlot < 0)
                freeSlot = j;
        }
        if (slot < 0)
        {
            slot = freeSlot;
            if (slot < 0)
                continue; // 槽位满了
            uiF[slot].active = true;
            uiF[slot].dying = false;
            uiF[slot].opa = 0; // 新目标淡入，像真实雷达捕获目标
            snprintf(uiF[slot].anum, sizeof(uiF[slot].anum), "%s", snap[i].anum);
            geo_to_px(snap[i].lat, snap[i].lon, uiF[slot].curX, uiF[slot].curY); // 新飞机直接落在目标点
        }
        geo_to_px(snap[i].lat, snap[i].lon, uiF[slot].tgtX, uiF[slot].tgtY);
        uiF[slot].angle = snap[i].angle;
        snprintf(uiF[slot].fnum, sizeof(uiF[slot].fnum), "%s", snap[i].fnum);
        snprintf(uiF[slot].dep, sizeof(uiF[slot].dep), "%s", snap[i].dep);
        snprintf(uiF[slot].arr, sizeof(uiF[slot].arr), "%s", snap[i].arr);
        uiF[slot].alt_ft = (int)(snap[i].heightM * 3.28084f);
        uiF[slot].spd_kt = (int)(snap[i].speedKmh / 1.852f);
    }
    // 消失的飞机下架
    for (int j = 0; j < MAX_TRACKS; j++)
    {
        if (!uiF[j].active)
            continue;
        bool seen = false;
        for (int i = 0; i < ns; i++)
            if (strcmp(uiF[j].anum, snap[i].anum) == 0)
            {
                seen = true;
                break;
            }
        if (!seen && !uiF[j].dying)
        {
            uiF[j].dying = true; // 不立即隐藏，渲染循环里淡出后释放
            if (selectedAnum[0] && strcmp(selectedAnum, uiF[j].anum) == 0)
            {
                selectedAnum[0] = 0;
                lv_obj_add_flag(card, LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(sel_ring, LV_OBJ_FLAG_HIDDEN);
            }
        }
    }
    // 渲染：插值移动 + 按航向旋转
    for (int j = 0; j < MAX_TRACKS; j++)
    {
        if (!uiF[j].active)
            continue;
        // 淡入淡出：消失的目标原地淡出后释放槽位；新目标淡入（真实雷达的目标捕获/丢失感）
        if (uiF[j].dying)
        {
            uiF[j].opa -= 14;
            if (uiF[j].opa <= 0)
            {
                uiF[j].active = false;
                uiF[j].dying = false;
                tagvis[j] = false;
                lv_obj_add_flag(plane_imgs[j], LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(tag_labels[j], LV_OBJ_FLAG_HIDDEN);
                lv_obj_add_flag(vec_lines[j], LV_OBJ_FLAG_HIDDEN);
                continue;
            }
        }
        else if (uiF[j].opa < 255)
        {
            uiF[j].opa += 14;
            if (uiF[j].opa > 255)
                uiF[j].opa = 255;
        }
        lv_obj_set_style_img_opa(plane_imgs[j], (lv_opa_t)uiF[j].opa, 0);
        lv_obj_set_style_text_opa(tag_labels[j], (lv_opa_t)uiF[j].opa, 0);
        lv_obj_set_style_line_opa(vec_lines[j], (lv_opa_t)(uiF[j].opa * 153 / 255), 0);
        uiF[j].curX += (uiF[j].tgtX - uiF[j].curX) * 0.08f;
        uiF[j].curY += (uiF[j].tgtY - uiF[j].curY) * 0.08f;
        int bx = RCX + (int)uiF[j].curX;
        int by = RCY + (int)uiF[j].curY;
        // 飞出量程的飞机钳在外环内侧边缘，避免图标被裁掉只剩标签
        {
            float cdx = (float)(bx - RCX), cdy = (float)(by - RCY);
            float cr = sqrtf(cdx * cdx + cdy * cdy);
            if (cr > RADIUS - 12 && cr > 0)
            {
                float k = (RADIUS - 12) / cr;
                bx = RCX + (int)(cdx * k);
                by = RCY + (int)(cdy * k);
            }
        }
        lv_img_set_angle(plane_imgs[j], (int16_t)(uiF[j].angle * 10.0f));
        lv_obj_set_pos(plane_imgs[j], bx - PLANE_SZ / 2, by - PLANE_SZ / 2);
        lv_obj_clear_flag(plane_imgs[j], LV_OBJ_FLAG_HIDDEN);

        // 数据标签：白色航班号 + 琥珀色航线 + 蓝色高度/速度（ATC 数据块风格）
        // 位置先存起来，循环结束后统一做防重叠排开再落位
        {
            int fl = (uiF[j].alt_ft + 50) / 100; // 百英尺
            lv_label_set_text_fmt(tag_labels[j],
                                  "#ffffff %s#\n#ffc800 %s-%s#\n#5ac8ff FL%03d %dkt#",
                                  uiF[j].fnum, uiF[j].dep, uiF[j].arr, fl, uiF[j].spd_kt);
            int tx = (bx < RCX) ? bx + 24 : bx - 24 - 80;
            int ty = (by < RCY) ? by + 14 : by - 14 - 40;
            if (tx < 2) tx = 2;
            if (tx > 480 - 84) tx = 480 - 84;
            if (ty < 2) ty = 2;
            if (ty > 480 - 44) ty = 480 - 44;
            tagx[j] = tx;
            tagy[j] = ty;
            tagvis[j] = true;
            lv_obj_clear_flag(tag_labels[j], LV_OBJ_FLAG_HIDDEN);
        }
        // 速度矢量线：航向前方 2 分钟预达位置（真实 ATC 雷达要素），超出外环则截短
        {
            float arad = uiF[j].angle * 0.0174533f;
            float lenNm = uiF[j].spd_kt / 60.0f * 2.0f; // 2 分钟飞的距离
            float len = lenNm / RANGE_NM * RADIUS;
            float ex = bx + len * sinf(arad), ey = by - len * cosf(arad);
            float ddx = ex - RCX, ddy = ey - RCY;
            float dr = sqrtf(ddx * ddx + ddy * ddy);
            if (dr > RADIUS - 4 && dr > 0) // 裁进圆盘
            {
                float k = (RADIUS - 4) / dr;
                ex = RCX + ddx * k;
                ey = RCY + ddy * k;
            }
            vec_pts[j][0].x = bx;
            vec_pts[j][0].y = by;
            vec_pts[j][1].x = (lv_coord_t)ex;
            vec_pts[j][1].y = (lv_coord_t)ey;
            lv_line_set_points(vec_lines[j], vec_pts[j], 2);
            lv_obj_clear_flag(vec_lines[j], LV_OBJ_FLAG_HIDDEN);
        }

        // 选中染白
        bool sel = selectedAnum[0] && strcmp(uiF[j].anum, selectedAnum) == 0;
        lv_obj_set_style_img_recolor_opa(plane_imgs[j],
                                         sel ? LV_OPA_80 : LV_OPA_TRANSP, 0);
        if (sel)
            lv_obj_set_pos(sel_ring, bx - 26, by - 26);
    }

    // 标签防重叠：两两检测，纵向推开（ATC 标签 de-confliction 简化版）
    for (int a = 0; a < MAX_TRACKS; a++)
    {
        if (!tagvis[a])
            continue;
        for (int b = a + 1; b < MAX_TRACKS; b++)
        {
            if (!tagvis[b])
                continue;
            int dx = tagx[a] - tagx[b];
            int dy = tagy[a] - tagy[b];
            if (abs(dx) < 84 && abs(dy) < 42)
            {
                int push = 42 - abs(dy);
                int dir = (dy > 0) ? -1 : (dy < 0) ? 1 : (b % 2 ? 1 : -1);
                tagy[b] += dir * push; // b 往远离 a 的方向让位
                if (tagy[b] < 2)
                    tagy[b] = 2;
                if (tagy[b] > 480 - 44)
                    tagy[b] = 480 - 44;
            }
        }
    }
    for (int j = 0; j < MAX_TRACKS; j++)
        if (tagvis[j])
            lv_obj_set_pos(tag_labels[j], tagx[j], tagy[j]);

    lv_obj_invalidate(canvas);
}

static void canvas_click_cb(lv_event_t *e)
{
    lv_point_t p;
    lv_indev_get_point(lv_indev_get_act(), &p);
    Serial.printf("[ui] tap %d,%d\n", p.x, p.y);
    int best = -1;
    float bestd = 34.0f; // 飞机图标变大，点击半径同步放大
    for (int j = 0; j < MAX_TRACKS; j++)
    {
        if (!uiF[j].active)
            continue;
        float dx = p.x - (RCX + uiF[j].curX);
        float dy = p.y - (RCY + uiF[j].curY);
        float d = sqrtf(dx * dx + dy * dy);
        if (d < bestd)
        {
            bestd = d;
            best = j;
        }
    }
    if (best >= 0)
    {
        UiFlight &f = uiF[best];
        snprintf(selectedAnum, sizeof(selectedAnum), "%s", f.anum);
        lv_label_set_text_fmt(card_label,
                              "%s  %s-%s\nALT %d ft   SPD %d kt",
                              f.fnum, f.dep, f.arr, f.alt_ft, f.spd_kt);
        lv_obj_clear_flag(card, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(sel_ring, LV_OBJ_FLAG_HIDDEN);
    }
    else
    {
        selectedAnum[0] = 0;
        lv_obj_add_flag(card, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(sel_ring, LV_OBJ_FLAG_HIDDEN);
        // 点击圆心（30px 内）：触发一次测试播报，验证 AI 播报链路
        float cdx = p.x - RCX, cdy = p.y - RCY;
        if (sqrtf(cdx * cdx + cdy * cdy) < 30.0f)
            announce_test();
    }
}

static void build_ui()
{
    lv_obj_set_style_bg_color(lv_scr_act(), COL_BG, 0);
    // 屏幕本身默认可滚动：子对象贴边会撑出右侧滚动条，必须关掉
    lv_obj_clear_flag(lv_scr_act(), LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(lv_scr_act(), LV_SCROLLBAR_MODE_OFF);

    // 底层 canvas：只承载扫描渐变 + 辉光
    cbuf = (uint16_t *)heap_caps_malloc(RADAR_W * RADAR_H * 2, MALLOC_CAP_SPIRAM);
    if (!cbuf)
    {
        Serial.println("canvas buffer alloc FAILED");
        while (1)
            delay(1000);
    }
    // 初始填充磷光底色（不是纯黑）
    uint32_t n0 = RADAR_W * RADAR_H;
    for (uint32_t i = 0; i < n0; i++)
        cbuf[i] = FLOOR565;
    canvas = lv_canvas_create(lv_scr_act());
    lv_canvas_set_buffer(canvas, cbuf, RADAR_W, RADAR_H, LV_IMG_CF_TRUE_COLOR);
    lv_obj_align(canvas, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(canvas, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_clear_flag(canvas, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_event_cb(canvas, canvas_click_cb, LV_EVENT_CLICKED, NULL);

    // 距离环（三环与外圈同粗同亮，统一风格）
    make_ring(RADIUS / 3, COL_GRID, LV_OPA_100, 5);
    make_ring((RADIUS / 3) * 2, COL_GRID, LV_OPA_100, 5);
    make_ring(RADIUS, COL_GRID, LV_OPA_100, 5);
    // 外圈边框环已移除（与提亮加粗的外环视觉上重复）

    // 距离环数值标签（18 号粗体透明底，压在 45° 方向的环线上）
    const char *range_txt[3] = {"40", "80", "120 NM"};
    const int range_r[3] = {RADIUS / 3, (RADIUS / 3) * 2, RADIUS};
    for (int i = 0; i < 3; i++)
    {
        lv_obj_t *l = lv_label_create(lv_scr_act());
        lv_label_set_text(l, range_txt[i]);
        lv_obj_set_style_text_font(l, &font_bold_18, 0);
        lv_obj_set_style_text_color(l, COL_TEXT, 0);
        lv_obj_update_layout(l);
        lv_area_t la;
        lv_obj_get_coords(l, &la);
        int lw = la.x2 - la.x1 + 1, lh = la.y2 - la.y1 + 1;
        int off = (int)(range_r[i] * 0.7071f);
        lv_obj_set_pos(l, RCX + off - lw / 2, RCY - off - lh / 2);
    }

    // 十字线（与圆环同粗同亮）
    {
        static lv_point_t hpts[2] = {{RCX - RADIUS, RCY}, {RCX + RADIUS, RCY}};
        static lv_point_t vpts[2] = {{RCX, RCY - RADIUS}, {RCX, RCY + RADIUS}};
        lv_obj_t *h = make_line_seg(COL_GRID, LV_OPA_100, 5);
        lv_line_set_points(h, hpts, 2);
        lv_obj_t *v = make_line_seg(COL_GRID, LV_OPA_100, 5);
        lv_line_set_points(v, vpts, 2);
    }

    // 方位刻度（每 30°，主线更长）
    // 注意：lv_line_set_points 只存指针不拷贝，点数组必须 static，否则栈内存失效后画出任意斜线
    static lv_point_t tick_pts[12][2];
    for (int a = 0; a < 360; a += 30)
    {
        float rad = a * 0.0174533f;
        float cs = cosf(rad), sn = sinf(rad);
        int len = (a % 90 == 0) ? 13 : 6;
        lv_obj_t *t = make_line_seg(COL_TICK, LV_OPA_90, (a % 90 == 0) ? 3 : 2);
        tick_pts[a / 30][0] = {(lv_coord_t)(RCX + (RADIUS - len) * sn), (lv_coord_t)(RCY - (RADIUS - len) * cs)};
        tick_pts[a / 30][1] = {(lv_coord_t)(RCX + RADIUS * sn), (lv_coord_t)(RCY - RADIUS * cs)};
        lv_line_set_points(t, tick_pts[a / 30], 2);
    }

    // 罗盘字母：44px 粗体透明底，中心压在环线上，环线从字母中间穿过
    const char *dirs[4] = {"N", "E", "S", "W"};
    const int dxs[4] = {0, 1, 0, -1};
    const int dys[4] = {-1, 0, 1, 0};
    for (int i = 0; i < 4; i++)
    {
        lv_obj_t *l = lv_label_create(lv_scr_act());
        lv_label_set_text(l, dirs[i]);
        lv_obj_set_style_text_color(l, COL_TEXT, 0);
        lv_obj_set_style_text_font(l, &font_bold_44, 0);
        lv_obj_update_layout(l);
        lv_area_t la;
        lv_obj_get_coords(l, &la);
        int lw = la.x2 - la.x1 + 1, lh = la.y2 - la.y1 + 1;
        // 四个方向统一内收 5px：宽字母（W/E）视觉中心偏外，大写字母（N/S）上下留白不均，
        // 统一内收让环线从四个字母的正中穿过、视觉一致
        lv_obj_set_pos(l, RCX + dxs[i] * (RADIUS - 5) - lw / 2, RCY + dys[i] * (RADIUS - 5) - lh / 2);
    }

    // 中心点
    {
        lv_obj_t *dot = lv_obj_create(lv_scr_act());
        lv_obj_set_size(dot, 6, 6);
        lv_obj_align(dot, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_style_bg_color(dot, COL_TICK, 0);
        lv_obj_set_style_bg_opa(dot, LV_OPA_80, 0);
        lv_obj_set_style_border_width(dot, 0, 0);
        lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_pad_all(dot, 0, 0);
        lv_obj_clear_flag(dot, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
    }

    // 萧山机场跑道 overlay：07/25 跑道（磁航向约 070°/250°），中心小横条 + 机场标
    {
        float rwy = 70.0f * 0.0174533f; // 跑道方向
        int rl = 15;                    // 半长（夸张显示，真实 3.6km 在 120NM 屏上只有几像素）
        int rdx = (int)(rl * sinf(rwy)), rdy = (int)(rl * cosf(rwy));
        static lv_point_t rwpts[2]; // static：lv_line 只存指针不拷贝
        rwpts[0] = {(lv_coord_t)(RCX - rdx), (lv_coord_t)(RCY + rdy)};
        rwpts[1] = {(lv_coord_t)(RCX + rdx), (lv_coord_t)(RCY - rdy)};
        lv_obj_t *rw = make_line_seg(lv_color_make(56, 150, 170), LV_OPA_90, 3);
        lv_line_set_points(rw, rwpts, 2);
        lv_obj_t *rwl = lv_label_create(lv_scr_act());
        lv_label_set_text(rwl, "HGH 07/25");
        lv_obj_set_style_text_color(rwl, COL_TEXT_DIM, 0);
        lv_obj_set_style_text_font(rwl, &font_bold_18, 0);
        lv_obj_clear_flag(rwl, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
        // 放在跑道垂直方向一侧，避开中心点（按实际宽高居中）
        lv_obj_update_layout(rwl);
        lv_area_t ra;
        lv_obj_get_coords(rwl, &ra);
        lv_obj_set_pos(rwl, RCX + (int)(26 * cosf(rwy)) - (ra.x2 - ra.x1 + 1) / 2,
                       RCY + (int)(26 * sinf(rwy)) - (ra.y2 - ra.y1 + 1) / 2);
    }

    // 飞机 ✈️ 图标（初始隐藏，等真实航班数据上架）
    build_plane_icon();
    for (int i = 0; i < MAX_TRACKS; i++)
    {
        plane_imgs[i] = lv_img_create(lv_scr_act());
        lv_img_set_src(plane_imgs[i], &plane_dsc);
        lv_img_set_pivot(plane_imgs[i], PLANE_SZ / 2, PLANE_SZ / 2);
        lv_img_set_antialias(plane_imgs[i], true);
        lv_obj_set_style_img_recolor(plane_imgs[i], lv_color_white(), 0);
        lv_obj_set_style_img_recolor_opa(plane_imgs[i], LV_OPA_TRANSP, 0);
        lv_obj_clear_flag(plane_imgs[i], (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
        lv_obj_add_flag(plane_imgs[i], LV_OBJ_FLAG_HIDDEN);

        // 数据标签（真实 ATC 雷达的 target tag，三色数据块）
        tag_labels[i] = lv_label_create(lv_scr_act());
        lv_label_set_text(tag_labels[i], "");
        lv_label_set_recolor(tag_labels[i], true); // 行内 #rrggbb 着色
        lv_obj_set_style_text_color(tag_labels[i], COL_TEXT, 0);
        lv_obj_set_style_text_font(tag_labels[i], &font_bold_12, 0);
        lv_obj_set_style_text_line_space(tag_labels[i], 1, 0);
        lv_obj_clear_flag(tag_labels[i], (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
        lv_obj_add_flag(tag_labels[i], LV_OBJ_FLAG_HIDDEN);

        // 速度矢量线（品红，朝航向前方）
        vec_lines[i] = lv_line_create(lv_scr_act());
        lv_obj_set_style_line_color(vec_lines[i], COL_VEC, 0);
        lv_obj_set_style_line_opa(vec_lines[i], LV_OPA_60, 0);
        lv_obj_set_style_line_width(vec_lines[i], 1, 0);
        lv_obj_clear_flag(vec_lines[i], (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
        lv_obj_add_flag(vec_lines[i], LV_OBJ_FLAG_HIDDEN);
    }

    // 选中光环（52px，套住 40px 飞机图标）
    sel_ring = lv_obj_create(lv_scr_act());
    lv_obj_set_size(sel_ring, 52, 52);
    lv_obj_set_style_bg_opa(sel_ring, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_color(sel_ring, lv_color_white(), 0);
    lv_obj_set_style_border_opa(sel_ring, LV_OPA_70, 0);
    lv_obj_set_style_border_width(sel_ring, 1, 0);
    lv_obj_set_style_radius(sel_ring, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_pad_all(sel_ring, 0, 0);
    lv_obj_clear_flag(sel_ring, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
    lv_obj_add_flag(sel_ring, LV_OBJ_FLAG_HIDDEN);

    // ---------- 四角要素（参考 ESP32-Plane-Radar 布局） ----------
    // 小号暗色标题在上，大号粗体彩色数值在下（AIR 绿 / RANGE 琥珀 / TIME 青）
    {
        // 左上：WIFI 信号强度（文字 + 4 格柱状图）
        lv_obj_t *cap = lv_label_create(lv_scr_act());
        lv_label_set_text(cap, "WIFI");
        lv_obj_set_style_text_color(cap, COL_TEXT_DIM, 0);
        lv_obj_set_style_text_font(cap, &lv_font_montserrat_14, 0);
        lv_obj_set_pos(cap, 26, 18);
        lv_obj_clear_flag(cap, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
        for (int i = 0; i < 4; i++)
        {
            wifi_bars[i] = lv_obj_create(lv_scr_act());
            int h = 6 + i * 4;
            lv_obj_set_size(wifi_bars[i], 6, h);
            lv_obj_set_pos(wifi_bars[i], 26 + i * 9, 56 - h);
            lv_obj_set_style_bg_color(wifi_bars[i], lv_color_make(40, 60, 80), 0);
            lv_obj_set_style_bg_opa(wifi_bars[i], LV_OPA_COVER, 0);
            lv_obj_set_style_border_width(wifi_bars[i], 0, 0);
            lv_obj_set_style_radius(wifi_bars[i], 1, 0);
            lv_obj_set_style_pad_all(wifi_bars[i], 0, 0);
            lv_obj_clear_flag(wifi_bars[i], (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
        }

        // 右上：AIR 在跟踪飞机数（绿色粗体大数）
        lv_obj_t *cap2 = lv_label_create(lv_scr_act());
        lv_label_set_text(cap2, "AIR");
        lv_obj_set_style_text_color(cap2, COL_TEXT_DIM, 0);
        lv_obj_set_style_text_font(cap2, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_align(cap2, LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_set_size(cap2, 90, 18);
        lv_obj_set_pos(cap2, 480 - 26 - 90, 18);
        lv_obj_clear_flag(cap2, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
        air_val = lv_label_create(lv_scr_act());
        lv_label_set_text(air_val, "0");
        lv_obj_set_style_text_color(air_val, lv_color_make(90, 230, 130), 0);
        lv_obj_set_style_text_font(air_val, &font_bold_28, 0);
        lv_obj_set_style_text_align(air_val, LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_set_size(air_val, 90, 36);
        lv_obj_set_pos(air_val, 480 - 26 - 90, 38);
        lv_obj_clear_flag(air_val, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));

        // 左下角留给 ON AIR 指示（RANGE 量程与环上的 40/80/120 NM 标签重复，不再单列）

        // 右下：TIME 当前时间（青色粗体）
        lv_obj_t *cap4 = lv_label_create(lv_scr_act());
        lv_label_set_text(cap4, "TIME");
        lv_obj_set_style_text_color(cap4, COL_TEXT_DIM, 0);
        lv_obj_set_style_text_font(cap4, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_align(cap4, LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_set_size(cap4, 90, 18);
        lv_obj_set_pos(cap4, 480 - 26 - 90, 480 - 26 - 36 - 20);
        lv_obj_clear_flag(cap4, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
        time_val = lv_label_create(lv_scr_act());
        lv_label_set_text(time_val, "--:--");
        lv_obj_set_style_text_color(time_val, lv_color_make(90, 215, 235), 0);
        lv_obj_set_style_text_font(time_val, &font_bold_28, 0);
        lv_obj_set_style_text_align(time_val, LV_TEXT_ALIGN_RIGHT, 0);
        lv_obj_set_size(time_val, 90, 36);
        lv_obj_set_pos(time_val, 480 - 26 - 90, 480 - 26 - 36);
        lv_obj_clear_flag(time_val, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
    }

    // 顶部标题/副标题已移除（表盘保持纯粹，N 字母区域无遮挡）

    // 详情卡
    card = lv_obj_create(lv_scr_act());
    lv_obj_clear_flag(card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(card, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_size(card, 280, 88);
    lv_obj_align(card, LV_ALIGN_TOP_MID, 0, 86);
    lv_obj_set_style_bg_color(card, lv_color_make(8, 20, 42), 0);
    lv_obj_set_style_bg_opa(card, LV_OPA_90, 0);
    lv_obj_set_style_border_color(card, COL_ACCENT, 0);
    lv_obj_set_style_border_opa(card, LV_OPA_50, 0);
    lv_obj_set_style_border_width(card, 1, 0);
    lv_obj_set_style_radius(card, 10, 0);
    lv_obj_set_style_shadow_width(card, 18, 0);
    lv_obj_set_style_shadow_opa(card, LV_OPA_40, 0);
    card_label = lv_label_create(card);
    lv_obj_set_style_text_color(card_label, COL_TEXT, 0);
    lv_obj_set_style_text_font(card_label, &font_bold_18, 0);
    lv_obj_center(card_label);
    lv_obj_add_flag(card, LV_OBJ_FLAG_HIDDEN);

    // 左下角：无线电状态（RADIO 小标题 + 粗体状态字，与其他三角同风格同基线）
    lv_obj_t *cap5 = lv_label_create(lv_scr_act());
    lv_label_set_text(cap5, "RADIO");
    lv_obj_set_style_text_color(cap5, COL_TEXT_DIM, 0);
    lv_obj_set_style_text_font(cap5, &lv_font_montserrat_14, 0);
    lv_obj_set_pos(cap5, 26, 398);
    lv_obj_clear_flag(cap5, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));
    tx_txt = lv_label_create(lv_scr_act());
    lv_label_set_text(tx_txt, "STBY");
    lv_obj_set_style_text_color(tx_txt, lv_color_make(90, 210, 140), 0);
    lv_obj_set_style_text_font(tx_txt, &font_bold_24, 0);
    lv_obj_set_pos(tx_txt, 26, 421);
    lv_obj_clear_flag(tx_txt, (lv_obj_flag_t)(LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE));

    lv_timer_create(radar_frame, 33, NULL); // ~30fps
    lv_timer_create(status_tick, 1000, NULL);
    lv_timer_create(tx_tick, 80, NULL); // TX/STBY 与声音同步，不走 1s 慢刷
}

void setup(void)
{
    Serial.begin(115200);
    delay(2000);
    Serial.println("[1] boot ok");

    if (!gfx->begin())
        Serial.println("[2] gfx->begin() FAILED!");
    bus->writeC8D8(0x36, 0xA0);
    gfx->fillScreen(RGB565_BLACK);
    gfx->setBrightness(160);
    Serial.println("[2] gfx ok");

    touch.setPins(TP_RST, TP_INT);
    if (!touch.begin(Wire, CST92XX_SLAVE_ADDRESS, IIC_SDA, IIC_SCL))
    {
        Serial.println("[3] touch offline!");
        while (1)
            delay(1000);
    }
    touch.setMaxCoordinates(LCD_WIDTH, LCD_HEIGHT);
    touch.setSwapXY(true);
    touch.setMirrorXY(true, false);
    pinMode(TP_INT, INPUT_PULLUP);
    Serial.println("[3] touch ok");

    lv_init();
    lv_color_t *buf1 = (lv_color_t *)heap_caps_malloc(LCD_WIDTH * BUF_LINES * sizeof(lv_color_t), MALLOC_CAP_DMA);
    lv_color_t *buf2 = (lv_color_t *)heap_caps_malloc(LCD_WIDTH * BUF_LINES * sizeof(lv_color_t), MALLOC_CAP_DMA);
    if (!buf1 || !buf2)
    {
        Serial.println("[4] lvgl buffer alloc FAILED!");
        while (1)
            delay(1000);
    }
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, LCD_WIDTH * BUF_LINES);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = LCD_WIDTH;
    disp_drv.ver_res = LCD_HEIGHT;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    const esp_timer_create_args_t tick_args = {.callback = &lvgl_tick_cb, .name = "lvgl_tick"};
    esp_timer_handle_t tick_timer = NULL;
    esp_timer_create(&tick_args, &tick_timer);
    esp_timer_start_periodic(tick_timer, 2000);

    Serial.printf("[4] lvgl %d.%d.%d ready, heap=%u psram=%u\n",
                  lv_version_major(), lv_version_minor(), lv_version_patch(),
                  heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                  heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    build_ui();
    Serial.println("[5] radar ui built");

    // WiFi（无凭据则开 HGH-RADAR 热点配网，配好后自动重启）→ 雷达中心 → 航班任务
    net_setup();
    net_center(CENTER_LAT, CENTER_LON, RANGE_NM);
    Serial.printf("[6] center %.4f,%.4f range %.0fNM\n", CENTER_LAT, CENTER_LON, RANGE_NM);

    // ES8311 音频（触摸已把 Wire 拉起来，codec 在同一 I2C 总线上）
    if (audio_init())
    {
        audio_play_tone(880, 150); // 开机提示音：叮
        delay(180);
        audio_play_tone(1320, 200);
        Serial.println("[7] audio ok");
    }
    else
        Serial.println("[7] audio init FAILED");

    flight_mgr_start();
    Serial.println("[8] flight mgr up");

    announce_start(); // AI 播报任务（进近 30km 内触发）
    Serial.println("[9] announce up");
}

void loop()
{
    lv_timer_handler();
    static uint32_t last = 0;
    if (millis() - last > 2000)
    {
        last = millis();
        Serial.printf("alive %lus heap=%u psram=%u\n", millis() / 1000,
                      heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                      heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    }
    delay(5);
}
