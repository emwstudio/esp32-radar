// AI 播报：MiniCPM5-2B 生成文案 → VoxCPM2 TTS → ES8311 播放
#include "announce.h"
#include "audio_player.h"
#include "secrets.h"
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <esp_heap_caps.h>
#include <mbedtls/base64.h>

// 1 = 用参考音频克隆塔台管制员音色（需自备 atc_ref.h，见 README）；0 = 默认音色
#define ANN_USE_REF_VOICE 1
#if ANN_USE_REF_VOICE
#include "atc_ref.h" // 真实塔台录音属个人素材，已在 .gitignore 中排除
#endif

// 播报总开关：0 = 整体静默，不发起任何付费 API 调用（雷达/UI 照常）
#define ANN_ENABLED 1

static const char MB_BASE[] = "https://api.modelbest.cn/v1";
static const char MB_KEY[] = MB_API_KEY;
static const char MB_TEXT_MODEL[] = "MiniCPM5-2B";
static const char MB_TTS_MODEL[] = "VoxCPM2";

#define ANN_RADIUS_KM 30.0f          // 进入这个距离才播报
#define ANN_COOLDOWN_MS (15 * 60000) // 同一架飞机 15 分钟内只播一次

// ---------- PSRAM 接收缓冲 ----------
static char *g_buf = nullptr;    // HTTP 响应（SSE 全文）
static uint8_t *g_wav = nullptr; // 解码后的 WAV
static char *g_req = nullptr;    // TTS 请求体（含 base64 参考音频）
static const size_t BUF_SZ = 1536 * 1024;
static const size_t WAV_SZ = 1400 * 1024;
static const size_t REQ_SZ = 400 * 1024;

// 参考音频对应的台词（atc_ref.h 8.6~17.6s 段），提高克隆相似度
static const char ATC_REF_TEXT[] =
    "幺六洞八，地面，幺六洞八已经完全准备好了吗？动作稍快一点，"
    "前面武汉已经开车了，你跟他四分钟，隔太远了，下一波北京过来，又走不了了";

class PsramSink : public Stream
{
public:
    size_t n = 0;
    size_t write(uint8_t b) override
    {
        if (n < BUF_SZ - 1)
            g_buf[n++] = (char)b;
        return 1;
    }
    size_t write(const uint8_t *p, size_t len) override
    {
        size_t room = BUF_SZ - 1 - n;
        size_t c = len < room ? len : room;
        memcpy(g_buf + n, p, c);
        n += c;
        return c;
    }
    int available() override { return 0; }
    int read() override { return -1; }
    int peek() override { return -1; }
};

static bool alloc_bufs()
{
    if (g_buf)
        return true;
    g_buf = (char *)heap_caps_malloc(BUF_SZ, MALLOC_CAP_SPIRAM);
    g_wav = (uint8_t *)heap_caps_malloc(WAV_SZ, MALLOC_CAP_SPIRAM);
    g_req = (char *)heap_caps_malloc(REQ_SZ, MALLOC_CAP_SPIRAM);
    if (!g_buf || !g_wav || !g_req)
        Serial.println("[ann] psram alloc failed");
    return g_buf && g_wav && g_req;
}

// HTTPS POST 整段接收到 g_buf，返回长度（0=失败）
static size_t http_post(const char *path, const char *body)
{
    char url[96];
    snprintf(url, sizeof(url), "%s%s", MB_BASE, path);
    WiFiClientSecure cli;
    cli.setInsecure();
    HTTPClient http;
    http.setTimeout(60000); // TTS 生成较慢
    if (!http.begin(cli, url))
        return 0;
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Authorization", String("Bearer ") + MB_KEY);
    int code = http.POST((uint8_t *)body, strlen(body));
    if (code != 200)
    {
        Serial.printf("[ann] %s http %d\n", path, code);
        http.end();
        return 0;
    }
    PsramSink sink;
    http.writeToStream(&sink);
    g_buf[sink.n] = 0;
    http.end();
    Serial.printf("[ann] %s resp %u bytes\n", path, (unsigned)sink.n);
    return sink.n;
}

// JSON 字符串解转义（含 \uXXXX → UTF-8），从 src('" 后) 到下一个未转义引号
static size_t json_unescape(const char *src, char *out, size_t outSz)
{
    size_t m = 0;
    const char *p = src;
    while (*p && *p != '"' && m < outSz - 1)
    {
        if (*p == '\\' && p[1])
        {
            p++;
            switch (*p)
            {
            case 'n': out[m++] = '\n'; break;
            case 't': out[m++] = '\t'; break;
            case 'r': out[m++] = '\r'; break;
            case 'u':
            {
                unsigned cp = 0;
                for (int i = 0; i < 4 && p[1]; i++)
                {
                    char c = *++p;
                    cp <<= 4;
                    if (c >= '0' && c <= '9') cp |= c - '0';
                    else if (c >= 'a' && c <= 'f') cp |= c - 'a' + 10;
                    else if (c >= 'A' && c <= 'F') cp |= c - 'A' + 10;
                }
                if (cp < 0x80) out[m++] = (char)cp;
                else if (cp < 0x800)
                {
                    out[m++] = (char)(0xC0 | (cp >> 6));
                    out[m++] = (char)(0x80 | (cp & 0x3F));
                }
                else
                {
                    out[m++] = (char)(0xE0 | (cp >> 12));
                    out[m++] = (char)(0x80 | ((cp >> 6) & 0x3F));
                    out[m++] = (char)(0x80 | (cp & 0x3F));
                }
                break;
            }
            default: out[m++] = *p; break;
            }
            p++;
        }
        else
            out[m++] = *p++;
    }
    out[m] = 0;
    return m;
}

// ---------- ① 文案生成 ----------
static const char *city_cn(const char *code)
{
    struct { const char *code, *cn; } M[] = {
        {"PEK", "北京"}, {"PKX", "北京"}, {"CAN", "广州"}, {"SZX", "深圳"},
        {"CKG", "重庆"}, {"CTU", "成都"}, {"TFU", "成都"}, {"XIY", "西安"},
        {"KMG", "昆明"}, {"XMN", "厦门"}, {"HGH", "杭州"}, {"SHA", "上海"},
        {"PVG", "上海"}, {"NKG", "南京"}, {"WUH", "武汉"}, {"CSX", "长沙"},
    };
    for (auto &m : M)
        if (!strcmp(m.code, code))
            return m.cn;
    return code;
}

// 播报风格池：每次播报随机抽一种，few-shot 示例随风格切换，避免文案千篇一律。
// 风格参考真实塔台通话（含萧山塔台录音的口吻：简短、直接、可带催促）。
struct AnnStyle
{
    const char *sys;       // 风格指令（接在通用约束后面）
    const char *shot_user; // few-shot 输入
    const char *shot_asst; // few-shot 输出
};
static const AnnStyle ANN_STYLES[] = {
    {"风格：进近通报，报航班号、高度，简短干脆。",
     "航班CA1724，从北京飞杭州，高度3452米，速度604公里每小时，距离60公里",
     "注意，CA1724正在进近，高度3452米"},
    {"风格：落地提醒，报航班号和剩余距离，语气平稳。",
     "航班MU5381，从上海飞杭州，高度610米，速度330公里每小时，距离12公里",
     "MU5381，距离跑道12公里，准备落地"},
    {"风格：航线通报，报航班号、出发地和目的地，像雷达席例行通报。",
     "航班CZ3847，从广州飞杭州，高度8900米，速度870公里每小时，距离180公里",
     "雷达看到，CZ3847，广州过来的，巡航8900米"},
    {"风格：塔台催促口吻，动作稍快、时间有限那一挂，简短直接。",
     "航班ZH9566，从深圳飞杭州，高度480米，速度280公里每小时，距离8公里",
     "ZH9566，距跑道8公里，动作稍快点，后面还有排队"},
    {"风格：过台通报，飞机正从机场附近上空经过，报航班号和速度。",
     "航班HU7789，从杭州飞西安，高度1500米，速度480公里每小时，距离15公里",
     "HU7789正从本场过，速度480，航向西安"},
    {"风格：欢迎抵达，带点人情味，但仍简短。",
     "航班MF8055，从厦门飞杭州，高度300米，速度260公里每小时，距离5公里",
     "MF8055，欢迎来杭州，五边5公里，落地请联系地面"},
    {"风格：高度速度通报，雷达席确认目标，报航班号、高度和速度。",
     "航班3U8921，从成都飞杭州，高度7200米，速度800公里每小时，距离95公里",
     "3U8921，雷达识别，高度7200，速度800"},
    {"风格：离场关注，飞机刚从本场起飞爬升，报航班号和目的地。",
     "航班GJ8823，从杭州飞昆明，高度900米，速度420公里每小时，距离10公里",
     "GJ8823刚离地，爬升中，去昆明，联系进近再见"},
};

static bool gen_script(const TrackFlight &f, float distKm, char *out, size_t outSz)
{
    char body[2048];
    // 按场景选风格池：本场进近 / 本场离场 / 过路巡航，各自有合适的口吻
    static const uint8_t POOL_ARR[] = {0, 1, 3, 5};  // 进近通报/落地提醒/催促/欢迎
    static const uint8_t POOL_DEP[] = {7, 2, 6};     // 离场关注/航线通报/高度速度
    static const uint8_t POOL_OVR[] = {2, 4, 6};     // 航线通报/过台/高度速度
    const uint8_t *pool;
    int poolN;
    if (!strcmp(f.arr, "HGH"))
    {
        pool = POOL_ARR;
        poolN = sizeof(POOL_ARR);
    }
    else if (!strcmp(f.dep, "HGH"))
    {
        pool = POOL_DEP;
        poolN = sizeof(POOL_DEP);
    }
    else
    {
        pool = POOL_OVR;
        poolN = sizeof(POOL_OVR);
    }
    // 随机抽风格，避免和上一次重复
    static int lastStyle = -1;
    int si;
    do
    {
        si = pool[random(0, poolN)];
    } while (si == lastStyle);
    lastStyle = si;
    const AnnStyle &st = ANN_STYLES[si];
    // 数字保留阿拉伯形式由 TTS 读，航班号让模型用航空读法（幺两拐洞）
    // max_tokens 要给够：该模型带推理链，推理部分也吃输出预算
    // temperature 拉高：同风格下句式也有变化
    snprintf(body, sizeof(body),
             "{\"model\":\"%s\",\"stream\":false,\"max_tokens\":600,\"temperature\":1.0,"
             "\"messages\":["
             "{\"role\":\"system\",\"content\":\"你是机场塔台管制员，在雷达席做实时播报。"
             "把航班信息浓缩成一句25字以内的中文口语播报，直接输出播报句本身，"
             "不要任何标题、前缀、引号或解释。"
             "航班号必须逐字符原样照抄（大写字母+阿拉伯数字，如CA1724），绝对不要转换成中文读法。"
             "高度、速度、距离保持阿拉伯数字不要转成中文。%s\"},"
             "{\"role\":\"user\",\"content\":\"%s\"},"
             "{\"role\":\"assistant\",\"content\":\"%s\"},"
             "{\"role\":\"user\",\"content\":\"航班%s，从%s飞%s，高度%.0f米，速度%.0f公里每小时，距离%.0f公里\"}"
             "]}",
             MB_TEXT_MODEL, st.sys, st.shot_user, st.shot_asst,
             f.fnum, city_cn(f.dep), city_cn(f.arr),
             f.heightM, f.speedKmh, distKm);
    if (!http_post("/chat/completions", body))
        return false;
    // 从 "role":"assistant" 之后找 "content":"（避开 reasoning_content）
    char *p = strstr(g_buf, "\"role\":\"assistant\"");
    if (!p)
        p = strstr(g_buf, "\"role\": \"assistant\"");
    if (!p)
        return false;
    p = strstr(p, "\"content\":\"");
    if (!p)
        p = strstr(p, "\"content\": \"");
    if (!p)
        return false;
    p = strchr(p, ':') + 1;
    while (*p == ' ')
        p++;
    p++; // 跳过引号
    size_t n = json_unescape(p, out, outSz);
    // 去掉模型可能带的首尾空白/换行
    while (n && (out[n - 1] == '\n' || out[n - 1] == ' '))
        out[--n] = 0;
    Serial.printf("[ann] script: %s\n", out);
    return n > 0;
}

// ---------- ② TTS：SSE 全文 → 摘 audio delta → base64 解码拼成 WAV ----------
// 航班号 → 航空读法（字母不变，数字转幺两拐洞），设备端确定性转换，不信 LLM 自觉
static void spoken_fnum(const char *fnum, char *out, size_t outSz)
{
    static const char *DIG[] = {"洞", "幺", "两", "三", "四", "五", "六", "拐", "八", "九"};
    size_t m = 0;
    for (const char *p = fnum; *p && m < outSz - 4; p++)
    {
        if (*p >= '0' && *p <= '9')
        {
            const char *d = DIG[*p - '0'];
            while (*d && m < outSz - 2)
                out[m++] = *d++;
        }
        else
            out[m++] = *p;
    }
    out[m] = 0;
}

// 文案后处理：原样航班号替换成航空读法；残余「零」（E9 9B B6）按读法改成「洞」（E6 B4 9E）
static void fix_script_fnum(const TrackFlight &f, char *script, size_t scriptSz)
{
    char spoken[32];
    spoken_fnum(f.fnum, spoken, sizeof(spoken));
    char *hit = strstr(script, f.fnum);
    if (hit)
    {
        size_t fnLen = strlen(f.fnum);
        size_t headLen = hit - script;
        size_t tailLen = strlen(hit + fnLen);
        if (headLen + strlen(spoken) + tailLen < scriptSz)
        {
            memmove(hit + strlen(spoken), hit + fnLen, tailLen + 1);
            memcpy(hit, spoken, strlen(spoken));
        }
    }
    for (char *p = script; p[0] && p[1] && p[2]; p++)
        if ((uint8_t)p[0] == 0xE9 && (uint8_t)p[1] == 0x9B && (uint8_t)p[2] == 0xB6)
        {
            p[0] = (char)0xE6;
            p[1] = (char)0xB4;
            p[2] = (char)0x9E;
            p += 2;
        }
}
static size_t tts_wav(const char *text)
{
    // input 里的引号/反斜杠转义
    char esc[512];
    size_t e = 0;
    for (const char *p = text; *p && e < sizeof(esc) - 2; p++)
    {
        if (*p == '"' || *p == '\\')
            esc[e++] = '\\';
        esc[e++] = *p;
    }
    esc[e] = 0;
    char *body = g_req;
#if ANN_USE_REF_VOICE
    // 带参考音色克隆：body = {"model":"...","input":"...","response_format":"wav","ref_audio":"data:audio/wav;base64,<...>"}
    int h = snprintf(body, REQ_SZ,
                     "{\"model\":\"%s\",\"input\":\"%s\",\"response_format\":\"wav\",\"ref_audio\":\"data:audio/wav;base64,",
                     MB_TTS_MODEL, esc);
    size_t b64len = 0;
    if (mbedtls_base64_encode((unsigned char *)body + h, REQ_SZ - h - 4, &b64len,
                              ATC_REF_WAV, ATC_REF_WAV_LEN) != 0)
    {
        Serial.println("[ann] ref b64 encode failed");
        return 0;
    }
    strcpy(body + h + b64len, "\",\"ref_text\":\"");
    strcat(body, ATC_REF_TEXT);
    strcat(body, "\"}");
#else
    snprintf(body, REQ_SZ,
             "{\"model\":\"%s\",\"input\":\"%s\",\"response_format\":\"wav\"}",
             MB_TTS_MODEL, esc);
#endif

    // SSE 流偶尔会被网络截断（收到几十 KB 就断），以 speech.audio.done 标记判断完整性，
    // 不完整重试，最多 3 次（克隆音色请求体大，截断率更高）
    for (int attempt = 0; attempt < 3; attempt++)
    {
        size_t n = http_post("/audio/speech", body);
        if (!n)
            continue;
        size_t wavLen = 0;
        const char *p = g_buf;
        const char *end = g_buf + n;
        while (p < end)
        {
            const char *a = strstr(p, "\"audio\":\"");
            if (!a)
                break;
            a += 9;
            const char *b = strchr(a, '"');
            if (!b)
                break;
            size_t b64len = b - a;
            size_t need = (b64len / 4 + 1) * 3;
            if (wavLen + need >= WAV_SZ)
            {
                Serial.println("[ann] wav buffer full, truncated");
                break;
            }
            size_t olen = 0;
            if (mbedtls_base64_decode(g_wav + wavLen, WAV_SZ - wavLen, &olen,
                                      (const unsigned char *)a, b64len) == 0)
                wavLen += olen;
            p = b + 1;
        }
        bool complete = strstr(g_buf, "speech.audio.done") != nullptr;
        Serial.printf("[ann] wav decoded %u bytes (%s)\n", (unsigned)wavLen,
                      complete ? "complete" : "TRUNCATED");
        if (complete && wavLen > 1000)
            return wavLen;
        Serial.println("[ann] tts truncated, retry");
        delay(800);
    }
    return 0;
}

// ---------- 任务与触发 ----------
// 本地兜底文案：LLM 推理链偶尔吃光 token 预算返回空文案时用模板顶上，
// 保证播报链路永不空跑。航班号数字转航空读法。
static void fallback_script(const TrackFlight &f, char *out, size_t outSz)
{
    char fn[24];
    spoken_fnum(f.fnum, fn, sizeof(fn));
    snprintf(out, outSz, "注意，%s航班，%s飞%s，高度%d米",
             fn, city_cn(f.dep), city_cn(f.arr), (int)f.heightM);
    Serial.printf("[ann] fallback script: %s\n", out);
}

static char g_status[48] = "";
static SemaphoreHandle_t g_sem = nullptr;
static TrackFlight g_pending;
static float g_pendingDist = 0;
static volatile bool g_hasPending = false;

// 已播报记录（机尾号 + 时间）
struct AnnRec
{
    char anum[8];
    uint32_t at;
};
static AnnRec g_ann[MAX_TRACKS];

void announce_status(char *buf, size_t n) { snprintf(buf, n, "%s", g_status); }

// 调试：跳过冷却/距离判断，直接播报一架虚拟测试航班（验证 LLM+TTS+播放链路）
void announce_test()
{
    Serial.println("[ann] test triggered");
    if (!ANN_ENABLED)
    {
        Serial.println("[ann] disabled (debug), skip");
        return;
    }
    TrackFlight f;
    f.used = true;
    snprintf(f.anum, sizeof(f.anum), "B6513");
    snprintf(f.fnum, sizeof(f.fnum), "CA1724");
    snprintf(f.dep, sizeof(f.dep), "PEK");
    snprintf(f.arr, sizeof(f.arr), "HGH");
    f.heightM = 3452;
    f.speedKmh = 604;
    if (g_hasPending)
        return;
    g_pending = f;
    g_pendingDist = 25;
    g_hasPending = true;
    if (g_sem)
        xSemaphoreGive(g_sem);
}

void announce_notify(const TrackFlight &f, float distKm)
{
    if (!ANN_ENABLED)
        return; // 调试静默：不产生任何费用
    if (!f.anum[0] || distKm > ANN_RADIUS_KM)
        return;
    if (g_hasPending)
        return; // 播报进行中：不标记冷却，下一秒轮询时再试，保证每架进圈的都能播上
    uint32_t now = millis();
    for (auto &r : g_ann)
        if (r.anum[0] && !strcmp(r.anum, f.anum))
        {
            if (now - r.at < ANN_COOLDOWN_MS)
                return; // 冷却中
            r.at = now;
            goto enqueue;
        }
    // 新机尾号：找个空槽记录
    for (auto &r : g_ann)
        if (!r.anum[0])
        {
            snprintf(r.anum, sizeof(r.anum), "%s", f.anum);
            r.at = now;
            goto enqueue;
        }
    g_ann[0].at = now; // 满了就挤掉最老的槽位（近似）
    snprintf(g_ann[0].anum, sizeof(g_ann[0].anum), "%s", f.anum);
enqueue:
    g_pending = f;
    g_pendingDist = distKm;
    g_hasPending = true;
    if (g_sem)
        xSemaphoreGive(g_sem);
}

static void announce_task(void *)
{
    while (true)
    {
        xSemaphoreTake(g_sem, portMAX_DELAY);
        if (!g_hasPending)
            continue;
        TrackFlight f = g_pending;
        float dist = g_pendingDist;
        g_hasPending = false;

        audio_play_tone(1175, 70); // 触发确认音：立刻有反馈，后台慢慢生成
        snprintf(g_status, sizeof(g_status), "GEN %s", f.fnum);
        char script[192];
        if (!gen_script(f, dist, script, sizeof(script)))
        {
            delay(600);
            if (!gen_script(f, dist, script, sizeof(script))) // 推理链偶尔吃光预算导致空文案，重试一次
                fallback_script(f, script, sizeof(script));   // 仍失败用本地模板兜底，链路不空跑
        }
        fix_script_fnum(f, script, sizeof(script)); // 航班号强制航空读法
        Serial.printf("[ann] final script: %s\n", script);
        snprintf(g_status, sizeof(g_status), "TTS %s", f.fnum);
        size_t wavLen = tts_wav(script);
        if (!wavLen)
        {
            snprintf(g_status, sizeof(g_status), "");
            continue;
        }
        snprintf(g_status, sizeof(g_status), ">> %s", f.fnum);
        audio_play_wav(g_wav, wavLen);
        g_status[0] = 0;
    }
}

void announce_start()
{
    if (!alloc_bufs())
        return;
    g_sem = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(announce_task, "ann", 16384, nullptr, 1, nullptr, 0);
    Serial.println("[ann] task started");
}
