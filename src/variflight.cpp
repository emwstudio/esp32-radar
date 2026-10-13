#include "variflight.h"
#include "secrets.h"
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <esp_heap_caps.h>

static const char VF_URL[] = "https://ai.variflight.com/servers/aviation/mcp";
static const char VF_KEY[] = VF_API_KEY;

// 响应体可能几十 KB（全天航线 full detail），放 PSRAM；raw 与解转义后文本分开两块
static char *g_buf = nullptr;  // 原始 HTTP 响应
static char *g_text = nullptr; // 解转义后的 content text
static const size_t BUF_SZ = 192 * 1024;
static const size_t TEXT_SZ = 128 * 1024;
static char g_err[120] = "";   // 最近一次 API 错误（isError=true 时的文本，纯 ASCII）

const char *vf_last_error(void) { return g_err; }

// HTTPClient::writeToStream 的接收端，写进 PSRAM buffer
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

// JSON-RPC POST，解转义后的 content[0].text 写入全局 g_text，失败返回 false
static bool rpc_call(const char *tool, const char *argsJson)
{
    if (!g_buf)
    {
        g_buf = (char *)heap_caps_malloc(BUF_SZ, MALLOC_CAP_SPIRAM);
        g_text = (char *)heap_caps_malloc(TEXT_SZ, MALLOC_CAP_SPIRAM);
        if (!g_buf || !g_text)
        {
            Serial.println("[vf] psram buf alloc failed");
            return false;
        }
    }
    char *out = g_text;
    const size_t outSz = TEXT_SZ;
    char body[512];
    snprintf(body, sizeof(body),
             "{\"jsonrpc\":\"2.0\",\"id\":1,\"method\":\"tools/call\","
             "\"params\":{\"name\":\"%s\",\"arguments\":%s}}",
             tool, argsJson);

    WiFiClientSecure cli;
    cli.setInsecure();
    HTTPClient http;
    http.setTimeout(25000);
    if (!http.begin(cli, VF_URL))
        return false;
    http.addHeader("Content-Type", "application/json");
    http.addHeader("Accept", "application/json, text/event-stream");
    http.addHeader("X-API-Key", VF_KEY);
    int code = http.POST((uint8_t *)body, strlen(body));
    if (code != 200)
    {
        Serial.printf("[vf] %s http %d\n", tool, code);
        http.end();
        return false;
    }
    PsramSink sink;
    http.writeToStream(&sink);
    g_buf[sink.n] = 0;
    http.end();
    Serial.printf("[vf] %s resp %u bytes\n", tool, sink.n);

    // 找 "text" 字段值并做 JSON 字符串解转义
    char *p = strstr(g_buf, "\"text\"");
    if (!p)
        return false;
    p = strchr(p + 6, ':');
    if (!p)
        return false;
    p++;
    while (*p == ' ' || *p == '\t')
        p++;
    if (*p != '"')
        return false;
    p++;
    size_t m = 0;
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
            case 'u': // \uXXXX → UTF-8（仅 BMP）
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

    // API 业务错误（如余额不足）：response 带 "isError":true，text 是错误描述
    g_err[0] = 0;
    if (strstr(g_buf, "\"isError\":true"))
    {
        if (strstr(out, "Insufficient balance"))
            snprintf(g_err, sizeof(g_err), "VARIFLIGHT INSUFFICIENT BALANCE - TOP UP FIRST");
        else
        {
            size_t k = 0;
            for (const char *q = out; *q && k < sizeof(g_err) - 1; q++)
            {
                unsigned char c = (unsigned char)*q;
                if (c < 0x80) // 只留 ASCII，中文等去掉，避免 LVGL 字体缺字形
                    g_err[k++] = (char)c;
            }
            g_err[k] = 0;
        }
        Serial.printf("[vf] API error: %s\n", g_err);
    }
    return true;
}

// ---------- Python repr 风格字段提取（值是 'str' 或裸数字） ----------
// 在 [from, end) 内找 'key': ，返回值的起点（引号后或数字首字符），找不到返回 nullptr
static const char *py_find(const char *from, const char *end, const char *key)
{
    char pat[40];
    snprintf(pat, sizeof(pat), "'%s':", key);
    size_t pl = strlen(pat);
    const char *p = from;
    while ((p = (const char *)memchr(p, '\'', end - p)) && p < end)
    {
        if (end - p >= (ptrdiff_t)pl && memcmp(p, pat, pl) == 0)
        {
            p += pl;
            while (p < end && (*p == ' '))
                p++;
            return p;
        }
        p++;
    }
    return nullptr;
}

static bool py_str(const char *from, const char *end, const char *key, char *out, size_t sz)
{
    const char *p = py_find(from, end, key);
    if (!p || *p != '\'')
        return false;
    p++;
    size_t m = 0;
    while (p < end && *p != '\'' && m < sz - 1)
        out[m++] = *p++;
    out[m] = 0;
    return true;
}

static float py_num(const char *from, const char *end, const char *key, float def = 0)
{
    const char *p = py_find(from, end, key);
    if (!p)
        return def;
    return strtof(p, nullptr);
}

// ---------- 对外接口 ----------
bool vf_search_route(const char *date, const char *dep, const char *arr,
                     VfRouteFlight *out, int maxOut, int *count)
{
    *count = 0;
    char args[160];
    snprintf(args, sizeof(args),
             "{\"date\":\"%s\",\"dep\":\"%s\",\"arr\":\"%s\",\"detail\":\"full\"}",
             date, dep, arr);
    if (!rpc_call("searchFlightsByDepArr", args))
        return false;
    const char *text = g_text;
    const char *textEnd = text + strlen(text);

    const char *p = text;
    while (*count < maxOut)
    {
        const char *rec = py_find(p, textEnd, "FlightNo");
        if (!rec)
            break;
        // 记录边界：到下一条 FlightNo 或文本尾
        const char *next = py_find(rec, textEnd, "FlightNo");
        const char *recEnd = next ? next : textEnd;
        VfRouteFlight &f = out[*count];
        // FlightNo 的值直接从 rec 处读
        if (*rec == '\'')
        {
            rec++;
            size_t m = 0;
            while (rec < recEnd && *rec != '\'' && m < sizeof(f.fnum) - 1)
                f.fnum[m++] = *rec++;
            f.fnum[m] = 0;
        }
        else
            f.fnum[0] = 0;
        if (!py_str(rec, recEnd, "AircraftNumber", f.anum, sizeof(f.anum)))
            f.anum[0] = 0;
        if (!py_str(rec, recEnd, "FlightState", f.state, sizeof(f.state)))
            f.state[0] = 0;
        if (!py_str(rec, recEnd, "FlightDepcode", f.dep, sizeof(f.dep)))
            snprintf(f.dep, sizeof(f.dep), "%s", dep);
        if (!py_str(rec, recEnd, "FlightArrcode", f.arr, sizeof(f.arr)))
            snprintf(f.arr, sizeof(f.arr), "%s", arr);
        (*count)++;
        p = recEnd;
    }
    return true;
}

bool vf_get_position(const char *anum, VfPosition &out)
{
    out = VfPosition();
    char args[64];
    snprintf(args, sizeof(args), "{\"anum\":\"%s\"}", anum);
    if (!rpc_call("getRealtimeLocationByAnum", args))
        return false;
    const char *text = g_text;
    const char *end = text + strlen(text);
    if (!strstr(text, "'position'"))
        return false;
    py_str(text, end, "fnum", out.fnum, sizeof(out.fnum));
    py_str(text, end, "dep", out.dep, sizeof(out.dep));
    py_str(text, end, "arr", out.arr, sizeof(out.arr));
    out.angle = py_num(text, end, "angle");
    out.speedKmh = py_num(text, end, "speed");
    out.heightM = py_num(text, end, "height");
    out.updatetime = (uint32_t)py_num(text, end, "updatetime");
    // position 子字典里的 lng/lat
    const char *pos = py_find(text, end, "position");
    if (pos)
    {
        out.lon = strtod(py_find(pos, end, "lng") ?: "", nullptr);
        out.lat = strtod(py_find(pos, end, "lat") ?: "", nullptr);
    }
    out.ok = (out.lon != 0 || out.lat != 0);
    return true;
}
