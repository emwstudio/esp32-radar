#include "audio_player.h"
#include <Wire.h>
#include <ESP_I2S.h>
#include <esp_heap_caps.h>
#include "es8311.h"

// ESP32-S3-Touch-AMOLED-2.16 音频引脚（微雪官方 demo）
#define I2S_MCK_IO 42
#define I2S_BCK_IO 9
#define I2S_DI_IO 10
#define I2S_WS_IO 45
#define I2S_DO_IO 8
#define PA_PIN 46

static I2SClass g_i2s;
static es8311_handle_t g_es = nullptr;
static bool g_i2s_on = false;
static int g_cur_rate = 0;
static bool g_cur_stereo = true;
static volatile bool g_busy = false;

bool audio_busy(void) { return g_busy; }

// 播完收尾：补 ~120ms 静音冲掉 DMA 尾，停 I2S 时钟，关功放（消除空闲杂音）
static void play_end(void)
{
    if (g_i2s_on)
    {
        static uint8_t zeros[2048] = {0};
        size_t tail = (size_t)g_cur_rate * (g_cur_stereo ? 4 : 2) / 8; // ~125ms
        while (tail)
        {
            size_t c = tail > sizeof(zeros) ? sizeof(zeros) : tail;
            if (!g_i2s.write(zeros, c))
                break;
            tail -= c;
        }
        g_i2s.end();
        g_i2s_on = false;
        g_cur_rate = 0;
    }
    digitalWrite(PA_PIN, LOW);
}

// 按目标采样率/声道重建 I2S 并同步 codec 时钟
static bool i2s_config(int rate, bool stereo)
{
    if (g_i2s_on && rate == g_cur_rate && stereo == g_cur_stereo)
        return true;
    if (g_i2s_on)
    {
        g_i2s.end();
        g_i2s_on = false;
    }
    if (!g_i2s.begin(I2S_MODE_STD, rate, I2S_DATA_BIT_WIDTH_16BIT,
                     stereo ? I2S_SLOT_MODE_STEREO : I2S_SLOT_MODE_MONO,
                     I2S_STD_SLOT_BOTH))
    {
        Serial.println("[audio] i2s begin failed");
        return false;
    }
    g_i2s_on = true;
    g_cur_rate = rate;
    g_cur_stereo = stereo;
    if (g_es)
        es8311_sample_frequency_config(g_es, rate * 256, rate);
    Serial.printf("[audio] i2s %d Hz %s\n", rate, stereo ? "stereo" : "mono");
    return true;
}

bool audio_init(void)
{
    pinMode(PA_PIN, OUTPUT);
    digitalWrite(PA_PIN, LOW); // 功放默认关，播放时才开（空闲静音）

    g_es = es8311_create(0, ES8311_ADDRRES_0); // 与触摸共用 Wire(bus 0)
    if (!g_es)
    {
        Serial.println("[audio] es8311 create failed");
        return false;
    }
    const es8311_clock_config_t clk = {
        .mclk_inverted = false,
        .sclk_inverted = false,
        .mclk_from_mclk_pin = true,
        .mclk_frequency = 16000 * 256,
        .sample_frequency = 16000,
    };
    if (es8311_init(g_es, &clk, ES8311_RESOLUTION_16, ES8311_RESOLUTION_16) != ESP_OK)
    {
        Serial.println("[audio] es8311 init failed");
        return false;
    }
    es8311_voice_volume_set(g_es, 85, NULL);
    es8311_microphone_config(g_es, false);

    g_i2s.setPins(I2S_BCK_IO, I2S_WS_IO, I2S_DO_IO, I2S_DI_IO, I2S_MCK_IO);
    bool ok = i2s_config(16000, true);
    Serial.printf("[audio] init %s\n", ok ? "ok" : "FAIL");
    return ok;
}

// ---------- 极简 WAV 解析 ----------
static uint16_t rd16(const uint8_t *p) { return p[0] | (p[1] << 8); }
static uint32_t rd32(const uint8_t *p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t)p[3] << 24); }

// ---------- 航空电台音效 ----------
// 带通 300Hz~3kHz（航空甚高频信道特性）+ 软削波压缩 + 轻微白噪底噪
static void radio_fx(int16_t *s, size_t nSamples, int rate)
{
    const float dt = 1.0f / rate;
    const float aHP = (1.0f / (2 * PI * 300.0f)) / (1.0f / (2 * PI * 300.0f) + dt);
    const float aLP = dt / (1.0f / (2 * PI * 3000.0f) + dt);
    float prevX = 0, yHP = 0, yLP = 0;
    for (size_t i = 0; i < nSamples; i++)
    {
        float x = s[i] / 32768.0f;
        yHP = aHP * (yHP + x - prevX); // 高通，砍掉人声低频轰鸣
        prevX = x;
        yLP += aLP * (yHP - yLP);      // 低通，砍掉高频泛音
        float y = tanhf(yLP * 2.6f);   // 软削波 → 电台压缩感
        y += (int16_t)random(-260, 260) / 32768.0f; // 极轻底噪
        int v = (int)(y * 29500.0f);
        s[i] = (int16_t)constrain(v, -32768, 32767);
    }
}

bool audio_play_wav(const uint8_t *wav, size_t len)
{
    // 注意不能检查 g_i2s_on：每次播完 play_end() 都会关 I2S，i2s_config() 会按需重启
    if (len < 44 || memcmp(wav, "RIFF", 4) || memcmp(wav + 8, "WAVE", 4))
    {
        Serial.println("[audio] bad wav");
        return false;
    }
    // 遍历 chunk 找 fmt 和 data
    int rate = 0, channels = 0, bits = 0, fmtTag = 0;
    const uint8_t *pcm = nullptr;
    size_t pcmLen = 0;
    size_t off = 12;
    while (off + 8 <= len)
    {
        uint32_t csz = rd32(wav + off + 4);
        if (!memcmp(wav + off, "fmt ", 4) && off + 16 <= len)
        {
            fmtTag = rd16(wav + off + 8);
            channels = rd16(wav + off + 10);
            rate = rd32(wav + off + 12);
            bits = rd16(wav + off + 22);
        }
        else if (!memcmp(wav + off, "data", 4))
        {
            pcm = wav + off + 8;
            size_t avail = len - (off + 8); // 先算可用长度，避免 0xFFFFFFFF 溢出
            pcmLen = csz > avail ? avail : csz;
        }
        off += 8 + csz + (csz & 1);
    }
    if (!pcm || !pcmLen || fmtTag != 1 || bits != 16 || (channels != 1 && channels != 2) ||
        rate < 8000 || rate > 48000)
    {
        Serial.printf("[audio] unsupported wav: fmt=%d ch=%d bits=%d rate=%d pcm=%u\n",
                      fmtTag, channels, bits, rate, (unsigned)(pcm ? pcmLen : 0));
        return false;
    }
    // 降速播放：TTS 语速偏快，按 0.9x 输出（音调同步略降，更接近电台质感）
    const int playRate = (int)(rate * 0.90f);
    if (!i2s_config(playRate, channels == 2))
        return false;

    g_busy = true;
    digitalWrite(PA_PIN, HIGH); // 开功放
    delay(5);
    Serial.printf("[audio] play %u->%d Hz %s %u bytes\n", rate, playRate,
                  channels == 2 ? "stereo" : "mono", (unsigned)pcmLen);
    // 分块写，喂狗；每块先过一遍电台音效
    const uint8_t *p = pcm;
    size_t left = pcmLen;
    while (left)
    {
        size_t chunk = left > 16384 ? 16384 : left;
        radio_fx((int16_t *)p, chunk / 2, playRate); // 就地处理（PCM 缓冲是一次性的）
        size_t w = g_i2s.write((uint8_t *)p, chunk);
        if (!w)
            break;
        p += w;
        left -= w;
        if (left)
            delay(1);
    }
    play_end();
    g_busy = false;
    return left == 0;
}

void audio_play_tone(int freq, int ms)
{
    const int rate = 16000;
    int n = rate * ms / 1000;
    int16_t *buf = (int16_t *)heap_caps_malloc(n * 2, MALLOC_CAP_SPIRAM);
    if (!buf)
        return;
    for (int i = 0; i < n; i++)
    {
        float t = (float)i / rate;
        float env = 1.0f;
        int fade = rate / 50; // 20ms 淡入淡出，防爆音
        if (i < fade)
            env = (float)i / fade;
        else if (i > n - fade)
            env = (float)(n - i) / fade;
        buf[i] = (int16_t)(sinf(2 * PI * freq * t) * 12000 * env);
    }
    if (i2s_config(rate, false))
    {
        g_busy = true;
        digitalWrite(PA_PIN, HIGH);
        delay(5);
        g_i2s.write((uint8_t *)buf, n * 2);
        play_end();
        g_busy = false;
    }
    heap_caps_free(buf);
}
