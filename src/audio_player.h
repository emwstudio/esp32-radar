#pragma once
#include <Arduino.h>

// 初始化 ES8311 codec + I2S（要求 Wire 已被触摸初始化）。
// 返回是否成功。
bool audio_init(void);

// 播放一段 WAV（整个文件在内存里）。自动解析采样率/声道/位深，
// 只接受 PCM 16-bit。返回是否成功。
bool audio_play_wav(const uint8_t *wav, size_t len);

// 测试音：freq Hz 正弦波 ms 毫秒（16kHz 单声道）
void audio_play_tone(int freq, int ms);

// 播放状态（上一轮是否还在播）
bool audio_busy(void);
