#pragma once

// Waveshare ESP32-S3-Touch-AMOLED-2.16
// 引脚定义来自官方 examples/arduino/libraries/Mylibrary/pin_config.h

#define XPOWERS_CHIP_AXP2101

// LCD CO5300 (QSPI)
#define LCD_SDIO0 4
#define LCD_SDIO1 5
#define LCD_SDIO2 6
#define LCD_SDIO3 7
#define LCD_SCLK  38
#define LCD_RESET 39
#define LCD_CS    12
#define LCD_WIDTH  480
#define LCD_HEIGHT 480

// Touch CST9220 / PMU AXP2101 共用 I2C
#define IIC_SDA 15
#define IIC_SCL 14
#define TP_INT 11
#define TP_RST 40

// Audio: ES7210 (mic ADC) + ES8311 (DAC) + PA
#define PIN_ES7210_BCLK 9
#define PIN_ES7210_LRCK 45
#define PIN_ES7210_DIN  10
#define PIN_ES7210_MCLK 42
#define PIN_ES8311_DOUT 8

#define PA 46
