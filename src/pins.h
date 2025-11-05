// Pin and bus definitions for ESP32-S3 (placeholders — change to match your board)
#pragma once

#include <stdint.h>

// I2C
#define I2C_PORT_NUM           0            // I2C port index
#define I2C_SCL_GPIO           9            // TODO: set to your SCL pin
#define I2C_SDA_GPIO           8            // TODO: set to your SDA pin
#define I2C_CLK_HZ             400000       // 400 kHz

// PCM1864 I2C address
// Note: ESP-IDF I2C APIs use 7-bit addressing. You provided 0x94; if that is the
// 8-bit write address, the corresponding 7-bit address is 0x4A. This code derives
// the 7-bit address from the 8-bit one.
#define PCM1864_I2C_ADDR_8BIT  0x94

// I2S/TDM RX
#define I2S_MCLK_GPIO          2            // TODO: set valid MCLK-capable pin for S3
#define I2S_BCLK_GPIO          3            // TODO: set to your BCLK pin
#define I2S_WS_GPIO            4            // TODO: set to your FS/WS pin
#define I2S_DIN_GPIO           5            // TODO: set to your SDOUT from PCM1864

// Audio format
#define AUDIO_SAMPLE_RATE_HZ   48000
#define AUDIO_CHANNELS         4
#define AUDIO_SLOT_BITS        32           // TDM slot width (PCM1864 usually 24-bit data in 32-bit slots)
#define AUDIO_SAMPLE_BITS_ADC  24           // ADC resolution from PCM1864
#define AUDIO_SAMPLE_BYTES_USB 2            // 16-bit packed to USB

