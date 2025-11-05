#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/i2c.h"

// Basic PCM1864 control over I2C. Register map programming must be
// adapted to your exact hardware and clocking. This header provides
// the interface used by audio_hw.

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  i2c_port_t i2c_port;
  uint8_t i2c_addr_7bit; // 7-bit I2C address
} pcm1864_t;

// Initialize device structure (does not touch hardware)
static inline void pcm1864_init_struct(pcm1864_t *dev, i2c_port_t port, uint8_t addr_7bit) {
  dev->i2c_port = port;
  dev->i2c_addr_7bit = addr_7bit;
}

// Low-level write to one register
bool pcm1864_write_reg(pcm1864_t *dev, uint8_t reg, uint8_t val);

// Configure PCM1864 for 4ch TDM, 48 kHz, 24-bit samples in 32-bit slots.
// NOTE: The actual register sequence depends on your clocking and desired
// gains/routing. This function contains a minimal placeholder sequence and
// should be adjusted per the datasheet.
bool pcm1864_configure_tdm_4ch_48k(pcm1864_t *dev);

#ifdef __cplusplus
}
#endif

