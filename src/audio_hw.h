#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Initialize I2C + I2S/TDM and internal buffers
bool audio_hw_init(void);

// Start the I2S reader task
bool audio_hw_start(void);

// Try to emit one 1ms USB frame (16-bit interleaved ch0..ch3)
// Returns number of bytes written to dst (0 if underflow)
size_t audio_hw_pop_usb_1ms(uint8_t *dst, size_t dst_bytes);

#ifdef __cplusplus
}
#endif

