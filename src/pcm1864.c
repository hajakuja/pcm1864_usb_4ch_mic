#include "pcm1864.h"
#include "pins.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

// NOTE: The register map below is intentionally expressed as symbolic
// names with descriptive comments. Please verify numerical addresses
// and field values against your PCM1864 datasheet and update as needed.

// Common control registers (replace with correct addresses)
#define PCM1864_REG_SW_RESET            0x00  // Software reset
#define PCM1864_REG_PWR_CTRL            0x01  // Power / clock control

// Audio Serial Interface (ASI) / TDM configuration (replace with correct addresses)
#define PCM1864_REG_ASI_FMT_1           0x10  // I2S/TDM format, master/slave select
#define PCM1864_REG_ASI_FMT_2           0x11  // Word length, LRCLK polarity, BCLK edge
#define PCM1864_REG_TDM_CTRL_1          0x12  // TDM enable, slots per frame
#define PCM1864_REG_TDM_CTRL_2          0x13  // Slot width selection
#define PCM1864_REG_SDOUT_MAP           0x14  // SDOUT channel map / first slot

// ADC / mixer / mute control (replace with correct addresses)
#define PCM1864_REG_ADC_MUTE            0x20  // ADC digital mute
#define PCM1864_REG_ADC_GAIN_CH1        0x21  // PGA/Gain channel 1
#define PCM1864_REG_ADC_GAIN_CH2        0x22  // PGA/Gain channel 2
#define PCM1864_REG_ADC_GAIN_CH3        0x23  // PGA/Gain channel 3
#define PCM1864_REG_ADC_GAIN_CH4        0x24  // PGA/Gain channel 4

// Convenience bitfield templates (update to match datasheet bit positions)
#define PCM1864_SW_RESET_ASSERT         0x01

// PWR: enable core clocks (exact bits TBD)
#define PCM1864_PWR_ENABLE_ALL          0xFF

// ASI_FMT_1: Slave mode, TDM/I2S format select (TDM, Philips)
#define PCM1864_ASI_SLAVE               0x00
#define PCM1864_ASI_MASTER              0x80
#define PCM1864_ASI_FMT_I2S             0x00
#define PCM1864_ASI_FMT_LEFT_J          0x01
#define PCM1864_ASI_FMT_TDM             0x02

// ASI_FMT_2: 24-bit data, standard polarity (update to match datasheet)
#define PCM1864_ASI_WORDLEN_24B         0x20
#define PCM1864_ASI_LRCLK_NORMAL        0x00
#define PCM1864_ASI_BCLK_NORMAL         0x00

// TDM_CTRL_1: Enable TDM, 4 slots
#define PCM1864_TDM_ENABLE              0x80
#define PCM1864_TDM_SLOTS_4             0x03

// TDM_CTRL_2: Slot width = 32 bits
#define PCM1864_TDM_SLOT_32BIT          0x20

// SDOUT map: start at slot 0 (ch1), increment
#define PCM1864_SDOUT_START_SLOT_0      0x00

// ADC mute off, unity gains (exact scale per datasheet)
#define PCM1864_ADC_UNMUTE              0x00
#define PCM1864_ADC_GAIN_UNITY          0x00

// Convert provided 8-bit address to 7-bit if needed
static inline uint8_t addr8_to_7(uint8_t addr8) {
  return (uint8_t)(addr8 >> 1);
}

bool pcm1864_write_reg(pcm1864_t *dev, uint8_t reg, uint8_t val) {
  uint8_t buf[2] = { reg, val };
  esp_err_t err = i2c_master_write_to_device(dev->i2c_port, dev->i2c_addr_7bit, buf, sizeof(buf), 100 / portTICK_PERIOD_MS);
  if (err != ESP_OK) {
    ESP_LOGE("pcm1864", "I2C write failed reg=0x%02X val=0x%02X err=%d", reg, val, (int)err);
    return false;
  }
  return true;
}

bool pcm1864_configure_tdm_4ch_48k(pcm1864_t *dev) {
  // Recommended MCLK from ESP32-S3 is set to 256xfs in audio_hw.c
  // Ensure PCM1864 clocking matches: Slave BCLK/LRCLK, MCLK provided.

  // 1) Software reset
  if (!pcm1864_write_reg(dev, PCM1864_REG_SW_RESET, PCM1864_SW_RESET_ASSERT)) return false;
  vTaskDelay(pdMS_TO_TICKS(5));

  // 2) Power/clock enable (ensure internal blocks powered)
  if (!pcm1864_write_reg(dev, PCM1864_REG_PWR_CTRL, PCM1864_PWR_ENABLE_ALL)) return false;

  // 3) Audio serial interface: Slave, TDM (Philips style)
  if (!pcm1864_write_reg(dev, PCM1864_REG_ASI_FMT_1, (PCM1864_ASI_SLAVE | PCM1864_ASI_FMT_TDM))) return false;

  // 4) Word length/polarity: 24-bit data, normal LRCLK/BCLK polarity per ESP defaults
  if (!pcm1864_write_reg(dev, PCM1864_REG_ASI_FMT_2, (PCM1864_ASI_WORDLEN_24B | PCM1864_ASI_LRCLK_NORMAL | PCM1864_ASI_BCLK_NORMAL))) return false;

  // 5) TDM control: enable TDM, 4 slots
  if (!pcm1864_write_reg(dev, PCM1864_REG_TDM_CTRL_1, (PCM1864_TDM_ENABLE | PCM1864_TDM_SLOTS_4))) return false;

  // 6) TDM slot width: 32-bit slots (24-bit valid data)
  if (!pcm1864_write_reg(dev, PCM1864_REG_TDM_CTRL_2, PCM1864_TDM_SLOT_32BIT)) return false;

  // 7) SDOUT mapping: start at slot 0 with CH1, CH2, CH3, CH4 in order
  if (!pcm1864_write_reg(dev, PCM1864_REG_SDOUT_MAP, PCM1864_SDOUT_START_SLOT_0)) return false;

  // 8) Unmute ADCs and set unity gain (adjust per your analog front-end)
  if (!pcm1864_write_reg(dev, PCM1864_REG_ADC_MUTE, PCM1864_ADC_UNMUTE)) return false;
  if (!pcm1864_write_reg(dev, PCM1864_REG_ADC_GAIN_CH1, PCM1864_ADC_GAIN_UNITY)) return false;
  if (!pcm1864_write_reg(dev, PCM1864_REG_ADC_GAIN_CH2, PCM1864_ADC_GAIN_UNITY)) return false;
  if (!pcm1864_write_reg(dev, PCM1864_REG_ADC_GAIN_CH3, PCM1864_ADC_GAIN_UNITY)) return false;
  if (!pcm1864_write_reg(dev, PCM1864_REG_ADC_GAIN_CH4, PCM1864_ADC_GAIN_UNITY)) return false;

  ESP_LOGI("pcm1864", "Configured PCM1864 for TDM 4ch, 48k, 24->16 conversion");
  return true;
}
