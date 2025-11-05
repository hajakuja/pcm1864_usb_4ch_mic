#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include "driver/i2s_tdm.h"
#include "driver/i2s_common.h"
#include "esp_log.h"

#include "pins.h"
#include "pcm1864.h"
#include "audio_hw.h"

// Logging tag
static const char *TAG = "audio_hw";

// I2C setup
static bool i2c_bus_init(void) {
  i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = I2C_SDA_GPIO,
      .scl_io_num = I2C_SCL_GPIO,
      .sda_pullup_en = true,
      .scl_pullup_en = true,
      .master.clk_speed = I2C_CLK_HZ,
  };
  ESP_ERROR_CHECK(i2c_param_config(I2C_PORT_NUM, &conf));
  ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT_NUM, conf.mode, 0, 0, 0));
  return true;
}

// PCM1864 device instance
static pcm1864_t s_pcm1864;

// I2S channel (RX)
static i2s_chan_handle_t s_rx_chan = NULL;

// TDM config constants
#define FRAMES_PER_MS           (AUDIO_SAMPLE_RATE_HZ / 1000)
#define RX_BYTES_PER_SAMPLE     4   // 32-bit slot (even if 24-bit data)
#define RX_MS_BYTES             (FRAMES_PER_MS * AUDIO_CHANNELS * RX_BYTES_PER_SAMPLE) // 768
#define USB_MS_BYTES            (FRAMES_PER_MS * AUDIO_CHANNELS * AUDIO_SAMPLE_BYTES_USB) // 384 for 16-bit

// Simple ring buffer of 1ms TDM frames
#define RING_CAP_MS             16
static uint8_t s_ring[RING_CAP_MS][RX_MS_BYTES];
static volatile uint32_t s_ring_head = 0; // write index
static volatile uint32_t s_ring_tail = 0; // read index
static SemaphoreHandle_t s_ring_mutex;

static inline bool ring_is_full(void) {
  return ((s_ring_head + 1) % RING_CAP_MS) == s_ring_tail;
}
static inline bool ring_is_empty(void) {
  return s_ring_head == s_ring_tail;
}
static inline void ring_push(const uint8_t *src) {
  memcpy(s_ring[s_ring_head], src, RX_MS_BYTES);
  s_ring_head = (s_ring_head + 1) % RING_CAP_MS;
}
static inline bool ring_pop(uint8_t *dst) {
  if (ring_is_empty()) return false;
  memcpy(dst, s_ring[s_ring_tail], RX_MS_BYTES);
  s_ring_tail = (s_ring_tail + 1) % RING_CAP_MS;
  return true;
}

// Convert one 1ms frame of 32-bit-slot TDM data to 16-bit interleaved USB payload
static void tdm32_to_usb16(const uint8_t *in32, uint8_t *out16) {
  // Assumes 32-bit little-endian words with 24-bit valid data.
  // We right-shift 8 bits to convert 24->16, simple truncation.
  const int32_t *inw = (const int32_t *)in32;
  int16_t *outw = (int16_t *)out16;
  for (int f = 0; f < FRAMES_PER_MS; ++f) {
    for (int ch = 0; ch < AUDIO_CHANNELS; ++ch) {
      int32_t s32 = inw[f * AUDIO_CHANNELS + ch];
      int16_t s16 = (int16_t)(s32 >> 8); // drop LSBs
      *outw++ = s16;
    }
  }
}

// I2S reader task: read exactly 1ms worth of TDM data and push to ring
static void i2s_reader_task(void *arg) {
  (void)arg;
  uint8_t ms_buf[RX_MS_BYTES];
  size_t read_bytes = 0;
  for (;;) {
    // Read one millisecond block
    read_bytes = 0;
    while (read_bytes < RX_MS_BYTES) {
      size_t just_read = 0;
      esp_err_t err = i2s_channel_read(s_rx_chan, ms_buf + read_bytes, RX_MS_BYTES - read_bytes, &just_read, portMAX_DELAY);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_read err=%d", (int)err);
        vTaskDelay(pdMS_TO_TICKS(1));
        continue;
      }
      read_bytes += just_read;
    }

    // Push into ring (drop oldest on overflow)
    if (xSemaphoreTake(s_ring_mutex, portMAX_DELAY) == pdTRUE) {
      if (ring_is_full()) {
        // drop one old frame
        s_ring_tail = (s_ring_tail + 1) % RING_CAP_MS;
      }
      ring_push(ms_buf);
      xSemaphoreGive(s_ring_mutex);
    }
  }
}

bool audio_hw_init(void) {
  // I2C
  ESP_LOGI(TAG, "Init I2C on port %d (SCL=%d SDA=%d)", I2C_PORT_NUM, I2C_SCL_GPIO, I2C_SDA_GPIO);
  if (!i2c_bus_init()) return false;

  // PCM1864 at provided 0x94 (8-bit) -> 0x4A (7-bit)
  uint8_t addr7 = (uint8_t)(PCM1864_I2C_ADDR_8BIT >> 1);
  pcm1864_init_struct(&s_pcm1864, I2C_PORT_NUM, addr7);

  // Configure codec (placeholder sequence)
  if (!pcm1864_configure_tdm_4ch_48k(&s_pcm1864)) {
    ESP_LOGW(TAG, "PCM1864 configuration is placeholder; please update register writes");
  }

  // I2S channel allocation (RX only, master)
  i2s_chan_config_t chan_cfg = {
      .id = I2S_NUM_0,
      .role = I2S_ROLE_MASTER,
      .dma_desc_num = 8,
      .dma_frame_num = FRAMES_PER_MS, // 1ms per DMA frame
      .auto_clear = true,
  };
  ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, NULL, &s_rx_chan));

  // TDM clock config
  i2s_tdm_clock_config_t clk_cfg = I2S_TDM_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE_HZ);
  clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256; // common multiple

  // TDM slot config: 32-bit slots, 24-bit valid data, 4 active channels
  i2s_tdm_slot_config_t slot_cfg = I2S_TDM_PHILIPS_SLOT_DEFAULT_CONFIG(AUDIO_SLOT_BITS, I2S_SLOT_MODE_MONO);
  slot_cfg.slot_bit_width = AUDIO_SLOT_BITS;
  slot_cfg.data_bit_width = I2S_DATA_BIT_WIDTH_24BIT;
  slot_cfg.total_slot_num = AUDIO_CHANNELS; // 4 active slots
  slot_cfg.active_chan_mask = (1ULL << AUDIO_CHANNELS) - 1ULL; // first N slots

  // TDM GPIO config
  i2s_tdm_gpio_config_t gpio_cfg = {
      .mclk = I2S_MCLK_GPIO,
      .bclk = I2S_BCLK_GPIO,
      .ws = I2S_WS_GPIO,
      .dout = I2S_GPIO_UNUSED,
      .din = I2S_DIN_GPIO,
      .invert_flags = {
          .mclk_inv = false,
          .bclk_inv = false,
          .ws_inv = false,
      },
  };

  // Initialize RX channel in TDM mode
  ESP_ERROR_CHECK(i2s_channel_init_tdm_mode(s_rx_chan, &clk_cfg, &slot_cfg, &gpio_cfg));

  // Enable channel
  ESP_ERROR_CHECK(i2s_channel_enable(s_rx_chan));

  // Ring buffer
  s_ring_mutex = xSemaphoreCreateMutex();
  s_ring_head = s_ring_tail = 0;

  return true;
}

bool audio_hw_start(void) {
  BaseType_t ok = xTaskCreatePinnedToCPU(i2s_reader_task, "i2s_reader", 4096, NULL, configMAX_PRIORITIES - 1, NULL, tskNO_AFFINITY);
  return ok == pdPASS;
}

size_t audio_hw_pop_usb_1ms(uint8_t *dst, size_t dst_bytes) {
  if (dst_bytes < USB_MS_BYTES) return 0;
  uint8_t rx_ms[RX_MS_BYTES];
  bool have = false;
  if (xSemaphoreTake(s_ring_mutex, pdMS_TO_TICKS(1)) == pdTRUE) {
    have = ring_pop(rx_ms);
    xSemaphoreGive(s_ring_mutex);
  }
  if (!have) {
    memset(dst, 0, USB_MS_BYTES);
    return 0;
  }
  tdm32_to_usb16(rx_ms, dst);
  return USB_MS_BYTES;
}
