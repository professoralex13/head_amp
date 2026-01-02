#include "driver/i2s_std.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "freertos/task.h"
#include <math.h>
#include <stdio.h>

static const char *TAG = "WM8988";

// Pin definitions - adjust these for your hardware
#define SPI_MOSI_PIN 22
#define SPI_SCLK_PIN 32
#define SPI_CS_PIN 4

#define I2S_BCLK_PIN 16
#define I2S_WS_PIN 5
#define I2S_DOUT_PIN 17

// Audio parameters
#define SAMPLE_RATE 48000
#define TONE_FREQ 440  // A4 note
#define AMPLITUDE 8000 // 16-bit amplitude

// WM8988 Register addresses
#define WM8988_LIN_VOL 0x00
#define WM8988_RIN_VOL 0x01
#define WM8988_RESET 0x0F
#define WM8988_POWER_MGMT1 0x19
#define WM8988_POWER_MGMT2 0x1A
#define WM8988_AUDIO_INTERFACE 0x07
#define WM8988_SAMPLE_RATE 0x08
#define WM8988_LEFT_DAC_VOL 0x0A
#define WM8988_RIGHT_DAC_VOL 0x0B
#define WM8988_LEFT_OUT_MIX1 0x22
#define WM8988_LEFT_OUT_MIX2 0x23
#define WM8988_RIGHT_OUT_MIX1 0x24
#define WM8988_RIGHT_OUT_MIX2 0x25
#define WM8988_LOUT1_VOL 0x02
#define WM8988_ROUT1_VOL 0x03
#define WM8988_LOUT2_VOL 0x28
#define WM8988_ROUT2_VOL 0x29

spi_device_handle_t spi;
i2s_chan_handle_t tx_handle;

// Write to WM8988 register via SPI (3-wire mode)
esp_err_t wm8988_write_reg(uint8_t reg, uint16_t val) {
  // WM8988 3-wire SPI: 7-bit address + 9-bit data = 16 bits
  uint16_t data = ((reg & 0x7F) << 9) | (val & 0x1FF);

  spi_transaction_t t = {.length = 16,
                         .tx_data = {(data >> 8) & 0xFF, data & 0xFF},
                         .flags = SPI_TRANS_USE_TXDATA};

  esp_err_t ret = spi_device_polling_transmit(spi, &t);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "SPI write failed: %s", esp_err_to_name(ret));
  }
  vTaskDelay(pdMS_TO_TICKS(1)); // Small delay between writes
  return ret;
}

// Initialize SPI for WM8988 control
esp_err_t wm8988_spi_init(void) {
  spi_bus_config_t buscfg = {.mosi_io_num = SPI_MOSI_PIN,
                             .miso_io_num = -1, // Not used in 3-wire mode
                             .sclk_io_num = SPI_SCLK_PIN,
                             .quadwp_io_num = -1,
                             .quadhd_io_num = -1,
                             .max_transfer_sz = 32};

  esp_err_t ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_DISABLED);
  if (ret != ESP_OK)
    return ret;

  spi_device_interface_config_t devcfg = {.clock_speed_hz = 1000000, // 1 MHz
                                          .mode = 0, // SPI mode 0
                                          .spics_io_num = SPI_CS_PIN,
                                          .queue_size = 1,
                                          .flags = SPI_DEVICE_NO_DUMMY};

  return spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
}

// Configure WM8988 codec
esp_err_t wm8988_configure(void) {
  ESP_LOGI(TAG, "Configuring WM8988...");

  // Reset codec
  wm8988_write_reg(WM8988_RESET, 0);
  vTaskDelay(pdMS_TO_TICKS(10));

  wm8988_write_reg(WM8988_LIN_VOL, 0b100011111);
  wm8988_write_reg(WM8988_RIN_VOL, 0b100011111);

  // Power Management 1: Enable VMIDSEL, VREF, DACL, DACR
  wm8988_write_reg(WM8988_POWER_MGMT1, 0b011111100);

  // Power Management 2: Enable LOUT1, ROUT1, LOUT2, ROUT2
  wm8988_write_reg(WM8988_POWER_MGMT2, 0b111100000);

  // Audio Interface: I2S format, 16-bit
  wm8988_write_reg(WM8988_AUDIO_INTERFACE, 0b000000010);

  // Sample Rate: 48kHz
  wm8988_write_reg(WM8988_SAMPLE_RATE, 0b000000000);

  // DAC Digital Volume: 0dB (0xFF = 0dB, update both channels)
  wm8988_write_reg(WM8988_LEFT_DAC_VOL, 0b111111111);
  wm8988_write_reg(WM8988_RIGHT_DAC_VOL, 0b11111111);

  // Left Output Mixer: Enable Left DAC to Left Output Mixer
  wm8988_write_reg(WM8988_LEFT_OUT_MIX1, 0b110000011);
  wm8988_write_reg(WM8988_LEFT_OUT_MIX2, 0b001010000);

  // Right Output Mixer: Enable Right DAC to Right Output Mixer
  wm8988_write_reg(WM8988_RIGHT_OUT_MIX1, 0b001010000);
  wm8988_write_reg(WM8988_RIGHT_OUT_MIX2, 0b110000011);

  // LOUT1/ROUT1 Volume: 0dB (0x79 = 0dB, update enabled)
  wm8988_write_reg(WM8988_LOUT1_VOL, 0b101111111);
  wm8988_write_reg(WM8988_ROUT1_VOL, 0b101111111);

  // LOUT2/ROUT2 Volume: 0dB
  wm8988_write_reg(WM8988_LOUT2_VOL, 0x179);
  wm8988_write_reg(WM8988_ROUT2_VOL, 0x179);

  ESP_LOGI(TAG, "WM8988 configured successfully");
  return ESP_OK;
}

// Initialize I2S
esp_err_t i2s_init(void) {
  i2s_chan_config_t chan_cfg =
      I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  esp_err_t ret = i2s_new_channel(&chan_cfg, &tx_handle, NULL);
  if (ret != ESP_OK)
    return ret;

  i2s_std_config_t std_cfg = {
      .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
      .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
                                                      I2S_SLOT_MODE_STEREO),
      .gpio_cfg =
          {
              .mclk = 0,
              .bclk = I2S_BCLK_PIN,
              .ws = I2S_WS_PIN,
              .dout = I2S_DOUT_PIN,
              .din = I2S_GPIO_UNUSED,
              .invert_flags =
                  {
                      .mclk_inv = false,
                      .bclk_inv = false,
                      .ws_inv = false,
                  },
          },
  };

  ret = i2s_channel_init_std_mode(tx_handle, &std_cfg);
  if (ret != ESP_OK)
    return ret;

  return i2s_channel_enable(tx_handle);
}

// Generate and play tone
void play_tone_task(void *pvParameters) {
  const int buf_size = 512;
  int16_t *samples = malloc(buf_size * sizeof(int16_t) * 2); // Stereo

  if (!samples) {
    ESP_LOGE(TAG, "Failed to allocate sample buffer");
    vTaskDelete(NULL);
    return;
  }

  ESP_LOGI(TAG, "Playing %d Hz tone...", TONE_FREQ);

  float phase = 0.0f;
  float phase_inc = 2.0f * M_PI * TONE_FREQ / SAMPLE_RATE;

  while (1) {
    // Generate samples
    for (int i = 0; i < buf_size; i++) {
      int16_t sample = (int16_t)(sinf(phase) * AMPLITUDE);
      samples[i * 2] = sample;     // Left channel
      samples[i * 2 + 1] = sample; // Right channel

      phase += phase_inc;
      if (phase >= 2.0f * M_PI) {
        phase -= 2.0f * M_PI;
      }
    }

    // Write to I2S
    size_t bytes_written;
    i2s_channel_write(tx_handle, samples, buf_size * sizeof(int16_t) * 2,
                      &bytes_written, portMAX_DELAY);
  }
}

void app_main(void) {
  ESP_LOGI(TAG, "Starting WM8988 tone generator");

  // Initialize SPI for WM8988 control
  if (wm8988_spi_init() != ESP_OK) {
    ESP_LOGE(TAG, "SPI initialization failed");
    return;
  }

  // Configure WM8988
  if (wm8988_configure() != ESP_OK) {
    ESP_LOGE(TAG, "WM8988 configuration failed");
    return;
  }

  // Initialize I2S for audio output
  if (i2s_init() != ESP_OK) {
    ESP_LOGE(TAG, "I2S initialization failed");
    return;
  }

  // Start tone generation task
  xTaskCreate(play_tone_task, "play_tone", 4096, NULL, 5, NULL);
}