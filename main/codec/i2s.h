#ifndef CODEC_I2S_H
#define CODEC_I2S_H

#include "driver/i2s_types.h"
#include "esp_err.h"

#define SAMPLE_RATE 48000

esp_err_t i2s_device_init(i2s_chan_handle_t *tx_handle,
                          i2s_chan_handle_t *rx_handle, uint8_t bclk_pin,
                          uint8_t ws_pin, uint8_t dout_pin, uint8_t din_pin);
#endif