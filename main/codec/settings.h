#ifndef CODEC_SETTINGS_H
#define CODEC_SETTINGS_H

#include "codec/spi.h"
#include "stdint.h"

typedef enum {
  Left,
  Right,
} Channel;

#define MAX_INPUT_VOLUME 0b111111

esp_err_t set_input_volume(spi_codec_device device, Channel channel,
                           uint8_t volume);

#define MAX_OUTPUT_VOLUME 0b1111111

esp_err_t set_output_volume(spi_codec_device device, Channel channel,
                            uint8_t volume);

esp_err_t set_digital_audio_interface(spi_codec_device device,
                                      uint8_t word_length);

esp_err_t set_power_management(spi_codec_device device, bool adc_left,
                               bool adc_right, bool dac_left, bool dac_right,
                               bool lout1, bool rout1);

#define MAX_DAC_VOLUME 0b11111111

esp_err_t set_dac_volume(spi_codec_device device, Channel channel,
                         uint8_t volume);

esp_err_t reset_registers(spi_codec_device device);

#define MAX_MIX_VOLUME 0b111

esp_err_t set_output_mix(spi_codec_device device, Channel channel,
                         uint8_t leftVolume, uint8_t rightVolume);

#endif