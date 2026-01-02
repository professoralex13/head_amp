#include "settings.h"
#include "esp_err.h"
#include "esp_log.h"
#include "spi.h"
#include <stdbool.h>

static const char *TAG = "WM8988";

typedef enum {
  LeftInputVolume = 0x00,
  RightInputVolume = 0x01,
  LeftOutput1Volume = 0x02,
  RightOutput1Volume = 0x03,
  ADCDACControl = 0x05,
  AudioInterface = 0x07,
  SampleRate = 0x08,
  LeftDACVolume = 0x0A,
  RightDACVolume = 0x0B,
  BassControl = 0x0C,
  TrebleControl = 0x0D,
  Reset = 0x0F,
  Control3D = 0x10,
  ALC1 = 0x11,
  ALC2 = 0x12,
  ALC3 = 0x13,
  NoiseGate = 0x14,
  LeftADCVolume = 0x15,
  RightADCVolume = 0x16,
  AdditionalControl1 = 0x17,
  AdditionalControl2 = 0x18,
  PowerManagement1 = 0x19,
  PowerManagement2 = 0x1A,
  AdditionalControl3 = 0x1B,
  ADCInputMode = 0x1F,
  ADCLSignalPath = 0x20,
  ADCRSignalPath = 0x21,
  LeftOutMix1 = 0x22,
  LeftOutMix2 = 0x23,
  RightOutMix1 = 0x24,
  RightOutMix2 = 0x25,
  LeftOutput2Volume = 0x28,
  RightOutput2Volume = 0x29,
  LowPowerPlayback = 0x43,
} RegisterAddress;

esp_err_t set_input_volume(spi_codec_device device, Channel channel,
                           uint8_t volume) {
  uint8_t address = channel == Left ? LeftInputVolume : RightInputVolume;

  if (volume > MAX_INPUT_VOLUME) {
    ESP_LOGE(TAG, "Cannot set input volume greater than %d", MAX_INPUT_VOLUME);

    return ESP_FAIL;
  }

  bool update_immediate = true;
  bool mute = volume == 0;
  bool zero_cross_detector = false;

  uint16_t data = (update_immediate << 8) | (mute << 7) |
                  (zero_cross_detector << 6) | (volume & 0b111111);

  return write_register(address, data, device);
}

esp_err_t set_output_volume(spi_codec_device device, Channel channel,
                            uint8_t volume) {
  uint8_t address = channel == Left ? LeftOutput1Volume : RightOutput1Volume;

  if (volume > MAX_OUTPUT_VOLUME) {
    ESP_LOGE(TAG, "Cannot set output volume greater than %d",
             MAX_OUTPUT_VOLUME);

    return ESP_FAIL;
  }

  bool update_immediate = true;
  bool zero_cross_detector = false;

  uint16_t data = (update_immediate << 8) | (zero_cross_detector << 7) |
                  (volume & 0b1111111);

  return write_register(address, data, device);
}

const int VALID_AUDIO_WORD_LENGTHS[] = {16, 20, 24, 32};

typedef enum {
  LeftJustified = 0b01,
  I2S = 0b10,
  DSP = 0b11,
} AudioDataFormat;

esp_err_t set_digital_audio_interface(spi_codec_device device,
                                      uint8_t word_length) {
  bool invert_bclk = false;
  bool master_mode = false;
  bool swap_left_right = false;
  bool invert_lrc_polarity = false;

  uint8_t word_length_repr = 0b111;

  for (int i = 0; i < 4; i++) {
    if (VALID_AUDIO_WORD_LENGTHS[i] == word_length) {
      word_length_repr = i;
    }
  }

  if (word_length_repr == 0b111) {
    ESP_LOGE(TAG, "Invalid I2S word length value");

    return ESP_FAIL;
  }

  uint8_t audio_format = I2S;

  uint8_t data = (invert_bclk << 7) | (master_mode << 6) |
                 (swap_left_right << 5) | (invert_lrc_polarity << 4) |
                 ((word_length_repr & 0b11) << 2) | (audio_format & 0b11);

  return write_register(AudioInterface, data, device);
}

esp_err_t set_power_management(spi_codec_device device, bool adc_left,
                               bool adc_right, bool dac_left, bool dac_right,
                               bool lout1, bool rout1) {
  uint8_t vmid_selection = 0b01;
  bool vref = true;

  bool pga_left = true;
  bool pga_right = true;

  bool lout2 = false;
  bool rout2 = false;

  bool master_clk_disabled = false;

  uint16_t data1 = ((vmid_selection & 0b11) << 7) | (vref << 6) |
                   (pga_left << 5) | (pga_right << 4) | (adc_left << 3) |
                   (adc_right << 2);
  uint16_t data2 = (dac_left << 8) | (dac_right << 7) | (lout1 << 6) |
                   (rout1 << 5) | (lout2 << 4) | (rout2 << 3) |
                   master_clk_disabled;

  esp_err_t result = write_register(PowerManagement1, data1, device);

  if (result != ESP_OK) {
    return result;
  }

  return write_register(PowerManagement2, data2, device);
}

esp_err_t set_dac_volume(spi_codec_device device, Channel channel,
                         uint8_t volume) {
  uint8_t address = channel == Left ? LeftDACVolume : RightDACVolume;

  if (volume > MAX_DAC_VOLUME) {
    ESP_LOGE(TAG, "Cannot set DAC volume greater than %d", MAX_DAC_VOLUME);

    return ESP_FAIL;
  }

  bool update_immediate = true;

  uint16_t data = (update_immediate << 8) | volume;

  return write_register(address, data, device);
}

esp_err_t reset_registers(spi_codec_device device) {
  return write_register(Reset, 0, device);
}

typedef enum {
  Input1 = 0b000,
  Input2 = 0b001,
  MicBoost = 0b011,
  Differential = 0b100,
} MixInputSelection;

esp_err_t set_output_mix(spi_codec_device device, Channel channel,
                         uint8_t leftVolume, uint8_t rightVolume) {
  uint8_t address1 = channel == Left ? LeftOutMix1 : RightOutMix1;
  uint8_t address2 = channel == Left ? LeftOutMix2 : RightOutMix2;

  if (leftVolume > MAX_MIX_VOLUME || rightVolume > MAX_MIX_VOLUME) {
    ESP_LOGE(TAG, "Cannot set mix volume greater than %d", MAX_MIX_VOLUME);

    return ESP_FAIL;
  }

  MixInputSelection mixInputSelection = MicBoost;

  bool leftDac = channel == Left;
  bool rightDac = channel == Right;

  bool leftMixEnabled = leftVolume != 0;
  bool rightMixEnabled = rightVolume != 0;

  // Register 1 controls inputs from left mix and left DAC into this channel,
  // as well as the input selection for this channel (which may feed into the
  // other channels mix)
  uint16_t data1 = (leftDac << 8) | (leftMixEnabled << 7) |
                   (((MAX_MIX_VOLUME - leftVolume) & 0b111) << 4) |
                   (mixInputSelection & 0b111);

  // Register 2 controls inputs from the right mix and right DAC into this
  // channel
  uint16_t data2 = (rightDac << 8) | (rightMixEnabled << 7) |
                   (((MAX_MIX_VOLUME - rightVolume) & 0b111) << 4);

  esp_err_t result = write_register(address1, data1, device);

  if (result != ESP_OK) {
    return result;
  }

  return write_register(address2, data2, device);
}

esp_err_t set_dac_mute(spi_codec_device device, bool mute) {
  bool adc_attenuate = false;
  bool dac_attenuate = false;

  bool adchpd = false;

  bool hpor = false;
  uint8_t adcpol = 0b00;

  uint8_t demphasis = 0b00;

  uint16_t value = (adc_attenuate << 8) | (dac_attenuate << 7) |
                   ((adchpd & 0b11) << 5) | (hpor << 4) | (mute << 3) |
                   ((demphasis & 0b11) << 1) | adcpol;

  return write_register(ADCDACControl, value, device);
}