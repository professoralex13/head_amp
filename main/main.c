#include "codec/settings.h"
#include "codec/spi.h"

//! Changed between board versions V1.0 and V1.1 due to unsuitable GPIO issues
#define SPI_MOSI_PIN 22

#define SPI_CLK_PIN 32

#define EXT_INT_CSB_PIN 4

void app_main(void) {
  spi_bus_init(SPI_CLK_PIN, SPI_MOSI_PIN);

  spi_codec_device ext_int_codec;

  spi_device_init(EXT_INT_CSB_PIN, &ext_int_codec);

  reset_registers(ext_int_codec);

  set_power_management(ext_int_codec, false, false, true, true, true, true);

  set_digital_audio_interface(ext_int_codec, 32);

  set_dac_volume(ext_int_codec, Left, 0);
  set_dac_volume(ext_int_codec, Right, 0);

  set_input_volume(ext_int_codec, Left, MAX_INPUT_VOLUME);
  set_input_volume(ext_int_codec, Right, MAX_INPUT_VOLUME);

  set_output_mix(ext_int_codec, Left, MAX_MIX_VOLUME, 0);
  set_output_mix(ext_int_codec, Right, 0, MAX_MIX_VOLUME);

  set_output_volume(ext_int_codec, Left, MAX_OUTPUT_VOLUME);
  set_output_volume(ext_int_codec, Right, MAX_OUTPUT_VOLUME);
}
