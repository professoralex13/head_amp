/**
 * This file contains logic for controlling the dual WM8988 codecs using the SPI
 * protocol
 */

#include "spi.h"
#include "driver/spi_master.h"
#include "esp_log.h"

static const char *TAG = "WM8988";

/**
 * Initializes the ESP32s SPI2 driver for controlling WM8988s CODECs
 */
esp_err_t spi_bus_init(uint8_t clock_pin, uint8_t data_pin) {
  spi_bus_config_t bus_config = {
      .sclk_io_num = clock_pin,
      .mosi_io_num = data_pin,
      .miso_io_num = -1,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 32,
  };

  return spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_DISABLED);
}

/**
 * Initializes an SPI device on the SPI2 driver to control a WM8988 CODEC
 */
esp_err_t spi_device_init(uint8_t chip_select_pin, spi_codec_device *handle) {
  spi_device_interface_config_t device_config = {
      .clock_speed_hz = SPI_FREQUENCY,
      .mode = 0,
      .spics_io_num = chip_select_pin,
      .queue_size = 1,
      .flags = SPI_DEVICE_NO_DUMMY,
  };

  return spi_bus_add_device(SPI2_HOST, &device_config, handle);
}

esp_err_t write_register(uint8_t address, uint16_t value,
                         spi_codec_device device) {
  // Each SPI word consists of a 7 bit address, followed by a 9 bit value
  uint16_t data = ((address & 0x7F) << 9) | (value & 0x1FF);

  spi_transaction_t transaction = {
      .length = 16,
      .tx_data = {(data >> 8) & 0xFF, data & 0xFF},
      .flags = SPI_TRANS_USE_TXDATA,
  };

  esp_err_t result = spi_device_polling_transmit(device, &transaction);

  if (result != ESP_OK) {
    ESP_LOGE(TAG, "SPI write failed: %s", esp_err_to_name(result));
  }

  vTaskDelay(pdMS_TO_TICKS(1)); // Might not be neccessary

  return result;
}