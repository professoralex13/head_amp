#ifndef CODEC_SPI_H
#define CODEC_SPI_H

#include "driver/spi_master.h"
#include "hal/spi_types.h"
#include "stdint.h"

#define SPI_FREQUENCY 1000000

typedef spi_device_handle_t spi_codec_device;

esp_err_t spi_bus_init(spi_host_device_t host_id, uint8_t clock_pin,
                       uint8_t data_pin);
esp_err_t spi_device_init(uint8_t chip_select_pin, spi_codec_device *handle);
esp_err_t write_register(uint8_t address, uint16_t value,
                         spi_codec_device device);

#endif