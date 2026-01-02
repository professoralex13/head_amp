idf_component_register(
    SRCS "main.c" "codec/i2s.c" "codec/settings.c" "codec/spi.c" "bluetooth/bluetooth.c"
    INCLUDE_DIRS "."
    REQUIRES bt driver esp_driver_i2s nvs_flash esp_ringbuf esp_driver_dac esp_driver_spi
)
