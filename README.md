# CosplayCore

TODO: Put information about it here

## IMPORTANT NOTES:
Main board V1.0 runs ADC data from a CODEC into GPIO12 which also serves as a strapping pin. This signal is pulled high by the CODEC resulting in the ESP32 booting with a flash chip voltage is 1.8V which is incorrect.
To work around this, the following command must be run:
```espefuse --port [PORT] burn-efuse XPD_SDIO_REG 1 XPD_SDIO_TIEH 1 XPD_SDIO_FORCE 1```

This overrides the strapping pin so the memory supply voltage is permenantly set at 3.3V.
Until this comand is run and the fuses are burned on the ESP32, it cannot be flashed with any code.
