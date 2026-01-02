#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "driver/i2s_types.h"
#include "esp_err.h"

void bluetooth_init(i2s_chan_handle_t i2s_handle);

#endif