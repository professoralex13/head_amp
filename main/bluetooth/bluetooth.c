#include "bluetooth.h"
#include "driver/gpio.h"
#include "driver/i2s_common.h"
#include "driver/i2s_types.h"
#include "esp_a2dp_api.h"
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include <stdio.h>
#include <string.h>

#define TAG "BT_SPEAKER"

// Configuration
#define PAIRING_BUTTON_GPIO 21 // GPIO for pairing button
#define BT_DEVICE_NAME "CosplayCore"

// Button debounce
#define BUTTON_DEBOUNCE_MS 50
#define BUTTON_LONG_PRESS_MS 3000

// Global state
static bool bt_discoverable = false;
static uint32_t button_press_time = 0;

// Function prototypes
void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param);
void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param);
void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len);
void button_task(void *pvParameters);
void enter_pairing_mode(void);
void exit_pairing_mode(void);

i2s_chan_handle_t i2s_channel;

// A2DP data callback - receives audio data and sends to I2S
void bt_app_a2d_data_cb(const uint8_t *data, uint32_t len) {
  size_t bytes_written = 0;

  // Write audio data to I2S
  i2s_channel_write(i2s_channel, data, len, &bytes_written, portMAX_DELAY);

  if (bytes_written < len) {
    ESP_LOGW(TAG, "I2S underrun: %d/%d bytes written", bytes_written, len);
  }
}

// A2DP event callback
void bt_app_a2d_cb(esp_a2d_cb_event_t event, esp_a2d_cb_param_t *param) {
  switch (event) {
  case ESP_A2D_CONNECTION_STATE_EVT:
    if (param->conn_stat.state == ESP_A2D_CONNECTION_STATE_CONNECTED) {
      ESP_LOGI(TAG, "A2DP connected");
      exit_pairing_mode();
    } else if (param->conn_stat.state ==
               ESP_A2D_CONNECTION_STATE_DISCONNECTED) {
      ESP_LOGI(TAG, "A2DP disconnected");
    }
    break;

  case ESP_A2D_AUDIO_STATE_EVT:
    if (param->audio_stat.state == ESP_A2D_AUDIO_STATE_STARTED) {
      ESP_LOGI(TAG, "Audio playback started");
    } else if (param->audio_stat.state == ESP_A2D_AUDIO_STATE_STOPPED) {
      ESP_LOGI(TAG, "Audio playback stopped");
    }
    break;

  case ESP_A2D_AUDIO_CFG_EVT:
    ESP_LOGI(TAG, "Audio config: sample_rate=%d, channel=%d",
             param->audio_cfg.mcc.cie.sbc_info.samp_freq,
             param->audio_cfg.mcc.cie.sbc_info);
    break;

  default:
    ESP_LOGD(TAG, "A2DP event: %d", event);
    break;
  }
}

// GAP callback for Bluetooth events
void bt_app_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param) {
  switch (event) {
  case ESP_BT_GAP_AUTH_CMPL_EVT:
    if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
      ESP_LOGI(TAG, "Authentication success: %s", param->auth_cmpl.device_name);
    } else {
      ESP_LOGE(TAG, "Authentication failed, status: %d", param->auth_cmpl.stat);
    }
    break;

  case ESP_BT_GAP_MODE_CHG_EVT:
    ESP_LOGI(TAG, "GAP mode change: %d", param->mode_chg.mode);
    break;

  default:
    ESP_LOGD(TAG, "GAP event: %d", event);
    break;
  }
}

// Enter pairing/discoverable mode
void enter_pairing_mode(void) {
  if (!bt_discoverable) {
    ESP_LOGI(TAG, "Entering pairing mode");

    // Set device to discoverable and connectable
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    bt_discoverable = true;

    // Optional: Set a timeout to exit pairing mode
    // This could be implemented with a timer
  }
}

// Exit pairing/discoverable mode
void exit_pairing_mode(void) {
  if (bt_discoverable) {
    ESP_LOGI(TAG, "Exiting pairing mode");

    // Set device to connectable only (not discoverable)
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);
    bt_discoverable = false;
  }
}

// Button monitoring task
void button_task(void *pvParameters) {
  bool last_state = 1;
  bool current_state;
  uint32_t press_duration;

  while (1) {
    current_state = gpio_get_level(PAIRING_BUTTON_GPIO);

    // Detect button press (assuming active low)
    if (current_state == 0 && last_state == 1) {
      vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
      current_state = gpio_get_level(PAIRING_BUTTON_GPIO);

      if (current_state == 0) {
        button_press_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
        ESP_LOGI(TAG, "Button pressed");
      }
    }
    // Detect button release
    else if (current_state == 1 && last_state == 0) {
      vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
      current_state = gpio_get_level(PAIRING_BUTTON_GPIO);

      if (current_state == 1) {
        press_duration =
            (xTaskGetTickCount() * portTICK_PERIOD_MS) - button_press_time;
        ESP_LOGI(TAG, "Button released, duration: %d ms", press_duration);

        // Short press: toggle pairing mode
        if (press_duration < BUTTON_LONG_PRESS_MS) {
          if (bt_discoverable) {
            exit_pairing_mode();
          } else {
            enter_pairing_mode();
          }
        }
        // Long press: could be used for other functions
        else {
          ESP_LOGI(TAG,
                   "Long press detected - could trigger reset/clear pairings");
          // Example: clear all paired devices
          // esp_bt_gap_remove_bond_device(...);
        }
      }
    }

    last_state = current_state;
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Initialize Bluetooth stack
esp_err_t bt_init(void) {
  esp_err_t ret;

  // Initialize NVS for Bluetooth
  ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Release classic BT memory (we're only using A2DP)
  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_BLE));

  // Initialize Bluetooth controller
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ret = esp_bt_controller_init(&bt_cfg);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Bluetooth controller init failed: %s", esp_err_to_name(ret));
    return ret;
  }

  // Enable Bluetooth controller
  ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Bluetooth controller enable failed: %s",
             esp_err_to_name(ret));
    return ret;
  }

  // Initialize Bluedroid stack
  ret = esp_bluedroid_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
    return ret;
  }

  ret = esp_bluedroid_enable();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
    return ret;
  }

  return ESP_OK;
}

// Initialize A2DP sink (receiver)
esp_err_t a2dp_init(void) {
  esp_err_t ret;

  // Register A2DP sink callback
  ret = esp_a2d_register_callback(bt_app_a2d_cb);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "A2DP callback register failed: %s", esp_err_to_name(ret));
    return ret;
  }

  // Register A2DP data callback
  ret = esp_a2d_sink_register_data_callback(bt_app_a2d_data_cb);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "A2DP data callback register failed: %s",
             esp_err_to_name(ret));
    return ret;
  }

  // Initialize A2DP sink
  ret = esp_a2d_sink_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "A2DP sink init failed: %s", esp_err_to_name(ret));
    return ret;
  }

  // Set device name
  esp_bt_dev_set_device_name(BT_DEVICE_NAME);

  // Set discoverable and connectable mode
  esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_NON_DISCOVERABLE);

  // Register GAP callback
  esp_bt_gap_register_callback(bt_app_gap_cb);

  // Set authentication mode (SSP)
  esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
  esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_NONE;
  esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

  ESP_LOGI(TAG, "A2DP sink initialized, device name: %s", BT_DEVICE_NAME);

  return ESP_OK;
}

// Initialize GPIO for button
void gpio_init_button(void) {
  gpio_config_t io_conf = {.pin_bit_mask = (1ULL << PAIRING_BUTTON_GPIO),
                           .mode = GPIO_MODE_INPUT,
                           .pull_up_en = GPIO_PULLUP_ENABLE,
                           .pull_down_en = GPIO_PULLDOWN_DISABLE,
                           .intr_type = GPIO_INTR_DISABLE};
  gpio_config(&io_conf);

  ESP_LOGI(TAG, "Button GPIO %d initialized", PAIRING_BUTTON_GPIO);
}

void bluetooth_init(i2s_chan_handle_t i2s_handle) {
  i2s_channel = i2s_handle;

  ESP_LOGI(TAG, "Starting ESP32 Bluetooth Speaker");

  // Initialize button GPIO
  gpio_init_button();

  // Initialize Bluetooth
  ESP_ERROR_CHECK(bt_init());

  // Initialize A2DP sink
  ESP_ERROR_CHECK(a2dp_init());

  // Create button monitoring task
  xTaskCreate(button_task, "button_task", 2048, NULL, 5, NULL);

  ESP_LOGI(TAG, "Bluetooth speaker ready. Press button to enter pairing mode.");
}