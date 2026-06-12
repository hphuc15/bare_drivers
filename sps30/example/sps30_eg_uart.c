/**
 * @file sps30_eg_uart.c
 * @brief SPS30 UART example - ESP32 / ESP-IDF
 *
 * Wiring (SPS30 pin 1..5):
 *  Pin 1 VDD   → 5V
 *  Pin 2 RX    → GPIO17
 *  Pin 3 TX    → GPIO16
 *  Pin 4 SEL   → Float (UART mode)        
 *  Pin 5 GND   → GND
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sps30.h"

#define EG_UART_PORT    UART_NUM_1
#define EG_UART_TX_IO   GPIO_NUM_17
#define EG_UART_RX_IO   GPIO_NUM_16
#define EG_UART_BAUD    SPS30_CFG_UART_BAUDRATE
#define EG_UART_BUFSZ   1024
#define EG_POLL_MS      2000

static const char *TAG = "[SPS30_EG]";

/* HAL */

/** Block until `n` bytes arrive or `ms` elapses; returns 0 on success. */
static int _sps30_hal_read(uint8_t *b, size_t n, uint32_t ms) {
    return uart_read_bytes(EG_UART_PORT, b, n, pdMS_TO_TICKS(ms)) == (int)n ? 0 : -1;
}

/** Transmit `n` bytes; returns 0 on success. */
static int _sps30_hal_write(const uint8_t *b, size_t n) {
    return uart_write_bytes(EG_UART_PORT, (const char *)b, n) == (int)n ? 0 : -1;
}

/** Discard all bytes currently in the UART RX FIFO. */
static void _sps30_hal_flush(void) {
    uart_flush_input(EG_UART_PORT);
}

/** FreeRTOS tick-based delay (minimum 1 tick). */
static void _sps30_hal_delay(uint32_t ms) {
    vTaskDelay((ms + portTICK_PERIOD_MS - 1) / portTICK_PERIOD_MS);
}

/* UART peripheral */

/** Install UART driver and configure pins; returns 0 on success, -1 on error. */
static int _sps30_uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = SPS30_CFG_UART_BAUDRATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    if (uart_driver_install(EG_UART_PORT, EG_UART_BUFSZ, 0, 0, NULL, 0) != ESP_OK) return -1;
    if (uart_param_config(EG_UART_PORT, &cfg)                         != ESP_OK) return -1;
    if (uart_set_pin(EG_UART_PORT, EG_UART_TX_IO, EG_UART_RX_IO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE)       != ESP_OK) return -1;
    return 0;
}

/* Helpers */

/** Map PM2.5 [µg/m³] to a US AQI qualitative label. */
static const char *_sps30_aqi_label(float pm25)
{
    if (pm25 <  12.0f) return "Good";
    if (pm25 <  35.4f) return "Moderate";
    if (pm25 <  55.4f) return "Unhealthy (Sensitive)";
    if (pm25 < 150.4f) return "Unhealthy";
    return "Very Unhealthy / Hazardous";
}

/** Log all mass/number concentration fields and AQI category. */
static void _sps30_log(const SPS30_Data *m)
{
    ESP_LOGI(TAG, "--------------------------------------------");
    ESP_LOGI(TAG, "  Mass [ug/m3]");
    ESP_LOGI(TAG, "    PM1.0 = %7.2f    PM2.5 = %7.2f", m->pm1_0, m->pm2_5);
    ESP_LOGI(TAG, "    PM4.0 = %7.2f    PM10  = %7.2f", m->pm4_0, m->pm10);
    ESP_LOGI(TAG, "  Number [#/cm3]");
    ESP_LOGI(TAG, "    NC0.5 = %7.2f    NC1.0 = %7.2f", m->nc0_5, m->nc1_0);
    ESP_LOGI(TAG, "    NC2.5 = %7.2f    NC4.0 = %7.2f", m->nc2_5, m->nc4_0);
    ESP_LOGI(TAG, "    NC10  = %7.2f", m->nc10);
    ESP_LOGI(TAG, "  Typical size : %.3f um", m->typical_size);
    ESP_LOGI(TAG, "  AQI (PM2.5)  : %s", _sps30_aqi_label(m->pm2_5));
    ESP_LOGI(TAG, "--------------------------------------------");
}

/** Read and log device status register; warn on any active fault bit. */
static void _sps30_check_status(SPS30_Dev *dev)
{
    uint32_t st = 0;
    if (SPS30_ReadDeviceStatus(dev, &st, false) != SPS30_OK) return;
    if (st & SPS30_STATUS_SPEED_BIT) ESP_LOGW(TAG, "Fan speed out of range");
    if (st & SPS30_STATUS_LASER_BIT) ESP_LOGE(TAG, "Laser current failure");
    if (st & SPS30_STATUS_FAN_BIT)   ESP_LOGE(TAG, "Fan blocked / failed");
}

/* Task */

/** Main demo task: init, reset, start measurement, poll loop. */
static void _sps30_task(void *arg)
{
    /* 1. UART peripheral init */
    if (_sps30_uart_init() != 0) {
        ESP_LOGE(TAG, "UART init failed"); vTaskDelete(NULL); return;
    }
    ESP_LOGI(TAG, "UART ready (port=%d TX=%d RX=%d baud=%d)",
             EG_UART_PORT, EG_UART_TX_IO, EG_UART_RX_IO, SPS30_CFG_UART_BAUDRATE);

    /* 2. Device handle init */
    SPS30_Dev dev = {
        .protocol = SPS30_PROTOCOL_UART,
        .fmt      = SPS30_FORMAT_FLOAT,
        .state    = SPS30_STATE_IDLE,
        .hal.uart = {
            .read     = _sps30_hal_read,
            .write    = _sps30_hal_write,
            .delay_ms = _sps30_hal_delay,
            .flush_rx = _sps30_hal_flush,
        },
    };
    if (SPS30_Init(&dev) != SPS30_OK) {
        ESP_LOGE(TAG, "SPS30_Init failed"); vTaskDelete(NULL); return;
    }

    /* 3. Optional: log device info and firmware version */
    SPS30_DeviceInfo info = {0};
    if (SPS30_ReadDeviceInfo(&dev, &info) == SPS30_OK)
        ESP_LOGI(TAG, "Type: %s  S/N: %s", info.product_type, info.serial_number);

    SPS30_Version ver = {0};
    if (SPS30_ReadVersion(&dev, &ver) == SPS30_OK)
        ESP_LOGI(TAG, "FW v%u.%u  HW rev %u", ver.fw_major, ver.fw_minor, ver.hw_rev);

    /* 4. Reset to known idle (handles stale state after ESP32 soft-reset) */
    if (SPS30_Reset(&dev) != SPS30_OK) {
        ESP_LOGE(TAG, "SPS30_Reset failed"); vTaskDelete(NULL); return;
    }

    /* 5. Start measurement */
    if (SPS30_StartMeasurement(&dev) != SPS30_OK) {
        ESP_LOGE(TAG, "SPS30_StartMeasurement failed"); vTaskDelete(NULL); return;
    }
    ESP_LOGI(TAG, "Measuring (poll every %d ms)...", EG_POLL_MS);

    /* 6. Poll loop */
    for (int cycle = 1; ; cycle++) {
        SPS30_Data meas;
        switch (SPS30_ReadMeasurement(&dev, &meas)) {
            case SPS30_OK:          _sps30_log(&meas);                          break;
            case SPS30_ERR_NO_DATA: ESP_LOGD(TAG, "Not ready yet");             break;
            case SPS30_ERR_TIMEOUT: ESP_LOGW(TAG, "Timeout - check wiring");    break;
            case SPS30_ERR_CRC:     ESP_LOGW(TAG, "CRC error");                 break;
            default:                ESP_LOGE(TAG, "Read error");                 break;
        }

        if (cycle % 30 == 0) _sps30_check_status(&dev);    /* ~every 60 s */
        vTaskDelay(pdMS_TO_TICKS(EG_POLL_MS));
    }
}

/* Entry point */

void app_main(void)
{
    xTaskCreate(_sps30_task, "sps30", 4096, NULL, 5, NULL);
}