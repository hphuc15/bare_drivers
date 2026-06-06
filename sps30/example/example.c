/**
 * @file main.c
 * @brief SPS30 library usage example - ESP32 / ESP-IDF / FreeRTOS
 *
 * Wiring (SPS30 JST ZHR-5 1.5 mm, pin 1..5):
 *   Pin 1  VDD → 5 V
 *   Pin 2  RX  → GPIO 17  (ESP32 TX → SPS30 RX)
 *   Pin 3  TX  → GPIO 16  (SPS30 TX → ESP32 RX)
 *   Pin 4  SEL → GND      (selects UART mode; float or VDD = I2C)
 *   Pin 5  GND → GND
 *
 * UART config: 115200 8N1, no flow control.
 */

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "sps30.h"

/* board config */
#define EXAMPLE_UART_PORT   UART_NUM_1
#define EXAMPLE_GPIO_TX     GPIO_NUM_17   /* ESP32 TX → SPS30 RX */
#define EXAMPLE_GPIO_RX     GPIO_NUM_16   /* ESP32 RX ← SPS30 TX */
#define EXAMPLE_UART_BAUD   115200
#define EXAMPLE_UART_BUFSZ  1024

/* poll interval - SPS30 produces a new sample every ~1 s */
#define EXAMPLE_POLL_MS     2000

static const char *TAG = "[SPS30][EXAMPLE]";

/* HAL implementation */

/**
 * HAL read: block until `len` bytes arrive or `timeout_ms` elapses.
 * Returns 0 on success, -1 on timeout / error.
 */
static int hal_uart_read(uint8_t *buf, size_t len, uint32_t timeout_ms)
{
    int n = uart_read_bytes(EXAMPLE_UART_PORT, buf, len, pdMS_TO_TICKS(timeout_ms));
    return (n == (int)len) ? 0 : -1;
}

/** HAL write: transmit all bytes. Returns 0 on success, -1 on error. */
static int hal_uart_write(const uint8_t *buf, size_t len)
{
    int n = uart_write_bytes(EXAMPLE_UART_PORT, (const char *)buf, len);
    return (n == (int)len) ? 0 : -1;
}

/** HAL flush: discard all bytes currently in the UART RX FIFO. */
static void hal_uart_flush_rx(void)
{
    uart_flush_input(EXAMPLE_UART_PORT);
}

/** HAL delay: FreeRTOS tick-based delay (minimum 1 tick). */
static void hal_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms > 0u ? ms : 1u));
}

/* UART peripheral init */

static esp_err_t uart_init(void)
{
    uart_config_t cfg = {
        .baud_rate  = EXAMPLE_UART_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    esp_err_t ret = ESP_OK;
    ret |= uart_driver_install(EXAMPLE_UART_PORT, EXAMPLE_UART_BUFSZ, 0, 0, NULL, 0);
    ret |= uart_param_config(EXAMPLE_UART_PORT, &cfg);
    ret |= uart_set_pin(EXAMPLE_UART_PORT, EXAMPLE_GPIO_TX, EXAMPLE_GPIO_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    return ret;
}

/* helpers */

/** Map PM2.5 [µg/m³] to a US AQI qualitative category. */
static const char *pm25_category(float pm25)
{
    if      (pm25 <  12.0f) return "Good";
    else if (pm25 <  35.4f) return "Moderate";
    else if (pm25 <  55.4f) return "Unhealthy for Sensitive Groups";
    else if (pm25 < 150.4f) return "Unhealthy";
    else                    return "Very Unhealthy / Hazardous";
}

/** Log all fields from a SPS30_Data measurement. */
static void log_measurement(const SPS30_Data *m)
{
    ESP_LOGI(TAG, "────────────────────────────────");
    ESP_LOGI(TAG, "Mass concentration [µg/m³]:");
    ESP_LOGI(TAG, "  PM 1.0  = %.2f", m->pm1_0);
    ESP_LOGI(TAG, "  PM 2.5  = %.2f", m->pm2_5);
    ESP_LOGI(TAG, "  PM 4.0  = %.2f", m->pm4_0);
    ESP_LOGI(TAG, "  PM 10   = %.2f", m->pm10);
    ESP_LOGI(TAG, "Number concentration [#/cm³]:");
    ESP_LOGI(TAG, "  NC 0.5  = %.2f", m->nc0_5);
    ESP_LOGI(TAG, "  NC 1.0  = %.2f", m->nc1_0);
    ESP_LOGI(TAG, "  NC 2.5  = %.2f", m->nc2_5);
    ESP_LOGI(TAG, "  NC 4.0  = %.2f", m->nc4_0);
    ESP_LOGI(TAG, "  NC 10   = %.2f", m->nc10);
    ESP_LOGI(TAG, "Typical particle size = %.3f µm", m->typical_size);
    ESP_LOGI(TAG, "Air quality (PM2.5)   : %s", pm25_category(m->pm2_5));
}

/* demo task */

static void sps30_example_task(void *arg)
{
    /* 1. Init UART peripheral */
    if (uart_init() != ESP_OK) {
        ESP_LOGE(TAG, "UART init failed - aborting");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "UART ready (port=%d TX=%d RX=%d baud=%d)", EXAMPLE_UART_PORT, EXAMPLE_GPIO_TX, EXAMPLE_GPIO_RX, EXAMPLE_UART_BAUD);

    /* 2. Build device handle */
    SPS30_Dev dev = {
        .protocol = SPS30_PROTOCOL_UART,
        .fmt      = SPS30_FORMAT_FLOAT,
        .state    = SPS30_STATE_IDLE,
        .hal.uart = {
            .read     = hal_uart_read,
            .write    = hal_uart_write,
            .delay_ms = hal_delay_ms,
            .flush_rx = hal_uart_flush_rx,
        },
    };

    SPS30_Status s = SPS30_Init(&dev);
    if (s != SPS30_OK) {
        ESP_LOGE(TAG, "SPS30_Init failed (%d) - aborting", s);
        vTaskDelete(NULL);
        return;
    }

    /* Give the sensor time to finish power-on (datasheet: ≤8 ms after VDD) */
    vTaskDelay(pdMS_TO_TICKS(100));

    /* 3. Optional: read device information */
    SPS30_DeviceInfo info = {0};
    if (SPS30_ReadDeviceInfo(&dev, &info) == SPS30_OK) {
        ESP_LOGI(TAG, "Product type  : %s", info.product_type);
        ESP_LOGI(TAG, "Serial number : %s", info.serial_number);
    } else {
        ESP_LOGW(TAG, "Could not read device info (non-fatal)");
    }

    /* 4. Optional: read firmware version */
    SPS30_Version ver = {0};
    if (SPS30_ReadVersion(&dev, &ver) == SPS30_OK) {
        ESP_LOGI(TAG, "Firmware v%u.%u  HW rev %u  SHDLC v%u.%u",
                 ver.fw_major, ver.fw_minor, ver.hw_rev,
                 ver.shdlc_major, ver.shdlc_minor);
    }

    /* 5. Optional: check / set auto-cleaning interval */
    uint32_t clean_interval = 0u;
    if (SPS30_ReadAutoCleaningInterval(&dev, &clean_interval) == SPS30_OK) {
        ESP_LOGI(TAG, "Auto-cleaning interval: %lu s (default 604800 = 1 week)", (unsigned long)clean_interval);
    }
    /* Uncomment to change interval (stored in NVM):
     * SPS30_WriteAutoCleaningInterval(&dev, 604800u);
     */

    /* 6. Reset to ensure a known idle state      *
     * The sensor retains its operational state across ESP32 soft-resets
     * (IDF app restarts without power cycling).  If the previous firmware
     * run started measurement and then crashed or was flashed over, the
     * sensor is still in Measurement mode on the next boot.
     * SPS30_StartMeasurement() returns SPS30_ERR_INVALID_STATE (-6) in
     * that case because the sensor's own exec_err byte is 0x43.
     *
     * A soft reset brings the sensor unconditionally back to Idle mode.
     * We ignore SPS30_ERR_INVALID_STATE here because SPS30_Reset() handles
     * the Sleep→Idle wake-up sequence internally; any other error is fatal.
     */
    ESP_LOGI(TAG, "Resetting sensor to ensure idle state...");
    s = SPS30_Reset(&dev);
    if (s != SPS30_OK) {
        ESP_LOGE(TAG, "SPS30_Reset failed (%d) - aborting", s);
        vTaskDelete(NULL);
        return;
    }
    /* Datasheet: sensor needs up to 100 ms after reset before accepting commands.
     * Wait 200 ms then flush the UART RX buffer in case the sensor sent any
     * spurious bytes during its internal reboot sequence. */
    vTaskDelay(pdMS_TO_TICKS(200));
    uart_flush_input(EXAMPLE_UART_PORT);

    /* 7. Start measurement */
    ESP_LOGI(TAG, "Starting measurement...");
    s = SPS30_StartMeasurement(&dev);
    if (s != SPS30_OK) {
        ESP_LOGE(TAG, "SPS30_StartMeasurement failed (%d) - aborting", s);
        vTaskDelete(NULL);
        return;
    }

    /*
     * The SPS30 needs ~1 s to produce its first measurement.
     * Allow extra margin; the first few readings may be unstable for up to
     * 30 s in very clean air (datasheet §1.1).
     */
    ESP_LOGI(TAG, "Waiting for first measurement...");
    vTaskDelay(pdMS_TO_TICKS(1200));

    /* 8. Poll loop */
    ESP_LOGI(TAG, "Entering measurement loop (interval = %d ms)", EXAMPLE_POLL_MS);

    while (1) {
        SPS30_Data meas;
        s = SPS30_ReadMeasurement(&dev, &meas);

        switch (s) {
            case SPS30_OK:
                log_measurement(&meas);
                break;

            case SPS30_ERR_NO_DATA:
                /* Sensor not ready yet - retry on next iteration */
                ESP_LOGD(TAG, "No new data yet, retrying...");
                break;

            case SPS30_ERR_TIMEOUT:
                ESP_LOGW(TAG, "Read timeout - check wiring");
                break;

            case SPS30_ERR_CRC:
                ESP_LOGW(TAG, "CRC / checksum error - frame corrupted");
                break;

            default:
                ESP_LOGE(TAG, "SPS30_ReadMeasurement error (%d)", s);
                break;
        }

        /* Optional: periodic device status check */
        static int cycle = 0;
        if (++cycle % 30 == 0) {     /* every ~60 s */
            uint32_t status = 0u;
            if (SPS30_ReadDeviceStatus(&dev, &status, false) == SPS30_OK) {
                if (status & SPS30_STATUS_SPEED_BIT) {
                    ESP_LOGW(TAG, "Status: fan speed out of range");
                }
                if (status & SPS30_STATUS_LASER_BIT) {
                    ESP_LOGE(TAG, "Status: laser current failure");
                }
                if (status & SPS30_STATUS_FAN_BIT) {
                    ESP_LOGE(TAG, "Status: fan blocked / failed");
                }
                if (status == 0u) {
                    ESP_LOGD(TAG, "Device status: OK");
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(EXAMPLE_POLL_MS));
    }

    /* Unreachable in normal operation - shown for completeness */
    SPS30_StopMeasurement(&dev);
    vTaskDelete(NULL);
}

/* app_main */

void app_main(void)
{
    xTaskCreate(sps30_example_task, "sps30_example", 4096, NULL, 5, NULL);
}