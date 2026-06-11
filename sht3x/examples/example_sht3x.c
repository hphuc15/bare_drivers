/**
 * @file    example_sht3x.c
 * @brief   Using driver SHT3x in ESP32 (ESP-IDF).
 */

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "sht3x.h"

/* =========================================================================
 * I2C Configuration
 * ========================================================================= */

#define I2C_PORT        I2C_NUM_0
#define I2C_SDA_PIN     21
#define I2C_SCL_PIN     22
#define I2C_FREQ_HZ     400000   /* 400 kHz – Fast Mode */

/* =========================================================================
 * HAL callbacks – bridge from driver to ESP-IDF I2C API
 * ========================================================================= */

static int hal_i2c_write(uint8_t addr, const uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return (ret == ESP_OK) ? 0 : -1;
}

static int hal_i2c_read(uint8_t addr, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, &data[len - 1], I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return (ret == ESP_OK) ? 0 : -1;
}

static void hal_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

/* =========================================================================
 * Initialize I2C bus
 * ========================================================================= */

static void i2c_bus_init(void)
{
    i2c_config_t conf = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = I2C_SDA_PIN,
        .scl_io_num       = I2C_SCL_PIN,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ_HZ,
    };
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
}

/* =========================================================================
 * Eg.1: Single-shot mode (no clock stretch)
 * ========================================================================= */

static void example_single_shot(void)
{
    printf("\n=== Single-shot mode ===\n");

    SHT3x_Dev dev = {
        .i2c_addr      = SHT3X_I2C_ADDR_DEFAULT,  /* ADDR pin nối GND */
        .mode          = SHT3X_MODE_SINGLE_SHOT,
        .repeatability = SHT3X_REPEAT_HIGH,
        .clock_stretch = false,
        .i2c_write     = hal_i2c_write,
        .i2c_read      = hal_i2c_read,
        .delay_ms      = hal_delay_ms,
    };

    SHT3x_Status st = SHT3x_Init(&dev);
    if (st != SHT3X_OK) {
        printf("Init thất bại: %d\n", st);
        return;
    }

    for (int i = 0; i < 5; i++) {
        SHT3x_Data data;
        st = SHT3x_Read(&dev, &data);
        if (st == SHT3X_OK) {
            printf("[%d] Nhiệt độ: %.2f °C  |  Độ ẩm: %.2f %%RH\n",
                   i, data.temperature_c, data.humidity_rh);
        } else {
            printf("[%d] Đọc lỗi: %d\n", i, st);
        }
        hal_delay_ms(1000);
    }

    SHT3x_Deinit(&dev);
}

/* =========================================================================
 * Eg.2: Single-shot mode using clock stretch
 * ========================================================================= */

static void example_single_shot_clock_stretch(void)
{
    printf("\n=== Single-shot + Clock Stretch ===\n");

    SHT3x_Dev dev = {
        .i2c_addr      = SHT3X_I2C_ADDR_DEFAULT,
        .mode          = SHT3X_MODE_SINGLE_SHOT,
        .repeatability = SHT3X_REPEAT_MEDIUM,
        .clock_stretch = true,   /* Sensor tự kéo SCL, không cần delay_ms */
        .i2c_write     = hal_i2c_write,
        .i2c_read      = hal_i2c_read,
        .delay_ms      = hal_delay_ms,
    };

    if (SHT3x_Init(&dev) != SHT3X_OK) {
        printf("Init thất bại\n");
        return;
    }

    SHT3x_Data data;
    if (SHT3x_Read(&dev, &data) == SHT3X_OK) {
        printf("Nhiệt độ: %.2f °C  |  Độ ẩm: %.2f %%RH\n",
               data.temperature_c, data.humidity_rh);
    }

    SHT3x_Deinit(&dev);
}

/* =========================================================================
 * Ví dụ 3: Periodic mode
 * Sensor tự đo liên tục, firmware chỉ việc gọi FETCH_DATA để lấy kết quả
 * Phù hợp khi cần logging liên tục hoặc tốc độ cao
 * ========================================================================= */

static void example_periodic(void)
{
    printf("\n=== Periodic mode (2 mps, high repeatability) ===\n");

    SHT3x_Dev dev = {
        .i2c_addr      = SHT3X_I2C_ADDR_DEFAULT,
        .mode          = SHT3X_MODE_PERIODIC,
        .repeatability = SHT3X_REPEAT_HIGH,
        .mps           = SHT3X_MPS_2,   /* 2 measurements/second */
        .i2c_write     = hal_i2c_write,
        .i2c_read      = hal_i2c_read,
        .delay_ms      = hal_delay_ms,
    };

    if (SHT3x_Init(&dev) != SHT3X_OK) {
        printf("Init thất bại\n");
        return;
    }

    /*
     * Sensor đang tự đo nền. Chờ ít nhất 1 chu kỳ (500ms với 2 mps)
     * trước lần FETCH đầu tiên để tránh đọc buffer rỗng.
     */
    hal_delay_ms(600);

    for (int i = 0; i < 10; i++) {
        SHT3x_Data data;
        SHT3x_Status st = SHT3x_Read(&dev, &data);
        if (st == SHT3X_OK) {
            printf("[%2d] %.2f °C  |  %.2f %%RH\n",
                   i, data.temperature_c, data.humidity_rh);
        } else {
            printf("[%2d] Lỗi: %d\n", i, st);
        }
        hal_delay_ms(500);   /* Đồng bộ với chu kỳ 2 mps */
    }

    SHT3x_Deinit(&dev);   /* Gửi BREAK, sensor dừng đo */
}

/* =========================================================================
 * Ví dụ 4: Đọc status register và dùng heater
 * ========================================================================= */

static void example_status_and_heater(void)
{
    printf("\n=== Status register + Heater ===\n");

    SHT3x_Dev dev = {
        .i2c_addr      = SHT3X_I2C_ADDR_DEFAULT,
        .mode          = SHT3X_MODE_SINGLE_SHOT,
        .repeatability = SHT3X_REPEAT_HIGH,
        .clock_stretch = false,
        .i2c_write     = hal_i2c_write,
        .i2c_read      = hal_i2c_read,
        .delay_ms      = hal_delay_ms,
    };

    if (SHT3x_Init(&dev) != SHT3X_OK) {
        return;
    }

    /* Đọc status */
    uint16_t status;
    if (SHT3x_ReadStatus(&dev, &status) == SHT3X_OK) {
        printf("Status: 0x%04X\n", status);
        printf("  Heater đang bật : %s\n", (status & SHT3X_SREG_HEATER_ON)      ? "CÓ" : "KHÔNG");
        printf("  Alert đang chờ  : %s\n", (status & SHT3X_SREG_ALERT_PENDING)   ? "CÓ" : "KHÔNG");
        printf("  Reset detected  : %s\n", (status & SHT3X_SREG_RESET_DETECTED)  ? "CÓ" : "KHÔNG");
        printf("  Lệnh thất bại   : %s\n", (status & SHT3X_SREG_CMD_FAILED)      ? "CÓ" : "KHÔNG");
    }

    /* Heater chỉ dùng để test/chẩn đoán, không dùng khi đo thật */
    printf("\nBật heater 3 giây để test...\n");
    SHT3x_HeaterEnable(&dev, true);
    hal_delay_ms(3000);
    SHT3x_HeaterEnable(&dev, false);
    printf("Tắt heater.\n");

    /* Xóa tất cả flag trong status register */
    SHT3x_ClearStatus(&dev);

    SHT3x_Deinit(&dev);
}

/* =========================================================================
 * app_main
 * ========================================================================= */

void app_main(void)
{
    i2c_bus_init();

    example_single_shot();
    example_single_shot_clock_stretch();
    example_periodic();
    example_status_and_heater();

    printf("\nTất cả ví dụ hoàn thành.\n");
}