#ifndef SPS30_H
#define SPS30_H

#include "sps30_types.h"
#include "sps30_defs.h"
#include <stdint.h>
#include <stddef.h>

/* =========================================================================
 * HAL callbacks
 * =========================================================================
 *
 * I2C:
 *   read / write use the 7-bit slave address (SPS30_I2C_ADDR = 0x69).
 *   Return 0 on success, negative on error.
 *
 * UART:
 *   No address argument - the bus is point-to-point.
 *   read()  must block until `len` bytes are received OR `timeout_ms`
 *           elapses.  Return 0 on success, negative on timeout / error.
 *   write() must transmit all `len` bytes. Return 0 on success.
 *   delay_ms: blocking millisecond delay.
 *
 * I2C Wake-Up limitation:
 *   The SPS30 datasheet requires a bare Start+Stop pulse (no data bytes)
 *   to activate the I2C interface before the actual Wake-Up command.
 *   The current HAL does not expose a pulse() primitive; instead two
 *   consecutive i2c_set_pointer(WAKEUP) calls are issued as an
 *   approximation.  If your platform's I2C driver rejects zero-length
 *   writes, add a `pulse` callback to SPS30_Hal_I2C.
 * ========================================================================= */

typedef struct {
    int  (*read)     (uint8_t addr, uint8_t *buf, size_t len);
    int  (*write)    (uint8_t addr, const uint8_t *buf, size_t len);
    void (*delay_ms) (uint32_t ms);
} SPS30_Hal_I2C;

typedef struct {
    /**
     * Read exactly `len` bytes into `buf`.
     * Block until data arrives OR `timeout_ms` elapses.
     * @return 0 on success, negative on timeout or I/O error.
     */
    int  (*read)     (uint8_t *buf, size_t len, uint32_t timeout_ms);
    int  (*write)    (const uint8_t *buf, size_t len);
    void (*delay_ms) (uint32_t ms);
    /**
     * Discard all bytes currently sitting in the RX buffer.
     * Called automatically before every command is sent to prevent
     * stale response bytes from a previous transaction being parsed
     * as the current response.  Must not block.  May be NULL.
     */
    void (*flush_rx) (void);
} SPS30_Hal_UART;

/* =========================================================================
 * Device handle
 * ========================================================================= */

typedef struct {
    SPS30_Protocol      protocol;
    SPS30_OutputFormat  fmt;
    SPS30_State         state;
    union {
        SPS30_Hal_I2C   i2c;
        SPS30_Hal_UART  uart;
    } hal;
} SPS30_Dev;

/* =========================================================================
 * Version / device info structures
 * ========================================================================= */

typedef struct {
    uint8_t fw_major;       /**< Firmware major version          */
    uint8_t fw_minor;       /**< Firmware minor version          */
    uint8_t hw_rev;         /**< Hardware revision               */
    uint8_t shdlc_major;    /**< SHDLC protocol major version    */
    uint8_t shdlc_minor;    /**< SHDLC protocol minor version    */
} SPS30_Version;

typedef struct {
    char product_type[32];  /**< Product Type string (null-terminated) */
    char serial_number[32]; /**< Serial Number string (null-terminated) */
} SPS30_DeviceInfo;

/* =========================================================================
 * Device status register bit masks  (datasheet §4.4)
 * ========================================================================= */

#define SPS30_STATUS_SPEED_BIT   (1u << 21)  /**< Warning: fan speed out of range */
#define SPS30_STATUS_LASER_BIT   (1u <<  5)  /**< Error:   laser current failure  */
#define SPS30_STATUS_FAN_BIT     (1u <<  4)  /**< Error:   fan blocked / failed   */

/* =========================================================================
 * Public API
 * =========================================================================
 *
 * Typical usage sequence:
 *
 *   SPS30_Dev dev = {
 *       .protocol = SPS30_PROTOCOL_UART,
 *       .fmt      = SPS30_FORMAT_FLOAT,
 *       .state    = SPS30_STATE_IDLE,    // must be set by caller
 *       .hal.uart = { my_uart_read, my_uart_write, my_delay },
 *   };
 *   SPS30_Init(&dev);
 *   SPS30_StartMeasurement(&dev);
 *   vTaskDelay(pdMS_TO_TICKS(1200));     // wait for first measurement
 *   SPS30_Data d;
 *   SPS30_ReadMeasurement(&dev, &d);
 *   SPS30_StopMeasurement(&dev);
 *
 * ========================================================================= */

/**
 * @brief Validate the device handle and HAL callbacks.
 *
 * Does NOT communicate with the sensor.  Sets dev->state to
 * SPS30_STATE_IDLE on success so the handle is ready for use even if the
 * caller did not zero-initialise the struct.
 *
 * @param dev  Pointer to caller-allocated, partially initialised handle.
 *             Must have .protocol, .fmt, and .hal populated.
 * @return SPS30_OK or SPS30_ERR_INVALID_ARGS.
 */
SPS30_Status SPS30_Init(SPS30_Dev *dev);

/**
 * @brief Start continuous measurement (Idle → Measurement mode).
 *
 * New readings are available every ~1 s after a warm-up period of
 * 8-30 s depending on particle concentration (datasheet §1.1).
 *
 * @return SPS30_OK, SPS30_ERR_INVALID_STATE, or I/O error.
 */
SPS30_Status SPS30_StartMeasurement(SPS30_Dev *dev);

/**
 * @brief Stop continuous measurement (Measurement → Idle mode).
 *
 * @return SPS30_OK, SPS30_ERR_INVALID_STATE, or I/O error.
 */
SPS30_Status SPS30_StopMeasurement(SPS30_Dev *dev);

/**
 * @brief Read the latest measured values.
 *
 * Returns SPS30_ERR_NO_DATA if the sensor has not produced a new sample
 * yet (UART: empty response frame; I2C: Data-Ready Flag not set).
 * The caller should retry after ~1 s.
 *
 * @return SPS30_OK, SPS30_ERR_NO_DATA, SPS30_ERR_INVALID_STATE, or error.
 */
SPS30_Status SPS30_ReadMeasurement(SPS30_Dev *dev, SPS30_Data *data);

/**
 * @brief Enter Sleep mode (Idle → Sleep, minimum power draw).
 *
 * The communication interface is disabled in Sleep mode; use
 * SPS30_WakeUp() to re-enable it.
 *
 * @return SPS30_OK, SPS30_ERR_INVALID_STATE, or I/O error.
 */
SPS30_Status SPS30_Sleep(SPS30_Dev *dev);

/**
 * @brief Wake the sensor from Sleep mode (Sleep → Idle).
 *
 * For UART: sends the required 0xFF pulse followed by the Wake-Up command.
 * For I2C: issues two consecutive Set-Pointer transactions (see HAL note
 * above regarding the bare Start+Stop limitation).
 *
 * @return SPS30_OK, SPS30_ERR_INVALID_STATE, or I/O error.
 */
SPS30_Status SPS30_WakeUp(SPS30_Dev *dev);

/**
 * @brief Trigger a manual fan-cleaning cycle (~10 s at maximum fan speed).
 *
 * Must be called while in Measurement mode.  Measurement values are not
 * updated during the cleaning cycle.
 *
 * @return SPS30_OK, SPS30_ERR_INVALID_STATE, or I/O error.
 */
SPS30_Status SPS30_StartFanCleaning(SPS30_Dev *dev);

/**
 * @brief Read the automatic fan-cleaning interval (seconds).
 *
 * @param interval  Output: interval in seconds; 0 = disabled.
 * @return SPS30_OK or I/O error.
 */
SPS30_Status SPS30_ReadAutoCleaningInterval(SPS30_Dev *dev, uint32_t *interval);

/**
 * @brief Write the automatic fan-cleaning interval.
 *
 * Stored in non-volatile memory.  Default: 604800 s (1 week).
 * Set to 0 to disable.
 *
 * @param interval  Desired interval in seconds.
 * @return SPS30_OK or I/O error.
 */
SPS30_Status SPS30_WriteAutoCleaningInterval(SPS30_Dev *dev, uint32_t interval);

/**
 * @brief Read firmware, hardware and protocol version fields.
 *
 * @param version  Output version structure.
 * @return SPS30_OK or I/O error.
 */
SPS30_Status SPS30_ReadVersion(SPS30_Dev *dev, SPS30_Version *version);

/**
 * @brief Read product type and serial number strings.
 *
 * Both strings are null-terminated and at most 31 characters long.
 *
 * @param info  Output structure.
 * @return SPS30_OK or I/O error.
 */
SPS30_Status SPS30_ReadDeviceInfo(SPS30_Dev *dev, SPS30_DeviceInfo *info);

/**
 * @brief Read the 32-bit Device Status Register.
 *
 * Check individual fields with the SPS30_STATUS_* masks.
 *
 * @param status  Output: raw 32-bit value.
 * @param clear   If true, all bits are cleared after reading.
 * @return SPS30_OK or I/O error.
 */
SPS30_Status SPS30_ReadDeviceStatus(SPS30_Dev *dev, uint32_t *status, bool clear);

/**
 * @brief Perform a soft reset (equivalent to a power-on reset).
 *
 * If the sensor is currently in Sleep mode a Wake-Up sequence is issued
 * automatically before the reset command.
 *
 * @return SPS30_OK or I/O error.
 */
SPS30_Status SPS30_Reset(SPS30_Dev *dev);

#endif /* SPS30_H */