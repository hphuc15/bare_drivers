/**
 * @file    sht3x.h
 * @brief   Bare-metal driver for Sensirion SHT3x-DIS humidity & temperature sensor.
 * @version 2.0
 *
 * @details Datasheet: SHT3x-DIS, August 2016 – Version 3.
 *
 * Single-shot mode supports two variants (Section 4.3 / 4.4):
 *
 * **Clock stretching enabled** (@ref SHT3X_CLK_STRETCH_ENABLE):
 * Master sends the measurement command, then immediately issues a
 * repeated-START + read header. The sensor ACKs the address and pulls SCL
 * low until the measurement is complete, then releases SCL and sends the
 * 6 data bytes. No explicit delay is needed in firmware.
 * Requires HAL support: `sht3x_hal_i2c_read_stretch()`.
 *
 * **Clock stretching disabled** (@ref SHT3X_CLK_STRETCH_DISABLE):
 * Master sends the measurement command, waits the worst-case measurement
 * time, then issues a read header. If the sensor is still busy it NACKs;
 * the driver retries up to `SHT3X_POLL_RETRIES` times.
 * Works on any I2C master (hardware or bit-bang).
 */

#ifndef SHT3X_H
#define SHT3X_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* =========================================================================
 * @defgroup SHT3X_TYPES Types, Enumerations and Structures
 * @{
 * ========================================================================= */

/**
 * @brief Driver return/status codes.
 */
typedef enum {
    SHT3X_OK           =  0,  /**< Operation completed successfully        */
    SHT3X_ERR_I2C      = -1,  /**< I2C bus communication error             */
    SHT3X_ERR_CRC      = -2,  /**< CRC-8 validation failed on received data */
    SHT3X_ERR_PARAM    = -3,  /**< Invalid or out-of-range parameter        */
    SHT3X_ERR_NOT_INIT = -4,  /**< Driver has not been initialised          */
} SHT3x_Status;

/**
 * @brief Measurement repeatability (precision) level.
 * @details Higher repeatability improves accuracy at the cost of longer
 *          conversion time and higher power consumption.
 */
typedef enum {
    SHT3X_REPEAT_LOW    = 0,  /**< Low repeatability – fastest, lowest accuracy  */
    SHT3X_REPEAT_MEDIUM = 1,  /**< Medium repeatability                          */
    SHT3X_REPEAT_HIGH   = 2,  /**< High repeatability – slowest, highest accuracy */
} SHT3x_Repeatability;

/**
 * @brief Sensor operating mode.
 */
typedef enum {
    SHT3X_MODE_SINGLE_SHOT = 0,  /**< On-demand single measurement per @ref SHT3x_Read call  */
    SHT3X_MODE_PERIODIC    = 1,  /**< Sensor samples autonomously at the configured MPS rate  */
} SHT3x_Mode;

/**
 * @brief Periodic-mode measurement rate (measurements per second).
 * @details Only relevant when @ref SHT3x_Dev::mode is @ref SHT3X_MODE_PERIODIC.
 */
typedef enum {
    SHT3X_MPS_05 = 0,   /**< 0.5 measurements per second */
    SHT3X_MPS_1   = 1,  /**< 1 measurement per second    */
    SHT3X_MPS_2   = 2,  /**< 2 measurements per second   */
    SHT3X_MPS_4   = 3,  /**< 4 measurements per second   */
    SHT3X_MPS_10  = 4,  /**< 10 measurements per second  */
} SHT3x_MPS;

/** @} */ /* end group SHT3X_TYPES */

/* =========================================================================
 * @defgroup SHT3X_ADDR I2C Addresses
 * @{
 * ========================================================================= */

#define SHT3X_I2C_ADDR_DEFAULT  0x44u   /**< ADDR pin connected to VSS (default) */
#define SHT3X_I2C_ADDR_ALT      0x45u   /**< ADDR pin connected to VDD           */

/** @} */ /* end group SHT3X_ADDR */

/* =========================================================================
 * @defgroup SHT3X_HAL HAL Function Pointer Types
 * @brief    Platform-specific callback signatures to be provided by the user.
 * @{
 * ========================================================================= */

/**
 * @brief   I2C write callback type.
 * @param   addr  7-bit I2C device address.
 * @param   data  Pointer to byte buffer to transmit.
 * @param   len   Number of bytes to transmit.
 * @return  0 on success, non-zero on bus error.
 */
typedef int (*SHT3x_I2C_Write_Fn)(uint8_t addr, const uint8_t *data, size_t len);

/**
 * @brief   I2C read callback type.
 * @param   addr  7-bit I2C device address.
 * @param   data  Pointer to buffer to receive bytes.
 * @param   len   Number of bytes to receive.
 * @return  0 on success, non-zero on bus error.
 */
typedef int (*SHT3x_I2C_Read_Fn)(uint8_t addr, uint8_t *data, size_t len);

/**
 * @brief   Blocking millisecond delay callback type.
 * @param   ms  Duration to block in milliseconds.
 */
typedef void (*SHT3x_Delay_Ms_Fn)(uint32_t ms);

/** @} */ /* end group SHT3X_HAL */

/* =========================================================================
 * @defgroup SHT3X_STRUCTS Device Handle and Data Structures
 * @{
 * ========================================================================= */

/**
 * @brief   SHT3x device handle.
 * @details Populate all fields before passing to @ref SHT3x_Init.
 *          The @c initialized and @c periodic_running fields are managed
 *          internally by the driver and must not be modified by the application.
 */
typedef struct {
    uint8_t             i2c_addr;       /**< 7-bit I2C device address; must be
                                             @ref SHT3X_I2C_ADDR_DEFAULT or
                                             @ref SHT3X_I2C_ADDR_ALT              */
    SHT3x_Mode          mode;           /**< Measurement mode                     */
    SHT3x_Repeatability repeatability;  /**< Measurement repeatability level      */
    SHT3x_MPS           mps;            /**< Periodic measurement rate (ignored in
                                             single-shot mode)                     */
    bool                clock_stretch;  /**< @c true to use clock-stretching in
                                             single-shot mode; sensor holds SCL low
                                             until data is ready — no software delay
                                             required. Ignored in periodic mode.   */

    bool                initialized;      /**< @c true after a successful @ref SHT3x_Init.
                                               Managed by the driver; do not modify. */
    bool                periodic_running; /**< @c true while periodic mode is active.
                                               Managed by the driver; do not modify. */

    SHT3x_I2C_Write_Fn  i2c_write;  /**< Platform I2C write callback; must not be NULL */
    SHT3x_I2C_Read_Fn   i2c_read;   /**< Platform I2C read callback; must not be NULL  */
    SHT3x_Delay_Ms_Fn   delay_ms;   /**< Platform delay callback; must not be NULL      */
} SHT3x_Dev;

/**
 * @brief Measurement result container populated by @ref SHT3x_Read.
 */
typedef struct {
    float temperature_c;  /**< Temperature in degrees Celsius           */
    float humidity_rh;    /**< Relative humidity in percent (%RH)       */
} SHT3x_Data;

/** @} */ /* end group SHT3X_STRUCTS */

/* =========================================================================
 * @defgroup SHT3X_SREG Status Register Bitmasks
 * @brief    Flags for decoding the value returned by @ref SHT3x_ReadStatus
 *           (datasheet Section 4.11).
 * @{
 * ========================================================================= */

#define SHT3X_SREG_ALERT_PENDING   (1u << 15)  /**< At least one pending alert                 */
#define SHT3X_SREG_HEATER_ON       (1u << 13)  /**< On-chip heater is currently enabled        */
#define SHT3X_SREG_RH_ALERT        (1u << 11)  /**< Humidity tracking alert is active          */
#define SHT3X_SREG_T_ALERT         (1u << 10)  /**< Temperature tracking alert is active       */
#define SHT3X_SREG_RESET_DETECTED  (1u <<  4)  /**< Reset detected since last status clear     */
#define SHT3X_SREG_CMD_FAILED      (1u <<  1)  /**< Last command was not processed by sensor   */
#define SHT3X_SREG_CRC_FAILED      (1u <<  0)  /**< CRC mismatch on last write transfer        */

/** @} */ /* end group SHT3X_SREG */

/* =========================================================================
 * @defgroup SHT3X_API Public API
 * @{
 * ========================================================================= */

/**
 * @brief   Initialise the SHT3x device and start the selected operating mode.
 * @param[in,out] dev  Pointer to a fully populated @ref SHT3x_Dev handle.
 * @return             @ref SHT3X_OK on success, or an error code on failure.
 */
SHT3x_Status SHT3x_Init(SHT3x_Dev *dev);

/**
 * @brief   Trigger or fetch a temperature and humidity measurement.
 * @param[in]  dev  Pointer to an initialised @ref SHT3x_Dev handle.
 * @param[out] out  Pointer to a @ref SHT3x_Data struct to receive the result.
 * @return          @ref SHT3X_OK on success, or an error code on failure.
 */
SHT3x_Status SHT3x_Read(SHT3x_Dev *dev, SHT3x_Data *out);

/**
 * @brief   Stop periodic measurement and reset the device state.
 * @param[in,out] dev  Pointer to the @ref SHT3x_Dev handle to de-initialise.
 * @return             @ref SHT3X_OK on success, or an error code on failure.
 */
SHT3x_Status SHT3x_Deinit(SHT3x_Dev *dev);

/**
 * @brief   Enable or disable the on-chip heater.
 * @param[in,out] dev     Pointer to an initialised @ref SHT3x_Dev handle.
 * @param[in]     enable  @c true to enable the heater, @c false to disable.
 * @return                @ref SHT3X_OK on success, or an error code on failure.
 */
SHT3x_Status SHT3x_HeaterEnable(SHT3x_Dev *dev, bool enable);

/**
 * @brief   Read the 16-bit device status register.
 * @details Use the @c SHT3X_SREG_* bitmasks to decode individual flag bits.
 * @param[in]  dev     Pointer to an initialised @ref SHT3x_Dev handle.
 * @param[out] status  Pointer to receive the raw 16-bit status register value.
 * @return             @ref SHT3X_OK on success, or an error code on failure.
 */
SHT3x_Status SHT3x_ReadStatus(SHT3x_Dev *dev, uint16_t *status);

/**
 * @brief   Clear all alert and error flags in the device status register.
 * @param[in,out] dev  Pointer to an initialised @ref SHT3x_Dev handle.
 * @return             @ref SHT3X_OK on success, or an error code on failure.
 */
SHT3x_Status SHT3x_ClearStatus(SHT3x_Dev *dev);

/** @} */ /* end group SHT3X_API */

#ifdef __cplusplus
}
#endif

#endif /* SHT3X_H */