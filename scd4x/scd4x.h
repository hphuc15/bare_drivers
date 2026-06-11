/**
 * @file scd4x.h
 * @brief Bare-metal driver header for Sensirion SCD4x CO2 sensor
 *        Supports SCD40, SCD41, SCD43
 *        Based on Datasheet v1.7 - April 2025
 */
#ifndef SCD4X_H
#define SCD4X_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SCD4X_I2C_ADDR 0x62



/**
 * @brief Write len bytes to device; return 0 on success
 * @note  Pass 7-bit address; HAL must handle R/W bit shift if required (e.g. addr << 1 for STM32)
 *
 * @param addr  7-bit I2C device address
 * @param data  Pointer to data buffer to transmit
 * @param len   Number of bytes to write
 * @return      0 on success, non-zero on error
 */
typedef int (*scd4x_i2c_write_fn)(uint8_t addr, const uint8_t *data, size_t len);

/**
 * @brief Read len bytes from device; return 0 on success
 *
 * @param addr  7-bit I2C device address
 * @param data  Pointer to buffer to store received bytes
 * @param len   Number of bytes to read
 * @return      0 on success, non-zero on error
 */
typedef int (*scd4x_i2c_read_fn)(uint8_t addr, uint8_t *data, size_t len);

/**
 * @brief Millisecond delay
 *
 * @param ms  Duration to block in milliseconds
 */
typedef void (*scd4x_delay_ms_fn)(uint32_t ms);

typedef enum {
    SCD4X_OK            =  0,
    SCD4X_ERR_I2C       = -1,   /* I2C bus error */
    SCD4X_ERR_CRC       = -2,   /* CRC mismatch error */
    SCD4X_ERR_FRC       = -3,   /* forced recalibration fail */
    SCD4X_ERR_NOT_READY = -4,   /* data not ready */
    SCD4X_ERR_PARAM     = -5,   /* arguments is out of value */
} SCD4x_Status;

typedef enum {
    SCD4X_MODEL_40 = 0,
    SCD4X_MODEL_41,
    SCD4X_MODEL_43,
} SCD4x_Model;

typedef enum {
    SCD4X_MODE_PERIODIC = 0,
    SCD4X_MODE_PERIODIC_LOW_POWER,
    SCD4X_MODE_SINGLE_SHOT,      /* Only available on SCD41 and SCD43 models */
} SCD4x_Mode;

typedef struct {
    uint16_t co2_ppm;
    float temperature_c;
    float humidity_rh;
} SCD4x_Data;

typedef struct SCD4x_Dev {
    uint8_t i2c_addr;              /* 7-bit I2C address: SCD4X_I2C_ADDR */
    SCD4x_Mode mode;
    scd4x_i2c_write_fn i2c_write; /* Write len bytes to device; return 0 on success */
    scd4x_i2c_read_fn i2c_read;   /* Read  len bytes from device; return 0 on success */
    scd4x_delay_ms_fn delay_ms;   /* Millisecond delay                                */
} SCD4x_Dev;

/* Public APIs */

/**
 * @brief Initialize SCD4x device and verify communication via serial number read.
 * @param[in] dev  Device handle with populated HAL callbacks and I2C address.
 * @return SCD4X_OK on success, error code otherwise.
 */
SCD4x_Status SCD4x_Init(SCD4x_Dev *dev);
/**
 * @brief Start CO2 measurement in the specified mode.
 * @param[in] dev   Device handle.
 * @param[in] mode  Measurement mode: periodic, low-power periodic, or single-shot.
 * @return SCD4X_OK on success, SCD4X_ERR_PARAM on invalid mode.
 */
SCD4x_Status SCD4x_StartMeasurement(SCD4x_Dev *dev, SCD4x_Mode mode);
/**
 * @brief Read CO2, temperature, and humidity from the last completed measurement.
 * Must be called after data is ready (see SCD4x_GetDataReadyStatus).
 * @param[in]  dev  Device handle.
 * @param[out] out  Pointer to measurement output structure.
 * @return SCD4X_OK on success, error code otherwise.
 */
SCD4x_Status SCD4x_ReadMeasurement(SCD4x_Dev *dev, SCD4x_Data *out);
/**
 * @brief Stop an ongoing periodic measurement.
 * No-op if mode is single-shot. Required before issuing configuration commands.
 * @param[in] dev  Device handle.
 * @return SCD4X_OK on success, error code otherwise.
 */
SCD4x_Status SCD4x_Stop(SCD4x_Dev *dev);
/**
 * @brief Set on-chip temperature offset to compensate self-heating.
 * Stops periodic measurement if active, applies offset, then resumes.
 * @param[in] dev       Device handle.
 * @param[in] offset_c  Temperature offset in °C [SCD4X_TEMP_OFFSET_MIN_C .. SCD4X_TEMP_OFFSET_MAX_C].
 * @return SCD4X_OK on success, SCD4X_ERR_PARAM if offset out of range.
 */
SCD4x_Status SCD4x_SetTemperatureOffset(SCD4x_Dev *dev, float offset_c);
/**
 * @brief Get the current temperature offset from the sensor.
 * @param[in]  dev       Device handle.
 * @param[out] offset_c  Decoded temperature offset in °C.
 * @return SCD4X_OK on success, error code otherwise.
 */
SCD4x_Status SCD4x_GetTemperatureOffset(SCD4x_Dev *dev, float *offset_c);
/**
 * @brief Set sensor altitude for on-chip pressure compensation.
 * Overridden by SCD4x_SetAmbientPressure if called simultaneously.
 * @param[in] dev         Device handle.
 * @param[in] altitude_m  Altitude in meters [SCD4X_ALTITUDE_MIN_M .. SCD4X_ALTITUDE_MAX_M].
 * @return SCD4X_OK on success, SCD4X_ERR_PARAM if out of range.
 */
SCD4x_Status SCD4x_SetSensorAltitude(SCD4x_Dev *dev, uint16_t altitude_m);
/**
 * @brief Get the currently configured sensor altitude.
 * @param[in]  dev         Device handle.
 * @param[out] altitude_m  Altitude in meters.
 * @return SCD4X_OK on success, error code otherwise.
 */
SCD4x_Status SCD4x_GetSensorAltitude(SCD4x_Dev *dev, uint16_t *altitude_m);
/**
 * @brief Update ambient pressure for real-time pressure compensation.
 * Can be called during periodic measurement without stopping.
 * @param[in] dev          Device handle.
 * @param[in] pressure_pa  Ambient pressure in Pa [SCD4X_PRESSURE_MIN_PA .. SCD4X_PRESSURE_MAX_PA].
 * @return SCD4X_OK on success, SCD4X_ERR_PARAM if out of range.
 */
SCD4x_Status SCD4x_SetAmbientPressure(SCD4x_Dev *dev, uint32_t pressure_pa);
/**
 * @brief Get the currently configured ambient pressure.
 * @param[in]  dev          Device handle.
 * @param[out] pressure_pa  Pressure in Pa.
 * @return SCD4X_OK on success, error code otherwise.
 */
SCD4x_Status SCD4x_GetAmbientPressure(SCD4x_Dev *dev, uint32_t *pressure_pa);
/**
 * @brief Perform forced recalibration (FRC) against a known CO2 reference.
 * Sensor must have been running in periodic measurement for >= 3 minutes
 * before calling. Stops measurement automatically.
 * @param[in]  dev             Device handle.
 * @param[in]  target_ppm      Reference CO2 concentration in ppm.
 * @param[out] correction_ppm  Applied correction offset in ppm. NULL to ignore.
 * @return SCD4X_OK on success, SCD4X_ERR_FRC if calibration failed.
 */
SCD4x_Status SCD4x_PerformForcedRecalibration(SCD4x_Dev *dev, uint16_t target_ppm, int16_t *correction_ppm);

/**
 * @brief Enable or disable Automatic Self-Calibration (ASC).
 * @param[in] dev      Device handle.
 * @param[in] enabled  true to enable ASC, false to disable.
 * @return SCD4X_OK on success, error code otherwise.
 */
SCD4x_Status SCD4x_SetAutoSelfCalibEnabled(SCD4x_Dev *dev, bool enabled);
/**
 * @brief Get the current ASC enable state.
 * @param[in]  dev      Device handle.
 * @param[out] enabled  true if ASC is enabled.
 * @return SCD4X_OK on success, error code otherwise.
 */
SCD4x_Status SCD4x_GetAutoSelfCalibEnabled(SCD4x_Dev *dev, bool *enabled);
/**
 * @brief Set the ASC target CO2 concentration.
 * Default is 400 ppm (fresh outdoor air).
 * @param[in] dev         Device handle.
 * @param[in] target_ppm  ASC reference target in ppm.
 * @return SCD4X_OK on success, error code otherwise.
 */
SCD4x_Status SCD4x_SetAutoSelfCalibTarget(SCD4x_Dev *dev, uint16_t target_ppm);
/**
 * @brief Get the configured ASC target CO2 concentration.
 * @param[in]  dev         Device handle.
 * @param[out] target_ppm  ASC target in ppm.
 * @return SCD4X_OK on success, error code otherwise.
 */
SCD4x_Status SCD4x_GetAutoSelfCalibTarget(SCD4x_Dev *dev, uint16_t *target_ppm);

/* ===================== Low power periodic measurement mode ====================== */

/**
 * @brief Check whether a new measurement result is ready to be read.
 * @param[in]  dev    Device handle.
 * @param[out] ready  true if measurement data is available.
 * @return SCD4X_OK on success, error code otherwise.
 */
SCD4x_Status SCD4x_GetDataReadyStatus(SCD4x_Dev *dev, bool *ready);

/* ==================== Advanced Features ========================== */

/**
 * @brief Persist current settings to EEPROM (survives power cycle).
 * Avoid calling frequently — EEPROM endurance is limited (~2000 write cycles).
 * @param[in] dev  Device handle.
 * @return SCD4X_OK on success, error code otherwise.
 */
SCD4x_Status SCD4x_PersistSettings(SCD4x_Dev *dev);
/**
 * @brief Read the 48-bit unique serial number from the sensor.
 * @param[in]  dev     Device handle.
 * @param[out] serial  48-bit serial number (packed into uint64_t).
 * @return SCD4X_OK on success, error code otherwise.
 */
SCD4x_Status SCD4x_GetSerialNumber(SCD4x_Dev *dev, uint64_t *serial);
/**
 * @brief Trigger on-chip self-test (~10 s blocking).
 * @param[in]  dev  Device handle.
 * @param[out] ok   true if self-test passed (response word == 0x0000).
 * @return SCD4X_OK on success, error code otherwise.
 */
SCD4x_Status SCD4x_PerformSelfTest(SCD4x_Dev *dev, bool *ok);
/**
 * @brief Restore all settings to factory defaults and clear EEPROM.
 * @param[in] dev  Device handle.
 * @return SCD4X_OK on success, error code otherwise.
 */
SCD4x_Status SCD4x_PerformFactoryReset(SCD4x_Dev *dev);
/**
 * @brief Re-initialize the sensor without power cycling.
 * Sensor must be in idle state (periodic measurement stopped).
 * @param[in] dev  Device handle.
 * @return SCD4X_OK on success, error code otherwise.
 */
SCD4x_Status SCD4x_Reinit(SCD4x_Dev *dev);
/**
 * @brief Read the sensor variant identifier (bits[15:12] of response).
 * Use to distinguish SCD40 / SCD41 / SCD43 at runtime.
 * @param[in]  dev      Device handle.
 * @param[out] variant  Masked variant value.
 * @return SCD4X_OK on success, error code otherwise.
 */
SCD4x_Status SCD4x_GetSensorVariant(SCD4x_Dev *dev, uint16_t *variant);
/* SINGLE SHOT MODE (SCD41 & SCD43 ONLY) */

/**
 * @brief Trigger a single-shot measurement (CO2 + RHT). SCD41/SCD43 only.
 * Blocks ~5 s. Call SCD4x_ReadMeasurement after completion.
 * @param[in] dev  Device handle.
 * @return SCD4X_OK on success, error code otherwise.
 */
SCD4x_Status SCD4x_MeasureSingleShot(SCD4x_Dev *dev);
/**
 * @brief Trigger a single-shot RHT-only measurement. SCD41/SCD43 only.
 * Skips CO2 measurement; blocks ~50 ms. Lower power than full single-shot.
 * @param[in] dev  Device handle.
 * @return SCD4X_OK on success, error code otherwise.
 */
SCD4x_Status SCD4x_MeasureSingleShotRHTOnly(SCD4x_Dev *dev);
/**
 * @brief Put sensor into low-power sleep state. SCD41/SCD43 only.
 * Wake with SCD4x_WakeUp(). VDD must remain powered during sleep.
 * @param[in] dev  Device handle.
 * @return SCD4X_OK on success, error code otherwise.
 */
SCD4x_Status SCD4x_PowerDown(SCD4x_Dev *dev);
/**
 * @brief Wake up sensor from power-down state. SCD41/SCD43 only.
 * The wake_up command intentionally receives no I2C ACK from the sensor —
 * this is expected per datasheet and the error is ignored.
 * Responsiveness is verified by reading the serial number after the
 * required wake-up delay.
 * @param[in] dev  Device handle.
 * @return SCD4X_OK if sensor is responsive after wake-up,
 *         SCD4X_ERR_I2C if serial number read fails.
 */
SCD4x_Status SCD4x_WakeUp(SCD4x_Dev *dev);
/**
 * @brief Set ASC initial calibration period (must be multiple of 4 hours).
 * Applies during the first @p period_h hours after deployment.
 * @param[in] dev       Device handle.
 * @param[in] period_h  Initial ASC period in hours (multiple of 4).
 * @return SCD4X_OK on success, SCD4X_ERR_PARAM if not multiple of 4.
 */
SCD4x_Status SCD4x_SetASCInitialPeriod(SCD4x_Dev *dev, uint16_t period_h);
/**
 * @brief Get the configured ASC initial period.
 * @param[in]  dev       Device handle.
 * @param[out] period_h  ASC initial period in hours.
 * @return SCD4X_OK on success, error code otherwise.
 */
SCD4x_Status SCD4x_GetASCInitialPeriod(SCD4x_Dev *dev, uint16_t *period_h);
/**
 * @brief Set ASC standard calibration period (must be multiple of 4 hours).
 * Applied after the initial period has elapsed.
 * @param[in] dev       Device handle.
 * @param[in] period_h  Standard ASC period in hours (multiple of 4).
 * @return SCD4X_OK on success, SCD4X_ERR_PARAM if not multiple of 4.
 */
SCD4x_Status SCD4x_SetASCStandardPeriod(SCD4x_Dev *dev, uint16_t period_h);
/**
 * @brief Get the configured ASC standard period.
 * @param[in]  dev       Device handle.
 * @param[out] period_h  ASC standard period in hours.
 * @return SCD4X_OK on success, error code otherwise.
 */
SCD4x_Status SCD4x_GetASCStandardPeriod(SCD4x_Dev *dev, uint16_t *period_h);

#ifdef __cplusplus
}
#endif

#endif /* SCD4X_H */