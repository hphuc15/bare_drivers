/**
 * @file scd4x.c
 * @brief Bare-metal driver implementation for Sensirion SCD4x CO2 sensor
 *        Based on Datasheet v1.7 - April 2025
 */
#include "scd4x.h"
#include "scd4x_defs.h"

/**
 * @brief Compute CRC-8 for SCD4x protocol.
 * Polynomial: 0x31
 * Initial value: 0xFF
 * @param[in] data Pointer to input buffer.
 * @param[in] len  Number of bytes to process.
 * @return Calculated CRC-8 value.
 */
uint8_t _scd4x_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = SCD4X_CRC8_INIT;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80){
                crc = (crc << 1) ^ SCD4X_CRC8_POLYNOMIAL;
            }
            else{
                crc = (crc << 1);
            }
        }
    }
    return crc;
}

/**
 * @brief Send 16-bit command to sensor (big-endian).
 * @param[in] dev  Device handle.
 * @param[in] cmd  16-bit command.
 * @return SCD4X_OK on success, error code otherwise.
 */
static SCD4x_Status _scd4x_send_command(SCD4x_Dev *dev, uint16_t cmd)
{
    if(!dev){
        return SCD4X_ERR_PARAM;
    }
    uint8_t buf[2] = {
        (uint8_t)(cmd >> 8),  /* cmd MSB */
        (uint8_t)(cmd & 0xFF) /* cmd LSB */
    };
    return (dev->i2c_write(dev->i2c_addr, buf, 2) == 0) ? SCD4X_OK : SCD4X_ERR_I2C;
}

/**
 * @brief Send 16-bit command and wait execution time.
 * Blocking delay is applied after command transmission.
 * @param[in] dev      Device handle.
 * @param[in] cmd      16-bit command.
 * @param[in] exec_ms  Execution delay in milliseconds.
 * @return SCD4X_OK on success, error code otherwise.
 */
static SCD4x_Status _scd4x_send_command_with_delay(SCD4x_Dev *dev, uint16_t cmd, uint32_t exec_ms){
    if(!dev){
        return SCD4X_ERR_PARAM;
    }
    uint8_t buf[2] = {
        (uint8_t)(cmd >> 8),  /* cmd MSB */
        (uint8_t)(cmd & 0xFF) /* cmd LSB */
    };

    if(dev->i2c_write(dev->i2c_addr, buf, 2) != 0){
        return SCD4X_ERR_I2C;
    }
    dev->delay_ms(exec_ms);
    
    return SCD4X_OK;
}

/**
 * @brief Send command with 16-bit argument and CRC.
 * Argument is transmitted in big-endian format and
 * followed by CRC-8 as required by SCD4x protocol.
 * @param[in] dev  Device handle.
 * @param[in] cmd  16-bit command.
 * @param[in] arg  16-bit argument.
 * @return SCD4X_OK on success, error code otherwise.
 */
static SCD4x_Status _scd4x_send_command_with_arg(SCD4x_Dev *dev, uint16_t cmd, uint16_t arg)
{
    if(!dev){
        return SCD4X_ERR_PARAM;
    }
    uint8_t buf[5] = {
        (uint8_t)(cmd >> 8),
        (uint8_t)(cmd & 0xFF),
        (uint8_t)(arg >> 8),
        (uint8_t)(arg & 0xFF),
        0x00
    };
    buf[4] = _scd4x_crc8(&buf[2], 2);
    return (dev->i2c_write(dev->i2c_addr, buf, 5) == 0) ? SCD4X_OK : SCD4X_ERR_I2C;
}

/**
 * @brief Send command with 16-bit argument and wait execution time.
 * @param dev       Pointer to SCD4x device descriptor.
 * @param cmd       16-bit command (big-endian on bus).
 * @param arg       16-bit argument (CRC handled internally).
 * @param exec_ms   Execution time in milliseconds.
 * @return SCD4X_OK on success, error code otherwise.
 */
static SCD4x_Status _scd4x_send_command_with_arg_delay(SCD4x_Dev *dev, uint16_t cmd, uint16_t arg, uint32_t exec_ms){
    SCD4x_Status st;

    /* Send command and argument */
    st = _scd4x_send_command_with_arg(dev, cmd, arg);
    if (st != SCD4X_OK) {
        return st;
    }

    /* Wait for command execution time */
    dev->delay_ms(exec_ms);

    return SCD4X_OK;
}

/**
 * @brief Read multiple 16-bit words from sensor.
 * Each word consists of:
 *   - 2 data bytes (MSB first)
 *   - 1 CRC byte
 * CRC is verified for each word.
 * @param[in]  dev       Device handle.
 * @param[out] words     Output buffer for decoded words.
 * @param[in]  n_words   Number of words to read.
 * @return SCD4X_OK on success,
 *         SCD4X_ERR_CRC if CRC mismatch,
 *         SCD4X_ERR_I2C on bus error.
 */
static SCD4x_Status _scd4x_read_words(SCD4x_Dev *dev, uint16_t *words, size_t n_words)
{
    size_t  n_bytes = n_words * 3;                  /* 3 bytes/word */
    uint8_t buf[n_bytes];

    if (dev->i2c_read(dev->i2c_addr, buf, n_bytes) != 0) {
        return SCD4X_ERR_I2C;
    }

    for (size_t i = 0; i < n_words; i++) {
        uint8_t *p   = &buf[i * 3];
        uint8_t  crc = _scd4x_crc8(p, 2);
        if (crc != p[2]) {
            return SCD4X_ERR_CRC;
        }
        words[i] = ((uint16_t)p[0] << 8) | p[1];
    }

    return SCD4X_OK;
}

/**
 * @brief Send command, wait execution time, then read response.
 * Blocking transaction:
 *   1. Send command
 *   2. Delay for execution time
 *   3. Read response words
 * @param[in]  dev        Device handle.
 * @param[in]  cmd        16-bit command.
 * @param[in]  exec_ms    Execution delay in milliseconds.
 * @param[out] words_out  Output buffer for response words.
 * @param[in]  n_words    Number of words to read.
 * @return SCD4X_OK on success, error code otherwise.
 */
static SCD4x_Status _scd4x_send_and_fetch(SCD4x_Dev *dev, uint16_t cmd, uint32_t exec_ms, uint16_t *words_out, size_t n_words)
{
    SCD4x_Status st = _scd4x_send_command(dev, cmd);
    if (st != SCD4X_OK){
        return st;
    }
    dev->delay_ms(exec_ms);
    return _scd4x_read_words(dev, words_out, n_words);
}

/**
 * @brief Stop measurement if currently in a periodic mode.
 *        Saves current mode so it can be resumed later.
 * @param[in]  dev       Device handle.
 * @param[out] prev_mode Mode before stopping (to resume later).
 * @return SCD4X_OK on success, error code otherwise.
 */
static SCD4x_Status _scd4x_stop_if_periodic(SCD4x_Dev *dev, SCD4x_Mode *prev_mode)
{
    *prev_mode = dev->mode;

    if (dev->mode == SCD4X_MODE_PERIODIC || dev->mode == SCD4X_MODE_PERIODIC_LOW_POWER) {
        return _scd4x_send_command_with_delay(dev, SCD4X_CMD_STOP_PERIODIC, SCD4X_EXEC_STOP_PERIODIC_MS);
    }
    return SCD4X_OK;    /* SINGLE_SHOT / already idle: nothing to do */
}

/**
 * @brief Resume measurement mode after settings change.
 * @param[in] dev   Device handle.
 * @param[in] mode  Mode to resume (saved from _scd4x_stop_if_periodic).
 * @return SCD4X_OK on success, error code otherwise.
 */
static SCD4x_Status _scd4x_resume(SCD4x_Dev *dev, SCD4x_Mode mode)
{
    switch (mode) {
        case SCD4X_MODE_PERIODIC:
            return _scd4x_send_command_with_delay(dev,
                        SCD4X_CMD_START_PERIODIC,
                        SCD4X_EXEC_START_PERIODIC_MS);

        case SCD4X_MODE_PERIODIC_LOW_POWER:
            return _scd4x_send_command_with_delay(dev,
                        SCD4X_CMD_START_LOW_POWER_PERIODIC,
                        SCD4X_EXEC_START_LOW_POWER_PERIODIC_MS);

        case SCD4X_MODE_SINGLE_SHOT:
        default:
            return SCD4X_OK;    /* No start command needed */
    }
}

/** Public APIs */


SCD4x_Status SCD4x_Init(SCD4x_Dev *dev){
    if(!dev || !(dev->i2c_read) || !(dev->i2c_write) || !(dev->delay_ms)){
        return SCD4X_ERR_PARAM;
    }
    SCD4x_Status st;

    /* Power-up time */
    dev->delay_ms(SCD4X_POWERUP_TIME_MS);

    /* Stop any going measurement unconditionally */
    _scd4x_send_command_with_delay(dev, SCD4X_CMD_STOP_PERIODIC, SCD4X_EXEC_STOP_PERIODIC_MS);

    /* Verify serial number */
    uint64_t serial;
    st = SCD4x_GetSerialNumber(dev, &serial);

    return st;
}

SCD4x_Status SCD4x_StartMeasurement(SCD4x_Dev *dev, SCD4x_Mode mode){
    if(!dev){
        return SCD4X_ERR_PARAM;
    }
    SCD4x_Status st = SCD4X_OK;
    dev->mode = mode;
    
    switch(dev->mode){
        case SCD4X_MODE_PERIODIC:{
            st = _scd4x_send_command_with_delay(dev, SCD4X_CMD_START_PERIODIC, SCD4X_EXEC_START_PERIODIC_MS);
            break;
        }
        case SCD4X_MODE_PERIODIC_LOW_POWER:{
            st = _scd4x_send_command_with_delay(dev, SCD4X_CMD_START_LOW_POWER_PERIODIC, SCD4X_EXEC_START_LOW_POWER_PERIODIC_MS);
            break;
        }
        case SCD4X_MODE_SINGLE_SHOT:{
            /* Single mode not need start command */
            break;
        }
        default:{
            st = SCD4X_ERR_PARAM;
        }
    }
    return st;
}

SCD4x_Status SCD4x_ReadMeasurement(SCD4x_Dev *dev, SCD4x_Measurement *out){
    if(!dev || !out){
        return SCD4X_ERR_PARAM;
    }
    SCD4x_Status st;

    /* Read Measurement */
    st = _scd4x_send_command_with_delay(dev, SCD4X_CMD_READ_MEASUREMENT, SCD4X_EXEC_READ_MEASUREMENT_MS);
    if(st != SCD4X_OK){
        return st;
    }

    uint16_t words[3];      /* CO2, Temperature, Humidity raw */
    st = _scd4x_read_words(dev, words, 3);
    if(st != SCD4X_OK){
        return st;
    }

    out->co2_ppm = SCD4X_CO2_PPM(words[0]);
    out->temperature_c = SCD4X_TEMP_C(words[1]);
    out->humidity_rh = SCD4X_HUMID_RH(words[2]);

    return SCD4X_OK;
}

SCD4x_Status SCD4x_Stop(SCD4x_Dev *dev){
    if(!dev){
        return SCD4X_ERR_PARAM;
    }

    switch(dev->mode){
        case SCD4X_MODE_PERIODIC:{
            return _scd4x_send_command_with_delay(dev, SCD4X_CMD_STOP_PERIODIC, SCD4X_EXEC_STOP_PERIODIC_MS);
        }
        case SCD4X_MODE_PERIODIC_LOW_POWER:{
            return _scd4x_send_command_with_delay(dev, SCD4X_CMD_STOP_PERIODIC, SCD4X_EXEC_STOP_PERIODIC_MS);
        }
        case SCD4X_MODE_SINGLE_SHOT:{
            /* SCD4X Single Shot not need stop command */
            return SCD4X_OK;
        }
        default:{
            return SCD4X_OK;
        }
    }
}

SCD4x_Status SCD4x_SetTemperatureOffset(SCD4x_Dev *dev, float offset_c){
    if(!dev || offset_c < SCD4X_TEMP_OFFSET_MIN_C || offset_c > SCD4X_TEMP_OFFSET_MAX_C){
        return SCD4X_ERR_PARAM;
    }

    SCD4x_Status st;
    SCD4x_Mode prev_mode;

    st = _scd4x_stop_if_periodic(dev, &prev_mode);
    if(st != SCD4X_OK){
        return st;
    }

    uint16_t word = SCD4X_TEMP_OFFSET_ENCODE(offset_c);
    st = _scd4x_send_command_with_arg_delay(dev, SCD4X_CMD_SET_TEMPERATURE_OFFSET, word, SCD4X_EXEC_SET_TEMPERATURE_OFFSET_MS);

    SCD4x_Status resume_st = _scd4x_resume(dev, prev_mode);
    return (st != SCD4X_OK) ? st : resume_st;
}

SCD4x_Status SCD4x_GetTemperatureOffset(SCD4x_Dev *dev, float *offset_c){
    if(!dev || !offset_c){
        return SCD4X_ERR_PARAM;
    }
    SCD4x_Status st;
    SCD4x_Mode prev_mode;

    st = _scd4x_stop_if_periodic(dev, &prev_mode);
    if(st != SCD4X_OK){
        return st;
    }

    uint16_t word;
    st = _scd4x_send_and_fetch(dev, SCD4X_CMD_GET_TEMPERATURE_OFFSET, SCD4X_EXEC_GET_TEMPERATURE_OFFSET_MS, &word, 1);
    if(st == SCD4X_OK){
        *offset_c = SCD4X_TEMP_OFFSET_DECODE(word);
    }

    SCD4x_Status resume_st = _scd4x_resume(dev, prev_mode);
    return (st != SCD4X_OK) ? st : resume_st;
}

SCD4x_Status SCD4x_SetSensorAltitude(SCD4x_Dev *dev, uint16_t altitude_m){
    if(!dev || altitude_m > SCD4X_ALTITUDE_MAX_M){
        return SCD4X_ERR_PARAM;
    }
    SCD4x_Status st;
    SCD4x_Mode prev_mode;
    
    st = _scd4x_stop_if_periodic(dev, &prev_mode);
    if(st != SCD4X_OK){
        return st;
    }

    st = _scd4x_send_command_with_arg_delay(dev, SCD4X_CMD_SET_SENSOR_ALTITUDE, altitude_m, SCD4X_EXEC_SET_SENSOR_ALTITUDE_MS);

    SCD4x_Status resume_st = _scd4x_resume(dev, prev_mode);
    return (st != SCD4X_OK) ? st : resume_st;
}

SCD4x_Status SCD4x_GetSensorAltitude(SCD4x_Dev *dev, uint16_t *altitude_m)
{
    if (!dev || !altitude_m) {
        return SCD4X_ERR_PARAM;
    }

    SCD4x_Status st;
    SCD4x_Mode prev_mode;

    st = _scd4x_stop_if_periodic(dev, &prev_mode);
    if(st != SCD4X_OK){
        return st;
    }

    st = _scd4x_send_and_fetch(dev, SCD4X_CMD_GET_SENSOR_ALTITUDE, SCD4X_EXEC_GET_SENSOR_ALTITUDE_MS, altitude_m, 1);

    SCD4x_Status resume_st = _scd4x_resume(dev, prev_mode);
    return (st != SCD4X_OK) ? st : resume_st;
}

SCD4x_Status SCD4x_SetAmbientPressure(SCD4x_Dev *dev, uint32_t pressure_pa)
{
    if (!dev || pressure_pa < SCD4X_PRESSURE_MIN_PA || pressure_pa > SCD4X_PRESSURE_MAX_PA){
        return SCD4X_ERR_PARAM;
    }

    uint16_t word = SCD4X_PRESSURE_ENCODE(pressure_pa);
    return _scd4x_send_command_with_arg_delay(dev, SCD4X_CMD_AMBIENT_PRESSURE, word, SCD4X_EXEC_AMBIENT_PRESSURE_MS);
}

SCD4x_Status SCD4x_GetAmbientPressure(SCD4x_Dev *dev, uint32_t *pressure_pa)
{
    if (!dev || !pressure_pa){
        return SCD4X_ERR_PARAM;
    }

    uint16_t word;
    SCD4x_Status st = _scd4x_send_and_fetch(dev, SCD4X_CMD_AMBIENT_PRESSURE, SCD4X_EXEC_AMBIENT_PRESSURE_MS, &word, 1);

    if(st == SCD4X_OK){
        *pressure_pa = SCD4X_PRESSURE_DECODE(word);
    }
    return st;
}

SCD4x_Status SCD4x_PerformForcedRecalibration(SCD4x_Dev *dev, uint16_t target_ppm, int16_t *correction_ppm)
{
    if(!dev){
        return SCD4X_ERR_PARAM;
    }

    /* Send command with target concentration */
    uint8_t buf[5] = {
        (uint8_t)(SCD4X_CMD_PERFORM_FORCED_RECALIBRATION >> 8),
        (uint8_t)(SCD4X_CMD_PERFORM_FORCED_RECALIBRATION & 0xFF),
        (uint8_t)(target_ppm >> 8),
        (uint8_t)(target_ppm & 0xFF),
        0x00
    };
    buf[4] = _scd4x_crc8(&buf[2], 2);

    int ret = dev->i2c_write(dev->i2c_addr, buf, 5);
    if (ret != 0){
        return SCD4X_ERR_I2C;   
    }

    dev->delay_ms(SCD4X_EXEC_FORCED_RECALIBRATION_MS);

    uint16_t word;
    SCD4x_Status st = _scd4x_read_words(dev, &word, 1);
    if (st != SCD4X_OK){
        return st;
    }

    if (word == SCD4X_FRC_FAILED){
        return SCD4X_ERR_FRC;
    }

    if (correction_ppm){
        *correction_ppm = SCD4X_FRC_CORRECTION_PPM(word);
    }

    return SCD4X_OK;
}

SCD4x_Status SCD4x_SetAutoSelfCalibEnabled(SCD4x_Dev *dev, bool enabled)
{
    if (!dev) {
        return SCD4X_ERR_PARAM;
    }

    SCD4x_Status st;
    SCD4x_Mode prev_mode;

    st = _scd4x_stop_if_periodic(dev, &prev_mode);
    if (st != SCD4X_OK) return st;

    uint16_t word = enabled ? 0x0001 : 0x0000;
    st = _scd4x_send_command_with_arg_delay(dev, SCD4X_CMD_SET_ASC_ENABLED, word, SCD4X_EXEC_SET_ASC_ENABLED_MS);

    SCD4x_Status resume_st = _scd4x_resume(dev, prev_mode);
    return (st != SCD4X_OK) ? st : resume_st;
}

SCD4x_Status SCD4x_GetAutoSelfCalibEnabled(SCD4x_Dev *dev, bool *enabled)
{
    if (!dev || !enabled) {
        return SCD4X_ERR_PARAM;
    }

    SCD4x_Status st;
    SCD4x_Mode prev_mode;

    st = _scd4x_stop_if_periodic(dev, &prev_mode);
    if (st != SCD4X_OK) return st;

    uint16_t word;
    st = _scd4x_send_and_fetch(dev, SCD4X_CMD_GET_ASC_ENABLED, SCD4X_EXEC_GET_ASC_ENABLED_MS, &word, 1);
    if (st == SCD4X_OK) {
        *enabled = (word == 0x0001);
    }

    SCD4x_Status resume_st = _scd4x_resume(dev, prev_mode);
    return (st != SCD4X_OK) ? st : resume_st;
}

SCD4x_Status SCD4x_SetAutoSelfCalibTarget(SCD4x_Dev *dev, uint16_t target_ppm)
{
    if (!dev) {
        return SCD4X_ERR_PARAM;
    }

    SCD4x_Status st;
    SCD4x_Mode prev_mode;

    st = _scd4x_stop_if_periodic(dev, &prev_mode);
    if (st != SCD4X_OK){
        return st;
    }

    st = _scd4x_send_command_with_arg_delay(dev, SCD4X_CMD_SET_ASC_TARGET, target_ppm, SCD4X_EXEC_SET_ASC_TARGET_MS);

    SCD4x_Status resume_st = _scd4x_resume(dev, prev_mode);
    return (st != SCD4X_OK) ? st : resume_st;
}

SCD4x_Status SCD4x_GetAutoSelfCalibTarget(SCD4x_Dev *dev, uint16_t *target_ppm)
{
    if (!dev || !target_ppm) {
        return SCD4X_ERR_PARAM;
    }

    SCD4x_Status st;
    SCD4x_Mode prev_mode;

    st = _scd4x_stop_if_periodic(dev, &prev_mode);
    if (st != SCD4X_OK){
        return st;
    }

    st = _scd4x_send_and_fetch(dev, SCD4X_CMD_GET_ASC_TARGET, SCD4X_EXEC_GET_ASC_TARGET_MS, target_ppm, 1);

    SCD4x_Status resume_st = _scd4x_resume(dev, prev_mode);
    return (st != SCD4X_OK) ? st : resume_st;
}

/* ===================== Low power periodic measurement mode ====================== */
SCD4x_Status SCD4x_GetDataReadyStatus(SCD4x_Dev *dev, bool *ready){
    if(!dev || !ready){
        return SCD4X_ERR_PARAM;
    }
    SCD4x_Status st;

    uint16_t word;
    st = _scd4x_send_and_fetch(dev, SCD4X_CMD_GET_DATA_READY_STATUS, SCD4X_EXEC_GET_DATA_READY_STATUS_MS, &word, 1);
    if(st != SCD4X_OK){
        return st;
    }

    *ready = SCD4X_IS_DATA_READY(word);
    return SCD4X_OK;
}


/* ==================== Advanced Features ========================== */


SCD4x_Status SCD4x_PersistSettings(SCD4x_Dev *dev){
    if(!dev){
        return SCD4X_ERR_PARAM;
    }
    return _scd4x_send_command_with_delay(dev, SCD4X_CMD_PERSIST_SETTINGS, SCD4X_EXEC_PERSIST_SETTINGS_MS);
}

SCD4x_Status SCD4x_GetSerialNumber(SCD4x_Dev *dev, uint64_t *serial){
    if(!dev || !serial){
        return SCD4X_ERR_PARAM;
    }
    SCD4x_Status st;

    uint16_t words[3];
    st = _scd4x_send_and_fetch(dev, SCD4X_CMD_GET_SERIAL_NUMBER, SCD4X_EXEC_GET_SERIAL_NUMBER_MS, words, 3);
    if(st != SCD4X_OK){
        return st;
    }

    *serial = ((uint64_t)words[0] << 32)| ((uint64_t)words[1] << 16) | ((uint64_t)words[2]);
    return SCD4X_OK;
}

SCD4x_Status SCD4x_PerformSelfTest(SCD4x_Dev *dev, bool *ok)
{
    if(!dev || !ok){
        return SCD4X_ERR_PARAM;
    }
    SCD4x_Status st;

    uint16_t word;
    st = _scd4x_send_and_fetch(dev, SCD4X_CMD_PERFORM_SELF_TEST, SCD4X_EXEC_PERFORM_SELF_TEST_MS, &word, 1);
    if (st != SCD4X_OK){
        return st;
    }

    *ok = (word == 0x0000);
    return SCD4X_OK;
}

SCD4x_Status SCD4x_PerformFactoryReset(SCD4x_Dev *dev)
{
    if(!dev){
        return SCD4X_ERR_PARAM;
    }
    return _scd4x_send_command_with_delay(dev, SCD4X_CMD_PERFORM_FACTORY_RESET, SCD4X_EXEC_PERFORM_FACTORY_RESET_MS);
}

SCD4x_Status SCD4x_Reinit(SCD4x_Dev *dev)
{
    if(!dev){
        return SCD4X_ERR_PARAM;
    }
    return _scd4x_send_command_with_delay(dev, SCD4X_CMD_REINIT, SCD4X_EXEC_REINIT_MS);
}

SCD4x_Status SCD4x_GetSensorVariant(SCD4x_Dev *dev, uint16_t *variant)
{
    if(!dev || !variant){
        return SCD4X_ERR_PARAM;
    }
    SCD4x_Status st;

    uint16_t word;
    st = _scd4x_send_and_fetch(dev, SCD4X_CMD_GET_SENSOR_VARIANT, SCD4X_EXEC_GET_SENSOR_VARIANT_MS, &word, 1);
    if (st != SCD4X_OK){
        return st;
    }

    /* bits[15:12] find variant */
    *variant = word & SCD4X_VARIANT_MASK;
    return SCD4X_OK;
}

/* SINGLE SHOT MODE (SCD41 & SCD43 ONLY) */


SCD4x_Status SCD4x_MeasureSingleShot(SCD4x_Dev *dev)
{
    if(!dev){
        return SCD4X_ERR_PARAM;
    }
    return _scd4x_send_command_with_delay(dev, SCD4X_CMD_MEASURE_SINGLE_SHOT, SCD4X_EXEC_MEASURE_SINGLE_SHOT_MS);
}

SCD4x_Status SCD4x_MeasureSingleShotRHTOnly(SCD4x_Dev *dev)
{
    if(!dev){
        return SCD4X_ERR_PARAM;
    }
    /* NOTE (Datasheet §3.11.2): CO2 output will be 0 ppm in read_measurement.
     * Only RH and temperature values are valid after this command. */
    return _scd4x_send_command_with_delay(dev, SCD4X_CMD_MEASURE_SINGLE_SHOT_RHT_ONLY, SCD4X_EXEC_MEASURE_SINGLE_SHOT_RHT_MS);
}

SCD4x_Status SCD4x_PowerDown(SCD4x_Dev *dev)
{
    if(!dev){
        return SCD4X_ERR_PARAM;
    }
    return _scd4x_send_command_with_delay(dev, SCD4X_CMD_POWER_DOWN, SCD4X_EXEC_POWER_DOWN_MS);
}

SCD4x_Status SCD4x_WakeUp(SCD4x_Dev *dev)
{
    if (!dev) return SCD4X_ERR_PARAM;

    /* Send raw bytes and ignore NACK from i2c_write. */
    uint8_t buf[2] = {
        (uint8_t)(SCD4X_CMD_WAKE_UP >> 8),
        (uint8_t)(SCD4X_CMD_WAKE_UP & 0xFF)
    };
    (void)dev->i2c_write(dev->i2c_addr, buf, 2); /* NACK expected, intentionally ignored */
    dev->delay_ms(SCD4X_EXEC_WAKE_UP_MS);         /* 30ms guaranteed regardless */

    /* Verify idle state per datasheet recommendation */
    uint64_t serial;
    return SCD4x_GetSerialNumber(dev, &serial);
}

SCD4x_Status SCD4x_SetASCInitialPeriod(SCD4x_Dev *dev, uint16_t period_h)
{
    if (!dev || period_h % 4 != 0) {
        return SCD4X_ERR_PARAM;
    }

    SCD4x_Status st;
    SCD4x_Mode prev_mode;

    st = _scd4x_stop_if_periodic(dev, &prev_mode);
    if (st != SCD4X_OK){
        return st;
    }

    st = _scd4x_send_command_with_arg_delay(dev, SCD4X_CMD_SET_ASC_INITIAL_PERIOD, period_h, SCD4X_EXEC_SET_ASC_INITIAL_PERIOD_MS);

    SCD4x_Status resume_st = _scd4x_resume(dev, prev_mode);
    return (st != SCD4X_OK) ? st : resume_st;
}

SCD4x_Status SCD4x_GetASCInitialPeriod(SCD4x_Dev *dev, uint16_t *period_h)
{
    if (!dev || !period_h) {
        return SCD4X_ERR_PARAM;
    }

    SCD4x_Status st;
    SCD4x_Mode prev_mode;

    st = _scd4x_stop_if_periodic(dev, &prev_mode);
    if (st != SCD4X_OK){
        return st;
    }

    st = _scd4x_send_and_fetch(dev, SCD4X_CMD_GET_ASC_INITIAL_PERIOD, SCD4X_EXEC_GET_ASC_INITIAL_PERIOD_MS, period_h, 1);

    SCD4x_Status resume_st = _scd4x_resume(dev, prev_mode);
    return (st != SCD4X_OK) ? st : resume_st;
}

SCD4x_Status SCD4x_SetASCStandardPeriod(SCD4x_Dev *dev, uint16_t period_h)
{
    if (!dev || period_h % 4 != 0) {
        return SCD4X_ERR_PARAM;
    }

    SCD4x_Status st;
    SCD4x_Mode prev_mode;

    st = _scd4x_stop_if_periodic(dev, &prev_mode);
    if (st != SCD4X_OK){
        return st;
    }

    st = _scd4x_send_command_with_arg_delay(dev, SCD4X_CMD_SET_ASC_STANDARD_PERIOD, period_h, SCD4X_EXEC_SET_ASC_STANDARD_PERIOD_MS);

    SCD4x_Status resume_st = _scd4x_resume(dev, prev_mode);
    return (st != SCD4X_OK) ? st : resume_st;
}

SCD4x_Status SCD4x_GetASCStandardPeriod(SCD4x_Dev *dev, uint16_t *period_h)
{
    if (!dev || !period_h) {
        return SCD4X_ERR_PARAM;
    }

    SCD4x_Status st;
    SCD4x_Mode prev_mode;

    st = _scd4x_stop_if_periodic(dev, &prev_mode);
    if (st != SCD4X_OK){
        return st;
    }

    st = _scd4x_send_and_fetch(dev, SCD4X_CMD_GET_ASC_STANDARD_PERIOD, SCD4X_EXEC_GET_ASC_STANDARD_PERIOD_MS, period_h, 1);

    SCD4x_Status resume_st = _scd4x_resume(dev, prev_mode);
    return (st != SCD4X_OK) ? st : resume_st;
}