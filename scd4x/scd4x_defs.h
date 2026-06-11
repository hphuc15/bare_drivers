/* ============================================================
 * scd4x_defs.h
 * SCD4x (SCD40 / SCD41 / SCD43) — Register & Constant Definitions
 * Ref: Datasheet v1.7
 * ============================================================ */
/**
 * @file scd4x_defs.h
 * @brief   SCD4x (SCD40 / SCD41 / SCD43) — Register & Constant Definitions
 *          Ref: Datasheet v1.7
 */
#ifndef SCD4X_DEFS_H
#define SCD4X_DEFS_H

#include <stdint.h>
#include <stdbool.h>

/* ============================================================
 * 1. SENSOR VARIANT
 *    Ref: Section 3.10 — get_sensor_variant, bits[15:12]
 * ============================================================ */
#define SCD4X_VARIANT_MASK          0xF000

typedef enum {
    SCD4X_VARIANT_SCD40 = 0x0000,
    SCD4X_VARIANT_SCD41 = 0x1000,
    SCD4X_VARIANT_SCD43 = 0x5000,
} scd4x_variant_t;

/* ============================================================
 * 2. POWER & RESET TIMING
 *    Ref: Section 2.4, Table 7
 * ============================================================ */
#define SCD4X_POWERUP_TIME_MS       30
#define SCD4X_SOFT_RESET_TIME_MS    30

/* ============================================================
 * 3. COMMANDS & EXECUTION TIMES
 *    Ref: Section 3.5 Table 9, Section 2.4 Table 7
 *
 *    Each command is paired with its worst-case execution time.
 *    Caller must wait at least CMD_EXEC_MS before next I2C transaction.
 *
 *    Naming: SCD4X_CMD_<NAME>        — 16-bit command code (uint16_t)
 *            SCD4X_EXEC_<NAME>_MS    — execution time in milliseconds
 * ============================================================ */

/* --- 3.1 Basic Measurement (Section 3.6) ------------------- */
#define SCD4X_CMD_START_PERIODIC                0x21B1
#define SCD4X_EXEC_START_PERIODIC_MS            0

#define SCD4X_CMD_READ_MEASUREMENT              0xEC05
#define SCD4X_EXEC_READ_MEASUREMENT_MS          1

#define SCD4X_CMD_STOP_PERIODIC                 0x3F86
#define SCD4X_EXEC_STOP_PERIODIC_MS             500

/* --- 3.2 Low Power Periodic Mode (Section 3.9) ------------- */
#define SCD4X_CMD_START_LOW_POWER_PERIODIC      0x21AC
#define SCD4X_EXEC_START_LOW_POWER_PERIODIC_MS  0

#define SCD4X_CMD_GET_DATA_READY_STATUS         0xE4B8
#define SCD4X_EXEC_GET_DATA_READY_STATUS_MS     1

/* --- 3.3 Single Shot Mode (Section 3.11) ------------------- *
 *    SCD41 & SCD43 ONLY.                                       *
 *    power_down/wake_up only needed after a previous           *
 *    power_down call; not required on first VDD supply.        */
#define SCD4X_CMD_MEASURE_SINGLE_SHOT           0x219D
#define SCD4X_EXEC_MEASURE_SINGLE_SHOT_MS       5000

#define SCD4X_CMD_MEASURE_SINGLE_SHOT_RHT_ONLY  0x2196
#define SCD4X_EXEC_MEASURE_SINGLE_SHOT_RHT_MS   50

#define SCD4X_CMD_POWER_DOWN                    0x36E0
#define SCD4X_EXEC_POWER_DOWN_MS                1

#define SCD4X_CMD_WAKE_UP                       0x36F6  /* No ACK returned by sensor */
#define SCD4X_EXEC_WAKE_UP_MS                   30

/* --- 3.4 On-Chip Signal Compensation (Section 3.7) --------- */
#define SCD4X_CMD_SET_TEMPERATURE_OFFSET        0x241D
#define SCD4X_EXEC_SET_TEMPERATURE_OFFSET_MS    1

#define SCD4X_CMD_GET_TEMPERATURE_OFFSET        0x2318
#define SCD4X_EXEC_GET_TEMPERATURE_OFFSET_MS    1

#define SCD4X_CMD_SET_SENSOR_ALTITUDE           0x2427
#define SCD4X_EXEC_SET_SENSOR_ALTITUDE_MS       1

#define SCD4X_CMD_GET_SENSOR_ALTITUDE           0x2322
#define SCD4X_EXEC_GET_SENSOR_ALTITUDE_MS       1

/* SET and GET share the same code; direction is controlled by R/W bit */
#define SCD4X_CMD_AMBIENT_PRESSURE              0xE000  /* write: R/W=0, read:  R/W=1 */
#define SCD4X_EXEC_AMBIENT_PRESSURE_MS          1

/* --- 3.5 Field Calibration (Section 3.8) ------------------- */
#define SCD4X_CMD_PERFORM_FORCED_RECALIBRATION  0x362F
#define SCD4X_EXEC_FORCED_RECALIBRATION_MS      400

#define SCD4X_CMD_SET_ASC_ENABLED               0x2416
#define SCD4X_EXEC_SET_ASC_ENABLED_MS           1

#define SCD4X_CMD_GET_ASC_ENABLED               0x2313
#define SCD4X_EXEC_GET_ASC_ENABLED_MS           1

#define SCD4X_CMD_SET_ASC_TARGET                0x243A
#define SCD4X_EXEC_SET_ASC_TARGET_MS            1

#define SCD4X_CMD_GET_ASC_TARGET                0x233F
#define SCD4X_EXEC_GET_ASC_TARGET_MS            1

#define SCD4X_CMD_SET_ASC_INITIAL_PERIOD        0x2445  /* SCD41 & SCD43 only */
#define SCD4X_EXEC_SET_ASC_INITIAL_PERIOD_MS    1

#define SCD4X_CMD_GET_ASC_INITIAL_PERIOD        0x2340  /* SCD41 & SCD43 only */
#define SCD4X_EXEC_GET_ASC_INITIAL_PERIOD_MS    1

#define SCD4X_CMD_SET_ASC_STANDARD_PERIOD       0x244E  /* SCD41 & SCD43 only */
#define SCD4X_EXEC_SET_ASC_STANDARD_PERIOD_MS   1

#define SCD4X_CMD_GET_ASC_STANDARD_PERIOD       0x234B  /* SCD41 & SCD43 only */
#define SCD4X_EXEC_GET_ASC_STANDARD_PERIOD_MS   1

/* --- 3.6 Advanced Features (Section 3.10) ------------------ */
#define SCD4X_CMD_PERSIST_SETTINGS              0x3615
#define SCD4X_EXEC_PERSIST_SETTINGS_MS          800

#define SCD4X_CMD_GET_SERIAL_NUMBER             0x3682
#define SCD4X_EXEC_GET_SERIAL_NUMBER_MS         1

#define SCD4X_CMD_PERFORM_SELF_TEST             0x3639
#define SCD4X_EXEC_PERFORM_SELF_TEST_MS         10000

#define SCD4X_CMD_PERFORM_FACTORY_RESET         0x3632
#define SCD4X_EXEC_PERFORM_FACTORY_RESET_MS     1200

#define SCD4X_CMD_REINIT                        0x3646
#define SCD4X_EXEC_REINIT_MS                    30

#define SCD4X_CMD_GET_SENSOR_VARIANT            0x202F
#define SCD4X_EXEC_GET_SENSOR_VARIANT_MS        1

/* ============================================================
 * 4. LIMITS & DEFAULTS
 *    Ref: Section 3.7, 3.8
 * ============================================================ */

/* Altitude compensation (meters) */
#define SCD4X_ALTITUDE_MIN_M                    0
#define SCD4X_ALTITUDE_MAX_M                    3000
#define SCD4X_ALTITUDE_DEFAULT_M                0

/* Ambient pressure (Pa) */
#define SCD4X_PRESSURE_MIN_PA                   70000
#define SCD4X_PRESSURE_MAX_PA                   120000
#define SCD4X_PRESSURE_DEFAULT_PA               101300

/* Temperature offset (°C) */
#define SCD4X_TEMP_OFFSET_MIN_C                 0.0f
#define SCD4X_TEMP_OFFSET_MAX_C                 20.0f
#define SCD4X_TEMP_OFFSET_DEFAULT_C             4.0f

/* Auto Self-Calibration (ASC) */
#define SCD4X_ASC_TARGET_DEFAULT_PPM            400
#define SCD4X_ASC_INITIAL_PERIOD_DEFAULT_H      44
#define SCD4X_ASC_STANDARD_PERIOD_DEFAULT_H     156

/* ============================================================
 * 5. SPECIAL VALUES & MASKS
 * ============================================================ */
#define SCD4X_FRC_FAILED                        0xFFFF
#define SCD4X_DATA_READY_MASK                   0x07FF  /* bits[10:0] */

/* ============================================================
 * 6. CRC-8
 *    Ref: Section 3.12, Table 40
 *    Polynomial: x^8 + x^5 + x^4 + 1 (0x31), Init: 0xFF
 * ============================================================ */
#define SCD4X_CRC8_POLYNOMIAL                   0x31
#define SCD4X_CRC8_INIT                         0xFF

/* ============================================================
 * 7. SIGNAL CONVERSION
 *    Ref: Section 3.6.2
 *
 *    Raw values are 16-bit words as received over I2C (after CRC strip).
 *    Float results assume standard IEEE 754 single precision.
 * ============================================================ */

/* Decode raw sensor words → physical units */
#define SCD4X_CO2_PPM(raw)                      ((uint16_t)(raw))
#define SCD4X_TEMP_C(raw)                       (-45.0f + 175.0f * (float)(raw) / 65535.0f)
#define SCD4X_HUMID_RH(raw)                     (100.0f  * (float)(raw) / 65535.0f)
#define SCD4X_TEMP_OFFSET_DECODE(raw)           ((float)(raw) * 175.0f / 65535.0f)
#define SCD4X_PRESSURE_DECODE(raw)              ((uint32_t)(raw) * 100)
#define SCD4X_FRC_CORRECTION_PPM(raw)           ((int16_t)((raw) - 0x8000))
#define SCD4X_TEMP_OFFSET_ENCODE(t_c)           ((uint16_t)((t_c) * 65535.0f / 175.0f))
#define SCD4X_PRESSURE_ENCODE(pa)               ((uint16_t)((pa) / 100))
#define SCD4X_IS_DATA_READY(raw)                (((raw) & SCD4X_DATA_READY_MASK) != 0)

#endif /* SCD4X_DEFS_H */