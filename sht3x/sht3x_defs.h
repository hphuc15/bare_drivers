/**
 * @file    sht3x_defs.h
 * @brief   SHT3x-DIS register map, command codes, bitmasks and timing constants.
 * @details Sourced from Sensirion SHT3x-DIS Datasheet, August 2016 - Version 3.
 * @version 1.0
 */

#ifndef SHT3X_DEFS_H
#define SHT3X_DEFS_H

/* =========================================================================
 * @defgroup SHT3X_ADDR I2C Addresses
 * @brief    7-bit I2C device addresses (Table 7).
 * @{
 * ========================================================================= */

#define SHT3X_I2C_ADDR_VSS              0x44u /**< ADDR pin connected to VSS (default) */
#define SHT3X_I2C_ADDR_VDD              0x45u /**< ADDR pin connected to VDD           */

/** @} */ /* end group SHT3X_ADDR */

/* =========================================================================
 * @defgroup SHT3X_CONV Signal Conversion Macros
 * @brief    Convert raw 16-bit sensor output to physical values (Section 4.13).
 * @{
 * ========================================================================= */

/**
 * @def     SHT3X_TEMP_C(raw)
 * @brief   Convert raw temperature value to degrees Celsius.
 * @param   raw  16-bit raw temperature output from sensor.
 * @return  Temperature in °C as float.
 */
#define SHT3X_TEMP_C(raw)               (-45.0f + 175.0f * (float)(raw) / 65535.0f)

/**
 * @def     SHT3X_HUMID_RH(raw)
 * @brief   Convert raw humidity value to relative humidity percentage.
 * @param   raw  16-bit raw humidity output from sensor.
 * @return  Relative humidity in %RH as float.
 */
#define SHT3X_HUMID_RH(raw)             (100.0f * (float)(raw) / 65535.0f)

/** @} */ /* end group SHT3X_CONV */

/* =========================================================================
 * @defgroup SHT3X_CMD_SINGLESHOT Single-Shot Commands
 * @brief    Trigger a single measurement (Table 8).
 *           CS = Clock Stretching; HIGH/MED/LOW = repeatability.
 * @{
 * ========================================================================= */

#define SHT3X_CMD_SINGLESHOT_CS_HIGH    0x2C06u /**< Single-shot, clock stretch, high repeatability   */
#define SHT3X_CMD_SINGLESHOT_CS_MED     0x2C0Du /**< Single-shot, clock stretch, medium repeatability */
#define SHT3X_CMD_SINGLESHOT_CS_LOW     0x2C10u /**< Single-shot, clock stretch, low repeatability    */
#define SHT3X_CMD_SINGLESHOT_HIGH       0x2400u /**< Single-shot, no clock stretch, high repeatability   */
#define SHT3X_CMD_SINGLESHOT_MED        0x240Bu /**< Single-shot, no clock stretch, medium repeatability */
#define SHT3X_CMD_SINGLESHOT_LOW        0x2416u /**< Single-shot, no clock stretch, low repeatability    */

/** @} */ /* end group SHT3X_CMD_SS */

/* =========================================================================
 * @defgroup SHT3X_CMD_PERIODIC Periodic Mode Commands
 * @brief    Start continuous measurement at fixed rate (Table 9).
 *           Naming: SHT3X_CMD_PERIODIC_{mps}_{repeatability}
 *           mps: 05 = 0.5, 1, 2, 4, 10 measurements per second.
 * @{
 * ========================================================================= */

#define SHT3X_CMD_PERIODIC_05_HIGH      0x2032u /**< 0.5 mps, high repeatability   */
#define SHT3X_CMD_PERIODIC_05_MED       0x2024u /**< 0.5 mps, medium repeatability */
#define SHT3X_CMD_PERIODIC_05_LOW       0x202Fu /**< 0.5 mps, low repeatability    */
#define SHT3X_CMD_PERIODIC_1_HIGH       0x2130u /**< 1 mps, high repeatability     */
#define SHT3X_CMD_PERIODIC_1_MED        0x2126u /**< 1 mps, medium repeatability   */
#define SHT3X_CMD_PERIODIC_1_LOW        0x212Du /**< 1 mps, low repeatability      */
#define SHT3X_CMD_PERIODIC_2_HIGH       0x2236u /**< 2 mps, high repeatability     */
#define SHT3X_CMD_PERIODIC_2_MED        0x2220u /**< 2 mps, medium repeatability   */
#define SHT3X_CMD_PERIODIC_2_LOW        0x222Bu /**< 2 mps, low repeatability      */
#define SHT3X_CMD_PERIODIC_4_HIGH       0x2334u /**< 4 mps, high repeatability     */
#define SHT3X_CMD_PERIODIC_4_MED        0x2322u /**< 4 mps, medium repeatability   */
#define SHT3X_CMD_PERIODIC_4_LOW        0x2329u /**< 4 mps, low repeatability      */
#define SHT3X_CMD_PERIODIC_10_HIGH      0x2737u /**< 10 mps, high repeatability    */
#define SHT3X_CMD_PERIODIC_10_MED       0x2721u /**< 10 mps, medium repeatability  */
#define SHT3X_CMD_PERIODIC_10_LOW       0x272Au /**< 10 mps, low repeatability     */

/** @} */ /* end group SHT3X_CMD_PERIOD */

/* =========================================================================
 * @defgroup SHT3X_CMD_MISC Miscellaneous Commands
 * @brief    Control, reset, heater, and status register commands.
 * @{
 * ========================================================================= */

#define SHT3X_CMD_FETCH_DATA            0xE000u /**< Read measurement result from buffer         */
#define SHT3X_CMD_ART                   0x2B32u /**< Activate Accelerated Response Time mode     */
#define SHT3X_CMD_BREAK                 0x3093u /**< Stop periodic measurement, return to idle   */
#define SHT3X_CMD_SOFT_RESET            0x30A2u /**< Soft reset via I2C interface                */
#define SHT3X_CMD_HEATER_ON             0x306Du /**< Enable on-chip heater                       */
#define SHT3X_CMD_HEATER_OFF            0x3066u /**< Disable on-chip heater                      */
#define SHT3X_CMD_READ_STATUS           0xF32Du /**< Read 16-bit status register                 */
#define SHT3X_CMD_CLEAR_STATUS          0x3041u /**< Clear all flags in status register          */

#define SHT3X_GENERAL_CALL_ADDR         0x00u   /**< I2C general call address (Table 14) */
#define SHT3X_GENERAL_CALL_RESET        0x06u   /**< Second byte to trigger general call reset */

/** @} */ /* end group SHT3X_CMD_MISC */

/* =========================================================================
 * @defgroup SHT3X_SREG Status Register Bitmasks
 * @brief    Bitmasks for decoding the 16-bit status register (Table 17).
 * @{
 * ========================================================================= */

#define SHT3X_SREG_ALERT_PENDING        (1u << 15) /**< At least one pending alert              */
#define SHT3X_SREG_HEATER_ON            (1u << 13) /**< On-chip heater is enabled               */
#define SHT3X_SREG_RH_ALERT             (1u << 11) /**< Humidity tracking alert active          */
#define SHT3X_SREG_T_ALERT              (1u << 10) /**< Temperature tracking alert active       */
#define SHT3X_SREG_RESET_DETECTED       (1u <<  4) /**< Reset detected since last status clear  */
#define SHT3X_SREG_CMD_FAILED           (1u <<  1) /**< Last command was not processed          */
#define SHT3X_SREG_CRC_WRITE_FAIL       (1u <<  0) /**< CRC mismatch on last write transfer     */

/** @} */ /* end group SHT3X_SREG */

/* =========================================================================
 * @defgroup SHT3X_EXEC   Execution Timing Constants
 * @brief    Safe maximum execution timing values in milliseconds (Table 4).
 * @{
 * ========================================================================= */

#define SHT3X_EXEC_POWERUP_MS           2u  /**< Power-up time before first command (ms)          */
#define SHT3X_EXEC_SOFT_RESET_MS        2u  /**< Soft reset execution time (ms)                   */
#define SHT3X_EXEC_MEAS_LOW_MS          4u  /**< Max measurement duration, low repeatability (ms) */
#define SHT3X_EXEC_MEAS_MED_MS          6u  /**< Max measurement duration, med repeatability (ms) */
#define SHT3X_EXEC_MEAS_HIGH_MS         15u /**< Max measurement duration, high repeatability (ms)*/
#define SHT3X_EXEC_BREAK_MS             15u /**< Time required after BREAK command (ms)           */

/** @} */ /* end group SHT3X_TIMING */

/* =========================================================================
 * @defgroup SHT3X_CRC CRC-8 Parameters
 * @brief    CRC parameters for data integrity verification (Table 19).
 *           Polynomial: 0x31 (x^8 + x^5 + x^4 + 1), Initial value: 0xFF.
 * @{
 * ========================================================================= */

#define SHT3X_CRC_POLY                  0x31u /**< CRC-8 polynomial: x^8 + x^5 + x^4 + 1 */
#define SHT3X_CRC_INIT                  0xFFu /**< CRC-8 initial value                     */

/** @} */ /* end group SHT3X_CRC */

#endif /* SHT3X_DEFS_H */