#ifndef SPS30_DEFS_H
#define SPS30_DEFS_H

/* =========================================================================
 * Private constants
 * ========================================================================= */

#define SPS30_T_POWERUP_MS          1000u

/* SHDLC framing */
#define SHDLC_START_STOP            0x7Eu
#define SHDLC_SLAVE_ADDR            0x00u

/* Byte-stuffing pairs (datasheet Table 5) */
#define STUFF_ESC                   0x7Du
#define STUFF_7E_ORIG               0x7Eu
#define STUFF_7E_REPL               0x5Eu
#define STUFF_7D_REPL               0x5Du
#define STUFF_11_ORIG               0x11u
#define STUFF_11_REPL               0x31u
#define STUFF_13_ORIG               0x13u
#define STUFF_13_REPL               0x33u

/* SHDLC command codes */
#define SPS30_SHDLC_CMD_START_MEAS          0x00u
#define SPS30_SHDLC_CMD_STOP_MEAS           0x01u
#define SPS30_SHDLC_CMD_READ_MEAS           0x03u
#define SPS30_SHDLC_CMD_SLEEP               0x10u
#define SPS30_SHDLC_CMD_WAKEUP              0x11u
#define SPS30_SHDLC_CMD_FAN_CLEAN           0x56u
#define SPS30_SHDLC_CMD_AUTO_CLEAN_INTERVAL 0x80u
#define SPS30_SHDLC_CMD_DEV_INFO            0xD0u
#define SPS30_SHDLC_CMD_READ_VERSION        0xD1u
#define SPS30_SHDLC_CMD_READ_STATUS         0xD2u
#define SPS30_SHDLC_CMD_RESET               0xD3u

/* I2C command pointer addresses */
#define SPS30_I2C_CMD_PTR_START_MEAS        0x0010u
#define SPS30_I2C_CMD_PTR_STOP_MEAS         0x0104u
#define SPS30_I2C_CMD_PTR_DATA_READY        0x0202u
#define SPS30_I2C_CMD_PTR_READ_MEAS         0x0300u
#define SPS30_I2C_CMD_PTR_SLEEP             0x1001u
#define SPS30_I2C_CMD_PTR_WAKEUP            0x1103u
#define SPS30_I2C_CMD_PTR_FAN_CLEAN         0x5607u
#define SPS30_I2C_CMD_PTR_AUTO_CLEAN        0x8004u
#define SPS30_I2C_CMD_PTR_PRODUCT_TYPE      0xD002u
#define SPS30_I2C_CMD_PTR_SERIAL_NUM        0xD033u
#define SPS30_I2C_CMD_PTR_READ_VERSION      0xD100u
#define SPS30_I2C_CMD_PTR_READ_STATUS       0xD206u
#define SPS30_I2C_CMD_PTR_CLEAR_STATUS      0xD210u
#define SPS30_I2C_CMD_PTR_RESET             0xD304u

/* -------------------------------------------------------------------------
 * Timing (ms)
 * Datasheet §1.1 / Table 7
 * ------------------------------------------------------------------------- */
/** Max SHDLC response time for most commands (datasheet: ≤20 ms). */
#define SPS30_T_RESPONSE_MS         20u

/**
 * Timeout for read_measurement UART responses.
 * The sensor produces a new value every ~1 s; allow generous margin for
 * the UART round-trip when polling at a slower rate.
 */
#define SPS30_T_READ_MEAS_MS        500u

/** Time to enter / leave Sleep-Mode (datasheet: ≤5 ms). */
#define SPS30_T_SLEEP_WAKEUP_MS     5u

/** Maximum window for sending the Wake-Up command after the 0xFF pulse. */
#define SPS30_T_WAKEUP_WINDOW_MS    100u

/** Soft-reset execution time (datasheet: ≤100 ms). */
#define SPS30_T_RESET_MS            100u

/* Misc */
#define SERIAL_NUMBER_MAX_LEN       32u
#define I2C_CRC8_POLYNOMIAL         0x31u
#define I2C_CRC8_INIT               0xFFu

#endif /* SPS30_DEFS_H */