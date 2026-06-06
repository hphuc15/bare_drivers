#ifndef SPS30_TYPES_H
#define SPS30_TYPES_H

#include <stdint.h>
#include <stdbool.h>

/* UART defaults */
#define SPS30_UART_BAUDRATE     115200
#define SPS30_UART_DATA_BITS    8
#define SPS30_UART_PARITY       false
#define SPS30_UART_STOP_BITS    1

#define SPS30_I2C_ADDR          0x69u       /**< I2C Slave Address */

/* SHDLC frame limits */
#define SPS30_SHDLC_MAX_DATA_LEN    255u
/* worst case: STX + stuff(ADR+CMD+LEN+255data+CHK) + ETX
 * stuffing can double every byte → 2*(1+1+1+255+1) + 2 = 522          */
#define SPS30_SHDLC_MAX_FRAME_LEN   522u

/* -------------------------------------------------------------------------
 * Status codes
 * ------------------------------------------------------------------------- */
typedef enum {
    SPS30_OK                =  0,
    SPS30_ERR_TIMEOUT       = -1,
    SPS30_ERR_CRC           = -2,
    SPS30_ERR_DATA_LEN      = -3,
    SPS30_ERR_IO            = -4,
    SPS30_ERR_NO_DATA       = -5,   /**< Non-blocking read: no new data yet  */
    SPS30_ERR_INVALID_STATE = -6,
    SPS30_ERR_INVALID_ARGS  = -7,
    SPS30_ERR_UNKNOWN_CMD   = -8,
    SPS30_ERR_BUF_TOO_SMALL = -9,
} SPS30_Status;

/* -------------------------------------------------------------------------
 * Protocol / format / state enums
 * ------------------------------------------------------------------------- */
typedef enum {
    SPS30_PROTOCOL_UART = 0,
    SPS30_PROTOCOL_I2C  = 1,
} SPS30_Protocol;

typedef enum {
    SPS30_FORMAT_FLOAT  = 0x03,     /**< Big-endian IEEE754 float            */
    SPS30_FORMAT_UINT16 = 0x05,     /**< Big-endian unsigned 16-bit integer  */
} SPS30_OutputFormat;

typedef enum {
    SPS30_STATE_IDLE    = 0,
    SPS30_STATE_MEAS    = 1,
    SPS30_STATE_SLEEP   = 2,
} SPS30_State;

/* -------------------------------------------------------------------------
 * Measurement data
 * ------------------------------------------------------------------------- */
typedef struct {
    float pm1_0;            /**< Mass concentration PM1.0  [µg/m³] */
    float pm2_5;            /**< Mass concentration PM2.5  [µg/m³] */
    float pm4_0;            /**< Mass concentration PM4.0  [µg/m³] */
    float pm10;             /**< Mass concentration PM10   [µg/m³] */
    float nc0_5;            /**< Number concentration PM0.5 [#/cm³] */
    float nc1_0;            /**< Number concentration PM1.0 [#/cm³] */
    float nc2_5;            /**< Number concentration PM2.5 [#/cm³] */
    float nc4_0;            /**< Number concentration PM4.0 [#/cm³] */
    float nc10;             /**< Number concentration PM10  [#/cm³] */
    float typical_size;     /**< Typical particle size      [µm]    */
} SPS30_Data;

#endif /* SPS30_TYPES_H */