#include "sps30.h"
#include "sps30_types.h"
#include "sps30_defs.h"

#include <stdint.h>
#include <stddef.h>
#include <string.h>

/* =========================================================================
 * Private helpers - CRC / checksum
 * ========================================================================= */

/* SHDLC checksum: ~(sum of bytes); built before stuffing, checked after */
static uint8_t _sps30_shdlc_checksum(const uint8_t *data, size_t len)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < len; i++) {
        sum = (uint8_t)(sum + data[i]);
    }
    return (uint8_t)(~sum);
}

/* I2C CRC-8 over a 2-byte word: poly 0x31, init 0xFF (datasheet 6.2) */
static uint8_t _sps30_i2c_crc8(const uint8_t data[2])
{
    uint8_t crc = I2C_CRC8_INIT;
    for (int i = 0; i < 2; i++) {
        crc ^= data[i];
        for (uint8_t bit = 8u; bit > 0u; --bit) {
            if (crc & 0x80u) {
                crc = (uint8_t)((crc << 1u) ^ I2C_CRC8_POLYNOMIAL);
            } else {
                crc = (uint8_t)(crc << 1u);
            }
        }
    }
    return crc;
}

/* =========================================================================
 * Private helpers - byte stuffing
 * ========================================================================= */

/* SHDLC-stuff one byte into out[]; returns 1 or 2 bytes written */
static size_t _sps30_stuff_byte(uint8_t byte, uint8_t *out)
{
    switch (byte) {
        case STUFF_7E_ORIG:
            out[0] = STUFF_ESC;
            out[1] = STUFF_7E_REPL;
            return 2u;
        case STUFF_ESC:
            out[0] = STUFF_ESC;
            out[1] = STUFF_7D_REPL;
            return 2u;
        case STUFF_11_ORIG:
            out[0] = STUFF_ESC;
            out[1] = STUFF_11_REPL;
            return 2u;
        case STUFF_13_ORIG:
            out[0] = STUFF_ESC;
            out[1] = STUFF_13_REPL;
            return 2u;
        default:
            out[0] = byte;
            return 1u;
    }
}

/* Reverse one SHDLC escape-replacement byte; 0xFF if unrecognised */
static uint8_t _sps30_unstuff_pair(uint8_t replacement)
{
    switch (replacement) {
        case STUFF_7E_REPL:
            return STUFF_7E_ORIG;
        case STUFF_7D_REPL:
            return STUFF_ESC;
        case STUFF_11_REPL:
            return STUFF_11_ORIG;
        case STUFF_13_REPL:
            return STUFF_13_ORIG;
        default:
            return 0xFFu;
    }
}

/* =========================================================================
 * Private helpers - SHDLC frame build / parse
 * ========================================================================= */

/*
 * Build a stuffed SHDLC MOSI frame: START | ADR CMD LEN DATA... CHK | STOP.
 * out must be >= SPS30_SHDLC_MAX_FRAME_LEN bytes.
 */
static SPS30_Status _sps30_shdlc_build_frame(uint8_t cmd, const uint8_t *tx_data, uint8_t tx_len, uint8_t *out, size_t out_size, size_t *out_len)
{
    if (out_size < SPS30_SHDLC_MAX_FRAME_LEN) {
        return SPS30_ERR_BUF_TOO_SMALL;
    }

    /* Un-stuffed content: ADR CMD LEN DATA... CHK */
    uint8_t raw[4u + SPS30_SHDLC_MAX_DATA_LEN + 1u];
    raw[0] = SHDLC_SLAVE_ADDR;
    raw[1] = cmd;
    raw[2] = tx_len;
    if (tx_len > 0u && tx_data != NULL) {
        memcpy(&raw[3u], tx_data, tx_len);
    }
    size_t content_len = 3u + (size_t)tx_len;
    raw[content_len] = _sps30_shdlc_checksum(raw, content_len);
    content_len++;  /* now includes CHK */

    /* Wrap with delimiters and apply stuffing */
    size_t pos = 0u;
    out[pos++] = SHDLC_START_STOP;
    for (size_t i = 0u; i < content_len; i++) {
        uint8_t tmp[2];
        size_t n = _sps30_stuff_byte(raw[i], tmp);
        out[pos] = tmp[0];
        if (n == 2u) {
            out[pos + 1u] = tmp[1];
        }
        pos += n;
    }
    out[pos++] = SHDLC_START_STOP;

    *out_len = pos;
    return SPS30_OK;
}

/*
 * Parse a raw SHDLC MISO frame: validate delimiters, de-stuff, verify
 * checksum, check exec-error, and copy out the payload.
 * Un-stuffed layout: ADR CMD STATE LEN DATA... CHK
 */
static SPS30_Status _sps30_shdlc_parse_frame(const uint8_t *frame, size_t frame_len, uint8_t *rx_data, size_t rx_max, uint8_t *rx_len)
{
    /* Minimum: START ADR CMD STATE LEN CHK STOP = 7 bytes */
    if (frame_len < 7u) {
        return SPS30_ERR_DATA_LEN;
    }
    if (frame[0] != SHDLC_START_STOP || frame[frame_len - 1u] != SHDLC_START_STOP) {
        return SPS30_ERR_CRC;
    }

    /* De-stuff everything between START and STOP */
    uint8_t unstuffed[4u + SPS30_SHDLC_MAX_DATA_LEN + 1u];
    size_t  ulen   = 0u;
    bool    in_esc = false;

    for (size_t i = 1u; i < frame_len - 1u; i++) {
        uint8_t b = frame[i];
        if (in_esc) {
            unstuffed[ulen++] = _sps30_unstuff_pair(b);
            in_esc = false;
        } else if (b == STUFF_ESC) {
            in_esc = true;
        } else {
            unstuffed[ulen++] = b;
        }
    }

    /* Minimum un-stuffed: ADR CMD STATE LEN CHK = 5 bytes */
    if (ulen < 5u) {
        return SPS30_ERR_DATA_LEN;
    }

    /* Checksum covers everything but CHK */
    uint8_t expected_chk = _sps30_shdlc_checksum(unstuffed, ulen - 1u);
    if (expected_chk != unstuffed[ulen - 1u]) {
        return SPS30_ERR_CRC;
    }

    /* State byte (index 2): bits 6..0 = execution error code */
    uint8_t exec_err = unstuffed[2] & 0x7Fu;
    if (exec_err != 0u) {
        switch (exec_err) {
            case 0x01u: return SPS30_ERR_DATA_LEN;
            case 0x02u: return SPS30_ERR_UNKNOWN_CMD;
            case 0x04u: return SPS30_ERR_INVALID_ARGS;
            case 0x43u: return SPS30_ERR_INVALID_STATE;
            default:    return SPS30_ERR_IO;
        }
    }

    /* unstuffed[3] = LEN, unstuffed[4..4+LEN-1] = payload */
    uint8_t data_len = unstuffed[3];

    /* Sanity: total length must equal ADR+CMD+STATE+LEN+data+CHK */
    if ((size_t)data_len + 5u != ulen) {
        return SPS30_ERR_DATA_LEN;
    }

    if (rx_len != NULL) {
        *rx_len = data_len;
    }
    if (data_len > 0u) {
        if (rx_data == NULL || (size_t)data_len > rx_max) {
            return SPS30_ERR_BUF_TOO_SMALL;
        }
        memcpy(rx_data, &unstuffed[4u], data_len);
    }

    return SPS30_OK;
}

/* =========================================================================
 * Private helpers - UART transaction
 * ========================================================================= */

/*
 * Send a SHDLC command and read back the response frame over UART.
 * Reads byte-by-byte until STOP (0x7E) or buffer full; timeout_ms applies
 * per byte (use SPS30_T_READ_MEAS_MS for measurement reads).
 */
static SPS30_Status _sps30_uart_transact(SPS30_Dev *dev, uint8_t cmd, const uint8_t *tx, uint8_t tx_len, uint8_t *rx, size_t rx_max, uint8_t *rx_len, uint32_t timeout_ms)
{
    uint8_t frame[SPS30_SHDLC_MAX_FRAME_LEN];
    size_t  flen = 0u;

    SPS30_Status s = _sps30_shdlc_build_frame(cmd, tx, tx_len, frame, sizeof(frame), &flen);
    if (s != SPS30_OK) {
        return s;
    }

    /* Flush stale RX bytes from a previous transaction so a leftover
     * frame isn't parsed as the reply to this command. */
    if (dev->hal.uart.flush_rx != NULL) {
        dev->hal.uart.flush_rx();
    }

    if (dev->hal.uart.write(frame, flen) != 0) {
        return SPS30_ERR_IO;
    }

    /* Read response byte-by-byte until STOP delimiter or buffer full */
    uint8_t resp[SPS30_SHDLC_MAX_FRAME_LEN];
    size_t  rpos    = 0u;
    bool    started = false;

    while (rpos < SPS30_SHDLC_MAX_FRAME_LEN) {
        uint8_t b;
        if (dev->hal.uart.read(&b, 1u, timeout_ms) != 0) {
            /* Timeout/IO error before START -> report timeout */
            return started ? SPS30_ERR_IO : SPS30_ERR_TIMEOUT;
        }

        if (!started) {
            if (b == SHDLC_START_STOP) {
                resp[rpos++] = b;
                started = true;
            }
            continue; /* discard stale bytes before START */
        }

        resp[rpos++] = b;

        /* Bare 0x7E after START is always the STOP byte (a stuffed 0x7E
         * appears as 0x7D 0x5E inside the frame) */
        if (b == SHDLC_START_STOP) {
            break;
        }
    }

    return _sps30_shdlc_parse_frame(resp, rpos, rx, rx_max, rx_len);
}

/* =========================================================================
 * Private helpers - I2C transactions
 * ========================================================================= */

/* Set Pointer transfer (16-bit address, no data) */
static SPS30_Status _sps30_i2c_set_pointer(SPS30_Dev *dev, uint16_t ptr)
{
    uint8_t buf[2] = { (uint8_t)(ptr >> 8u), (uint8_t)(ptr & 0xFFu) };
    if (dev->hal.i2c.write(SPS30_I2C_ADDR, buf, 2u) != 0) {
        return SPS30_ERR_IO;
    }
    return SPS30_OK;
}

/*
 * Set pointer then write data words, appending a CRC byte after each pair.
 * words[]/nwords are raw byte pairs (nwords must be even).
 */
static SPS30_Status _sps30_i2c_write_data(SPS30_Dev *dev, uint16_t ptr, const uint8_t *words, size_t nwords)
{
    /* 2 pointer bytes + up to 32 words x 3 bytes (data+crc) */
    uint8_t buf[2u + 32u * 3u];
    buf[0] = (uint8_t)(ptr >> 8u);
    buf[1] = (uint8_t)(ptr & 0xFFu);
    size_t pos = 2u;

    for (size_t i = 0u; i + 1u < nwords; i += 2u) {
        uint8_t pair[2] = { words[i], words[i + 1u] };
        buf[pos++] = pair[0];
        buf[pos++] = pair[1];
        buf[pos++] = _sps30_i2c_crc8(pair);
    }

    if (dev->hal.i2c.write(SPS30_I2C_ADDR, buf, pos) != 0) {
        return SPS30_ERR_IO;
    }
    return SPS30_OK;
}

/*
 * Set pointer then read raw_len bytes (groups of data0,data1,crc),
 * verify each CRC, and copy the 2 data bytes per group into out[].
 * raw_len must be a multiple of 3; out must hold raw_len*2/3 bytes.
 */
static SPS30_Status _sps30_i2c_read_data(SPS30_Dev *dev, uint16_t ptr, size_t raw_len, uint8_t *out, size_t out_len)
{
    SPS30_Status s = _sps30_i2c_set_pointer(dev, ptr);
    if (s != SPS30_OK) {
        return s;
    }

    uint8_t raw[128];
    if (raw_len > sizeof(raw) || out_len < (raw_len / 3u) * 2u) {
        return SPS30_ERR_BUF_TOO_SMALL;
    }
    if (dev->hal.i2c.read(SPS30_I2C_ADDR, raw, raw_len) != 0) {
        return SPS30_ERR_IO;
    }

    size_t out_pos = 0u;
    for (size_t i = 0u; i + 2u < raw_len; i += 3u) {
        uint8_t pair[2] = { raw[i], raw[i + 1u] };
        if (_sps30_i2c_crc8(pair) != raw[i + 2u]) {
            return SPS30_ERR_CRC;
        }
        out[out_pos++] = pair[0];
        out[out_pos++] = pair[1];
    }
    return SPS30_OK;
}

/* =========================================================================
 * Private helpers - measurement data parsing
 * ========================================================================= */

/* Big-endian bytes -> float */
static float _sps30_bytes_to_float(const uint8_t b[4])
{
    uint32_t u = ((uint32_t)b[0] << 24u) | ((uint32_t)b[1] << 16u) | ((uint32_t)b[2] << 8u) | (uint32_t)b[3];
    float f;
    memcpy(&f, &u, sizeof(f));
    return f;
}

/* Big-endian bytes -> uint16 */
static uint16_t _sps30_bytes_to_u16(const uint8_t b[2])
{
    return (uint16_t)(((uint16_t)b[0] << 8u) | b[1]);
}

/* Parse a 40-byte (float) or 20-byte (uint16) measurement payload */
static SPS30_Status _sps30_parse_measurement(const uint8_t *buf, size_t len, SPS30_OutputFormat fmt, SPS30_Data *data)
{
    if (fmt == SPS30_FORMAT_FLOAT) {
        if (len < 40u) {
            return SPS30_ERR_DATA_LEN;
        }
        data->pm1_0        = _sps30_bytes_to_float(&buf[ 0u]);
        data->pm2_5        = _sps30_bytes_to_float(&buf[ 4u]);
        data->pm4_0        = _sps30_bytes_to_float(&buf[ 8u]);
        data->pm10         = _sps30_bytes_to_float(&buf[12u]);
        data->nc0_5        = _sps30_bytes_to_float(&buf[16u]);
        data->nc1_0        = _sps30_bytes_to_float(&buf[20u]);
        data->nc2_5        = _sps30_bytes_to_float(&buf[24u]);
        data->nc4_0        = _sps30_bytes_to_float(&buf[28u]);
        data->nc10         = _sps30_bytes_to_float(&buf[32u]);
        data->typical_size = _sps30_bytes_to_float(&buf[36u]);
    } else {
        if (len < 20u) {
            return SPS30_ERR_DATA_LEN;
        }
        data->pm1_0        = (float)_sps30_bytes_to_u16(&buf[ 0u]);
        data->pm2_5        = (float)_sps30_bytes_to_u16(&buf[ 2u]);
        data->pm4_0        = (float)_sps30_bytes_to_u16(&buf[ 4u]);
        data->pm10         = (float)_sps30_bytes_to_u16(&buf[ 6u]);
        data->nc0_5        = (float)_sps30_bytes_to_u16(&buf[ 8u]);
        data->nc1_0        = (float)_sps30_bytes_to_u16(&buf[10u]);
        data->nc2_5        = (float)_sps30_bytes_to_u16(&buf[12u]);
        data->nc4_0        = (float)_sps30_bytes_to_u16(&buf[14u]);
        data->nc10         = (float)_sps30_bytes_to_u16(&buf[16u]);
        /* typical_size is in nm for uint16 mode; convert to um */
        data->typical_size = (float)_sps30_bytes_to_u16(&buf[18u]) / 1000.0f;
    }
    return SPS30_OK;
}

/* =========================================================================
 * Public API implementation
 * ========================================================================= */

SPS30_Status SPS30_Init(SPS30_Dev *dev)
{
    if (dev == NULL) {
        return SPS30_ERR_INVALID_ARGS;
    }
    if (dev->fmt != SPS30_FORMAT_FLOAT && dev->fmt != SPS30_FORMAT_UINT16) {
        return SPS30_ERR_INVALID_ARGS;
    }

    switch (dev->protocol) {
        case SPS30_PROTOCOL_I2C:
            if (!dev->hal.i2c.read || !dev->hal.i2c.write || !dev->hal.i2c.delay_ms) {
                return SPS30_ERR_INVALID_ARGS;
            }
            break;
        case SPS30_PROTOCOL_UART:
            if (!dev->hal.uart.read || !dev->hal.uart.write || !dev->hal.uart.delay_ms) {
                return SPS30_ERR_INVALID_ARGS;
            }
            break;
        default:
            return SPS30_ERR_INVALID_ARGS;
    }

    /* Force IDLE so init is safe even on a non-zero-initialised struct */
    dev->state = SPS30_STATE_IDLE;
    return SPS30_OK;
}

/* ------------------------------------------------------------------------- */

SPS30_Status SPS30_StartMeasurement(SPS30_Dev *dev)
{
    if (dev == NULL) {
        return SPS30_ERR_INVALID_ARGS;
    }
    if (dev->state != SPS30_STATE_IDLE) {
        return SPS30_ERR_INVALID_STATE;
    }

    SPS30_Status s;

    if (dev->protocol == SPS30_PROTOCOL_UART) {
        /* sub-command 0x01 (output format select) + format byte */
        uint8_t payload[2] = { 0x01u, (uint8_t)dev->fmt };
        s = _sps30_uart_transact(dev, SPS30_SHDLC_CMD_START_MEAS, payload, 2u, NULL, 0u, NULL, SPS30_T_RESPONSE_MS);
    } else {
        /* pointer (2B) + format word (2B) + CRC (1B) */
        uint8_t words[2] = { (uint8_t)dev->fmt, 0x00u };
        uint8_t buf[5];
        buf[0] = (uint8_t)(SPS30_I2C_CMD_PTR_START_MEAS >> 8u);
        buf[1] = (uint8_t)(SPS30_I2C_CMD_PTR_START_MEAS & 0xFFu);
        buf[2] = words[0];
        buf[3] = words[1];
        buf[4] = _sps30_i2c_crc8(words);
        s = (dev->hal.i2c.write(SPS30_I2C_ADDR, buf, 5u) == 0) ? SPS30_OK : SPS30_ERR_IO;
        dev->hal.i2c.delay_ms(SPS30_T_RESPONSE_MS);
    }

    if (s == SPS30_OK) {
        dev->state = SPS30_STATE_MEAS;
    }
    return s;
}

/* ------------------------------------------------------------------------- */

SPS30_Status SPS30_StopMeasurement(SPS30_Dev *dev)
{
    if (dev == NULL) {
        return SPS30_ERR_INVALID_ARGS;
    }
    if (dev->state != SPS30_STATE_MEAS) {
        return SPS30_ERR_INVALID_STATE;
    }

    SPS30_Status s;

    if (dev->protocol == SPS30_PROTOCOL_UART) {
        s = _sps30_uart_transact(dev, SPS30_SHDLC_CMD_STOP_MEAS, NULL, 0u, NULL, 0u, NULL, SPS30_T_RESPONSE_MS);
    } else {
        s = _sps30_i2c_set_pointer(dev, SPS30_I2C_CMD_PTR_STOP_MEAS);
        dev->hal.i2c.delay_ms(SPS30_T_RESPONSE_MS);
    }

    if (s == SPS30_OK) {
        dev->state = SPS30_STATE_IDLE;
    }
    return s;
}

/* ------------------------------------------------------------------------- */

SPS30_Status SPS30_ReadMeasurement(SPS30_Dev *dev, SPS30_Data *data)
{
    if (dev == NULL || data == NULL) {
        return SPS30_ERR_INVALID_ARGS;
    }
    if (dev->state != SPS30_STATE_MEAS) {
        return SPS30_ERR_INVALID_STATE;
    }

    SPS30_Status s;

    if (dev->protocol == SPS30_PROTOCOL_UART) {
        uint8_t rx[40];
        uint8_t rx_len = 0u;
        /* Longer timeout: caller polls ~1s apart, sample may not be ready yet */
        s = _sps30_uart_transact(dev, SPS30_SHDLC_CMD_READ_MEAS, NULL, 0u, rx, sizeof(rx), &rx_len, SPS30_T_READ_MEAS_MS);
        if (s != SPS30_OK) {
            return s;
        }
        /* Empty payload means no new data yet */
        if (rx_len == 0u) {
            return SPS30_ERR_NO_DATA;
        }
        return _sps30_parse_measurement(rx, rx_len, dev->fmt, data);

    } else {
        /* Check Data-Ready Flag first */
        uint8_t flag_raw[3];
        s = _sps30_i2c_set_pointer(dev, SPS30_I2C_CMD_PTR_DATA_READY);
        if (s != SPS30_OK) {
            return s;
        }
        if (dev->hal.i2c.read(SPS30_I2C_ADDR, flag_raw, 3u) != 0) {
            return SPS30_ERR_IO;
        }
        {
            uint8_t pair[2] = { flag_raw[0], flag_raw[1] };
            if (_sps30_i2c_crc8(pair) != flag_raw[2]) {
                return SPS30_ERR_CRC;
            }
            if (pair[1] != 0x01u) {
                return SPS30_ERR_NO_DATA;
            }
        }

        /* float: 20 words x 3 raw bytes = 60; uint16: 10 words x 3 = 30 */
        size_t raw_len   = (dev->fmt == SPS30_FORMAT_FLOAT) ? 60u : 30u;
        size_t data_size = (dev->fmt == SPS30_FORMAT_FLOAT) ? 40u : 20u;
        uint8_t parsed[40];

        s = _sps30_i2c_read_data(dev, SPS30_I2C_CMD_PTR_READ_MEAS, raw_len, parsed, sizeof(parsed));
        if (s != SPS30_OK) {
            return s;
        }
        return _sps30_parse_measurement(parsed, data_size, dev->fmt, data);
    }
}

/* ------------------------------------------------------------------------- */

SPS30_Status SPS30_Sleep(SPS30_Dev *dev)
{
    if (dev == NULL) {
        return SPS30_ERR_INVALID_ARGS;
    }
    if (dev->state != SPS30_STATE_IDLE) {
        return SPS30_ERR_INVALID_STATE;
    }

    SPS30_Status s;

    if (dev->protocol == SPS30_PROTOCOL_UART) {
        s = _sps30_uart_transact(dev, SPS30_SHDLC_CMD_SLEEP, NULL, 0u, NULL, 0u, NULL, SPS30_T_RESPONSE_MS);
    } else {
        s = _sps30_i2c_set_pointer(dev, SPS30_I2C_CMD_PTR_SLEEP);
        dev->hal.i2c.delay_ms(SPS30_T_SLEEP_WAKEUP_MS);
    }

    if (s == SPS30_OK) {
        dev->state = SPS30_STATE_SLEEP;
    }
    return s;
}

/* ------------------------------------------------------------------------- */

SPS30_Status SPS30_WakeUp(SPS30_Dev *dev)
{
    if (dev == NULL) {
        return SPS30_ERR_INVALID_ARGS;
    }
    if (dev->state != SPS30_STATE_SLEEP) {
        return SPS30_ERR_INVALID_STATE;
    }

    SPS30_Status s;

    if (dev->protocol == SPS30_PROTOCOL_UART) {
        /* Send 0xFF to re-enable UART, then Wake-Up within 100ms */
        uint8_t pulse = 0xFFu;
        if (dev->hal.uart.write(&pulse, 1u) != 0) {
            return SPS30_ERR_IO;
        }
        s = _sps30_uart_transact(dev, SPS30_SHDLC_CMD_WAKEUP, NULL, 0u, NULL, 0u, NULL, SPS30_T_SLEEP_WAKEUP_MS);
    } else {
        /* Bare Start+Stop pulse to re-enable I2C, then Wake-Up pointer.
         * Zero-length write may be unsupported; fall back to just the
         * Wake-Up command if so. */
        (void)dev->hal.i2c.write(SPS30_I2C_ADDR, NULL, 0u);
        dev->hal.i2c.delay_ms(2u);
        s = _sps30_i2c_set_pointer(dev, SPS30_I2C_CMD_PTR_WAKEUP);
        dev->hal.i2c.delay_ms(SPS30_T_SLEEP_WAKEUP_MS);
    }

    if (s == SPS30_OK) {
        dev->state = SPS30_STATE_IDLE;
    }
    return s;
}

/* ------------------------------------------------------------------------- */

SPS30_Status SPS30_StartFanCleaning(SPS30_Dev *dev)
{
    if (dev == NULL) {
        return SPS30_ERR_INVALID_ARGS;
    }
    if (dev->state != SPS30_STATE_MEAS) {
        return SPS30_ERR_INVALID_STATE;
    }

    if (dev->protocol == SPS30_PROTOCOL_UART) {
        return _sps30_uart_transact(dev, SPS30_SHDLC_CMD_FAN_CLEAN, NULL, 0u, NULL, 0u, NULL, SPS30_T_RESPONSE_MS);
    } else {
        SPS30_Status s = _sps30_i2c_set_pointer(dev, SPS30_I2C_CMD_PTR_FAN_CLEAN);
        dev->hal.i2c.delay_ms(SPS30_T_SLEEP_WAKEUP_MS);
        return s;
    }
}

/* ------------------------------------------------------------------------- */

SPS30_Status SPS30_ReadAutoCleaningInterval(SPS30_Dev *dev, uint32_t *interval)
{
    if (dev == NULL || interval == NULL) {
        return SPS30_ERR_INVALID_ARGS;
    }

    SPS30_Status s;
    uint8_t data[4];

    if (dev->protocol == SPS30_PROTOCOL_UART) {
        uint8_t subcmd = 0x00u;
        uint8_t rx_len = 0u;
        s = _sps30_uart_transact(dev, SPS30_SHDLC_CMD_AUTO_CLEAN_INTERVAL, &subcmd, 1u, data, sizeof(data), &rx_len, SPS30_T_RESPONSE_MS);
        if (s != SPS30_OK) {
            return s;
        }
        if (rx_len < 4u) {
            return SPS30_ERR_DATA_LEN;
        }
    } else {
        s = _sps30_i2c_read_data(dev, SPS30_I2C_CMD_PTR_AUTO_CLEAN, 6u, data, sizeof(data));
        if (s != SPS30_OK) {
            return s;
        }
    }

    *interval = ((uint32_t)data[0] << 24u) | ((uint32_t)data[1] << 16u) | ((uint32_t)data[2] << 8u) | (uint32_t)data[3];
    return SPS30_OK;
}

/* ------------------------------------------------------------------------- */

SPS30_Status SPS30_WriteAutoCleaningInterval(SPS30_Dev *dev, uint32_t interval)
{
    if (dev == NULL) {
        return SPS30_ERR_INVALID_ARGS;
    }

    uint8_t iv[4] = {
        (uint8_t)(interval >> 24u),
        (uint8_t)(interval >> 16u),
        (uint8_t)(interval >> 8u),
        (uint8_t)(interval)
    };

    if (dev->protocol == SPS30_PROTOCOL_UART) {
        uint8_t payload[5] = { 0x00u, iv[0], iv[1], iv[2], iv[3] };
        return _sps30_uart_transact(dev, SPS30_SHDLC_CMD_AUTO_CLEAN_INTERVAL, payload, 5u, NULL, 0u, NULL, SPS30_T_RESPONSE_MS);
    } else {
        return _sps30_i2c_write_data(dev, SPS30_I2C_CMD_PTR_AUTO_CLEAN, iv, 4u);
    }
}

/* ------------------------------------------------------------------------- */

SPS30_Status SPS30_ReadDeviceInfo(SPS30_Dev *dev, SPS30_DeviceInfo *info)
{
    if (dev == NULL || info == NULL) {
        return SPS30_ERR_INVALID_ARGS;
    }

    SPS30_Status s;

    if (dev->protocol == SPS30_PROTOCOL_UART) {
        uint8_t rx[32];
        uint8_t rx_len = 0u;

        /* Product Type (sub-command 0x00) */
        uint8_t subcmd = 0x00u;
        s = _sps30_uart_transact(dev, SPS30_SHDLC_CMD_DEV_INFO, &subcmd, 1u, rx, sizeof(rx), &rx_len, SPS30_T_RESPONSE_MS);
        if (s != SPS30_OK) {
            return s;
        }
        memcpy(info->product_type, rx, rx_len);
        info->product_type[rx_len < 31u ? rx_len : 31u] = '\0';

        /* Serial Number (sub-command 0x03) */
        subcmd = 0x03u;
        s = _sps30_uart_transact(dev, SPS30_SHDLC_CMD_DEV_INFO, &subcmd, 1u, rx, sizeof(rx), &rx_len, SPS30_T_RESPONSE_MS);
        if (s != SPS30_OK) {
            return s;
        }
        memcpy(info->serial_number, rx, rx_len);
        info->serial_number[rx_len < 31u ? rx_len : 31u] = '\0';

    } else {
        uint8_t data[32];

        /* Product Type: 16 words -> 48 raw bytes -> 32 data bytes */
        s = _sps30_i2c_read_data(dev, SPS30_I2C_CMD_PTR_PRODUCT_TYPE, 48u, data, sizeof(data));
        if (s != SPS30_OK) {
            return s;
        }
        memcpy(info->product_type, data, 31u);
        info->product_type[31u] = '\0';

        /* Serial Number: same layout */
        s = _sps30_i2c_read_data(dev, SPS30_I2C_CMD_PTR_SERIAL_NUM, 48u, data, sizeof(data));
        if (s != SPS30_OK) {
            return s;
        }
        memcpy(info->serial_number, data, 31u);
        info->serial_number[31u] = '\0';
    }

    return SPS30_OK;
}

/* ------------------------------------------------------------------------- */

SPS30_Status SPS30_ReadVersion(SPS30_Dev *dev, SPS30_Version *version)
{
    if (dev == NULL || version == NULL) {
        return SPS30_ERR_INVALID_ARGS;
    }

    memset(version, 0, sizeof(*version));
    SPS30_Status s;

    if (dev->protocol == SPS30_PROTOCOL_UART) {
        uint8_t rx[7];
        uint8_t rx_len = 0u;
        s = _sps30_uart_transact(dev, SPS30_SHDLC_CMD_READ_VERSION, NULL, 0u, rx, sizeof(rx), &rx_len, SPS30_T_RESPONSE_MS);
        if (s != SPS30_OK) {
            return s;
        }
        if (rx_len < 7u) {
            return SPS30_ERR_DATA_LEN;
        }
        version->fw_major    = rx[0];
        version->fw_minor    = rx[1];
        /* rx[2] reserved */
        version->hw_rev      = rx[3];
        /* rx[4] reserved */
        version->shdlc_major = rx[5];
        version->shdlc_minor = rx[6];
    } else {
        uint8_t data[2];
        s = _sps30_i2c_read_data(dev, SPS30_I2C_CMD_PTR_READ_VERSION, 3u, data, sizeof(data));
        if (s != SPS30_OK) {
            return s;
        }
        version->fw_major = data[0];
        version->fw_minor = data[1];
    }

    return SPS30_OK;
}

/* ------------------------------------------------------------------------- */

SPS30_Status SPS30_ReadDeviceStatus(SPS30_Dev *dev, uint32_t *status, bool clear)
{
    if (dev == NULL || status == NULL) {
        return SPS30_ERR_INVALID_ARGS;
    }

    SPS30_Status s;
    uint8_t data[4];

    if (dev->protocol == SPS30_PROTOCOL_UART) {
        uint8_t subcmd = clear ? 0x01u : 0x00u;
        uint8_t rx[5]; /* 4 bytes status + 1 reserved */
        uint8_t rx_len = 0u;
        s = _sps30_uart_transact(dev, SPS30_SHDLC_CMD_READ_STATUS, &subcmd, 1u, rx, sizeof(rx), &rx_len, SPS30_T_RESPONSE_MS);
        if (s != SPS30_OK) {
            return s;
        }
        if (rx_len < 4u) {
            return SPS30_ERR_DATA_LEN;
        }
        memcpy(data, rx, 4u);
    } else {
        /* 32-bit status = 2 words = 6 raw bytes */
        s = _sps30_i2c_read_data(dev, SPS30_I2C_CMD_PTR_READ_STATUS, 6u, data, sizeof(data));
        if (s != SPS30_OK) {
            return s;
        }
        if (clear) {
            /* Best-effort clear; don't override the status read result */
            (void)_sps30_i2c_set_pointer(dev, SPS30_I2C_CMD_PTR_CLEAR_STATUS);
            dev->hal.i2c.delay_ms(SPS30_T_SLEEP_WAKEUP_MS);
        }
    }

    *status = ((uint32_t)data[0] << 24u) | ((uint32_t)data[1] << 16u) | ((uint32_t)data[2] << 8u) | (uint32_t)data[3];
    return SPS30_OK;
}

/* ------------------------------------------------------------------------- */

SPS30_Status SPS30_Reset(SPS30_Dev *dev)
{
    if (dev == NULL) {
        return SPS30_ERR_INVALID_ARGS;
    }

    /* Wake up first if sleeping so the interface accepts commands */
    if (dev->state == SPS30_STATE_SLEEP) {
        SPS30_Status s = SPS30_WakeUp(dev);
        if (s != SPS30_OK) {
            return s;
        }
    }

    SPS30_Status s;

    if (dev->protocol == SPS30_PROTOCOL_UART) {
        s = _sps30_uart_transact(dev, SPS30_SHDLC_CMD_RESET, NULL, 0u, NULL, 0u, NULL, SPS30_T_RESET_MS);
    } else {
        s = _sps30_i2c_set_pointer(dev, SPS30_I2C_CMD_PTR_RESET);
        dev->hal.i2c.delay_ms(SPS30_T_RESET_MS);
    }

    if (s == SPS30_OK) {
        dev->state = SPS30_STATE_IDLE;
    }
    return s;
}