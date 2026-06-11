/**
 * @file  bh1750.h
 * @brief BH1750FVI ambient light sensor driver
 */
#ifndef BH1750_H
#define BH1750_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* I2C slave address (7-bit), determined by ADDR pin */
#define BH1750_I2C_ADDR_LOW 0x23  /* ADDR = GND */
#define BH1750_I2C_ADDR_HIGH 0x5C /* ADDR = VCC */

/* Opcodes */
#define BH1750_CMD_POWER_DOWN 0x00
#define BH1750_CMD_POWER_ON 0x01
#define BH1750_CMD_RESET 0x07 /* Clears data register, requires Power On state */

/* MTreg (Measurement Time register) range */
#define BH1750_MTREG_MIN     31U  /* Sensitivity = Default * 0.45 */
#define BH1750_MTREG_DEFAULT 69U  /* Factory default                */
#define BH1750_MTREG_MAX     254U /* Sensitivity = Default * 3.68  */

/** Measurement mode */
typedef enum
{
    BH1750_MODE_CONT_H_RES = 0x10,  /* Continuous, 1 lx resolution,   ~180ms */
    BH1750_MODE_CONT_H_RES2 = 0x11, /* Continuous, 0.5 lx resolution, ~180ms */
    BH1750_MODE_CONT_L_RES = 0x13,  /* Continuous, 4 lx resolution,   ~24ms  */
    BH1750_MODE_ONE_H_RES = 0x20,   /* One-time,   1 lx resolution,   ~180ms */
    BH1750_MODE_ONE_H_RES2 = 0x21,  /* One-time,   0.5 lx resolution, ~180ms */
    BH1750_MODE_ONE_L_RES = 0x23,   /* One-time,   4 lx resolution,   ~24ms  */
} BH1750_Mode;

/** Return status */
typedef enum
{
    BH1750_OK = 0,
    BH1750_ERR_NULL = -1,      /* NULL pointer passed         */
    BH1750_ERR_I2C_WRITE = -2, /* I2C write failed            */
    BH1750_ERR_I2C_READ = -3,  /* I2C read failed             */
} BH1750_Status;

/**
 * @brief Write len bytes to device; return 0 on success
 * @note  Pass 7-bit address; HAL must handle R/W bit shift if required (e.g. addr << 1 for STM32)
 */
typedef int (*bh1750_i2c_write_fn)(uint8_t addr, const uint8_t *data, size_t len);

/**
 * @brief Read len bytes from device; return 0 on success
 */
typedef int (*bh1750_i2c_read_fn)(uint8_t addr, uint8_t *data, size_t len);

/**
 * @brief Millisecond delay
 */
typedef void (*bh1750_delay_ms_fn)(uint32_t ms);

/**
 * @brief Device handle
 * @note  Populate i2c_write, i2c_read, delay_ms, and address before calling BH1750_Init()
 * @note  mode is set internally by BH1750_Init(), do not modify manually
 */
typedef struct
{
    uint8_t i2c_addr;              /* 7-bit I2C address: BH1750_I2C_ADDR_LOW or BH1750_I2C_ADDR_HIGH */
    BH1750_Mode mode;              /* BH1750 measurement mode */
    uint8_t mtreg;                  /* Measurement Time register; set to BH1750_MTREG_DEFAULT (69)
                                       by BH1750_Init(), updated by BH1750_SetMTreg() */
    bh1750_i2c_write_fn i2c_write; /* Write len bytes to device; return 0 on success */
    bh1750_i2c_read_fn i2c_read;   /* Read  len bytes from device; return 0 on success */
    bh1750_delay_ms_fn delay_ms;   /* Millisecond delay                                */
} BH1750_Dev;

BH1750_Status BH1750_Init(BH1750_Dev *dev, BH1750_Mode mode);
BH1750_Status BH1750_SetMTreg(BH1750_Dev *dev, uint8_t mtreg);
BH1750_Status BH1750_ReadLux(BH1750_Dev *dev, float *lux);
BH1750_Status BH1750_PowerDown(BH1750_Dev *dev);

#endif /* BH1750_H */

#ifdef __cplusplus
}
#endif