# SPS30 Driver - Examples

Portable driver for the Sensirion SPS30 particulate matter sensor.
Supports UART (SHDLC) and I2C protocols via HAL function pointer injection.

---

## Examples

### `sps30_eg_uart.c` - UART / SHDLC

Full usage demo over UART. Covers init, device info, firmware version,
soft-reset (handles stale state after ESP32 reboot without power cycle),
start measurement, and a 2 s poll loop with periodic status register check.

**Wiring**

```
SPS30 Pin 1  VDD → 5 V
SPS30 Pin 2  RX  → GPIO 17   (ESP32 TX)
SPS30 Pin 3  TX  → GPIO 16   (ESP32 RX)
SPS30 Pin 4  SEL → Float     (selects UART mode; GND = I2C)
SPS30 Pin 5  GND → GND
```

**Sample output**

```
I (282)  [SPS30_EG]: UART ready (port=1 TX=17 RX=16 baud=115200)
I (292)  [SPS30_EG]: Type: 00080000  S/N: 8B9DE87F7883D863
I (292)  [SPS30_EG]: FW v2.2  HW rev 7
I (1392) [SPS30_EG]: Measuring (poll every 2000 ms)...
I (3392) [SPS30_EG]: --------------------------------------------
I (3392) [SPS30_EG]:   Mass [ug/m3]
I (3392) [SPS30_EG]:     PM1.0 =    8.45    PM2.5 =   10.31
I (3402) [SPS30_EG]:     PM4.0 =   11.43    PM10  =   11.66
I (3402) [SPS30_EG]:   Number [#/cm3]
I (3402) [SPS30_EG]:     NC0.5 =   54.75    NC1.0 =   65.61
I (3412) [SPS30_EG]:     NC2.5 =   67.33    NC4.0 =   67.63
I (3412) [SPS30_EG]:     NC10  =   67.69
I (3422) [SPS30_EG]:   Typical size : 0.669 um
I (3422) [SPS30_EG]:   AQI (PM2.5)  : Good
I (3432) [SPS30_EG]: --------------------------------------------
```

---

### `sps30_eg_i2c.c` - I2C *(coming soon)*

Same coverage as the UART example, using the I2C protocol path.
Pull SEL (Pin 4) to GND to put the sensor into I2C mode.

---

## Build

Standard ESP-IDF component. Add `sps30` to `REQUIRES` in your
`CMakeLists.txt` and set the target:

```bash
idf.py set-target esp32
idf.py build flash monitor
```