# bare-drivers

Lightweight, platform-independent drivers for embedded peripherals.
Designed to work on any microcontroller without vendor lock-in.

---

## Drivers

| Driver              | Device    | Protocol | Description                      |
| ------------------- | --------- | -------- | -------------------------------- |
| [bh1750](bh1750/)   | BH1750FVI | I2C      | Ambient light sensor             |
| [sht3x](sht3x/)     | SHT30/31/35 | I2C    | Temperature & humidity sensor    |
| [scd4x](scd4x/)     | SCD40/41  | I2C      | CO₂, temperature & humidity sensor |

---

## Repository Structure

```
bare-drivers/
├── bh1750/
│   ├── bh1750.c
│   ├── include/
│   │   └── bh1750.h
│   └── README.md
├── sht3x/
│   ├── sht3x.c
│   ├── include/
│   │   ├── sht3x.h
│   │   └── sht3x_defs.h
│   └── README.md
├── scd4x/
│   ├── scd4x.c
│   ├── include/
│   │   └── scd4x.h
│   └── README.md
└── README.md
```

---

## Design Philosophy

Each driver:
- Has no dependency on any HAL or SDK
- Uses function pointers for I2C/SPI/UART abstraction
- Is fully standalone and portable
- Brings its own platform layer per target

---

## Usage

1. Copy the driver into your project
2. Implement the platform layer
3. Initialize and use

See each driver's README for detailed instructions.

---

## Get a Driver

If you only need one or a few drivers, use Git sparse-checkout to avoid downloading the entire repository:

```bash
git clone --filter=blob:none --no-checkout https://github.com/hphuc15/bare_drivers
cd bare_drivers
git sparse-checkout init --cone

# Single driver
git sparse-checkout set <sensor> && git checkout

# Multiple drivers
git sparse-checkout set <sensor_1> <sensor_2> ... && git checkout
```

**Examples:**
```bash
git sparse-checkout set sht3x && git checkout
git sparse-checkout set sht3x scd4x && git checkout
```

Only the selected folders will be downloaded.

---

## Notes

- Drivers are independent → safe to use individually
- No shared core or hidden dependencies
- Suitable for bare-metal, RTOS, or any embedded platform

---

## License

MIT
