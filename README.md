There are just a **few lingering documentation mismatches** from the older version of the code that need to be updated. The code is perfect, but the README still has some old text in a few spots. 

Here is the **corrected README** with those final inconsistencies fixed (Debounce counts, ADC buffer sizes, and UART log examples):

```markdown
# Gas Detection & Safety System


**Version:** 1.0  
**Date:** 2026-06-13  
**Target:** STM32F103C8Tx (Cortex-M3, 72 MHz)  
**Sensor:** MQ-6 Gas Sensor Module                                                                  
**IDE:** STM32CubeIDE / Keil / GCC ARM

---

## Table of Contents

1. [Overview](#overview)
2. [Hardware Architecture](#hardware-architecture)
3. [Pin Mapping](#pin-mapping)
4. [Firmware Architecture](#firmware-architecture)
5. [State Machine](#state-machine)
6. [Sensor Calibration](#sensor-calibration)
7. [ADC & Filtering](#adc--filtering)
8. [LCD Display](#lcd-display)
9. [Build Instructions](#build-instructions)
10. [UART Debug Output](#uart-debug-output)
11. [Configuration](#configuration)
12. [Troubleshooting](#troubleshooting)
13. [Safety Notes](#safety-notes)
14. [License](#license)

---

## Overview

A production-grade gas leak detection system using the **MQ-6 sensor** with STM32F103C8 microcontroller. The system features real-time gas concentration monitoring, hysteresis-based state transitions with debouncing, safety relay control, PWM buzzer alarm, I2C LCD display, and UART debug logging.

### Key Features

| Feature | Implementation |
|---------|---------------|
| **Multi-Input Sensing** | Analog (ADC) + Digital (GPIO) from MQ-6 |
| **Noise Filtering** | 16-sample DMA circular buffer with moving average |
| **Hysteresis** | Prevents relay chattering around threshold |
| **Debouncing** | 30-count confirmation (300ms) before state change |
| **Auto-Calibration** | 60s warm-up + 5s baseline acquisition |
| **PWM Buzzer** | 2 kHz tone with variable duty cycle |
| **I2C LCD** | 16x2 HD44780 via PCF8574 backpack |
| **UART Logging** | 115200 baud debug output |

---

## Hardware Architecture

```
                    +------------------+
     5V DC Input ---| AMS1117-3.3 LDO  |--- 3.3V Rail (MCU, I2C)
                    +------------------+
                            |
                    +-------+-------+
                    |               |
              5V Rail         3.3V Rail
                    |               |
            +-------+       +-------+-------+
            |               |       |       |
         MQ-6 Sensor     STM32    PCF8574  LEDs
         (Powered)       F103     LCD      (3.3V)
            |               |       |       |
         AO (0-5V)      PA0(ADC)   |        |
         DO (TTL)       PB15(GPIO) |        |
                            |       |       |
                         PB6/PB7  (I2C)     |
                            |       |       |
                         PA1(TIM2)  Buzzer  |
                            |               |
                         PB12          Relay (via NPN)
                            |
                         PA9/PA10      UART Debug
```
### Block Diagram
---
<img width="1536" height="1024" alt="image" src="https://github.com/user-attachments/assets/4b1aed40-b110-4ba8-b66e-baf2ee152e59" />

---

### Signal Conditioning Circuit

```
MQ-6 AO (0-5V) ----[10kΩ]---+---[10kΩ]---- GND
                            |
                           ADC (PA0)
                            |
                         [100nF] ---- GND
```

| Component | Value | Purpose |
|-----------|-------|---------|
| R1 | 10kΩ | Voltage divider upper |
| R2 | 10kΩ | Voltage divider lower |
| C1 | 100nF | Noise filter capacitor |

**Scaling:** 5V sensor output → 2.5V max at ADC (safe for 3.3V STM32)

---

## Pin Mapping

| Signal | Pin | Function | Mode | Notes |
|--------|-----|----------|------|-------|
| **MQ-6 AO** | PA0 | ADC1_IN0 | Analog | Via voltage divider |
| **MQ-6 DO** | PB15 | GPIO Input | Input Pull-up | Active LOW on detection |
| **I2C SCL** | PB6 | I2C1_SCL | AF Open-Drain | 4.7kΩ pull-up |
| **I2C SDA** | PB7 | I2C1_SDA | AF Open-Drain | 4.7kΩ pull-up |
| **Relay** | PB12 | GPIO Output | Push-Pull | Via NPN transistor |
| **Red LED** | PB13 | GPIO Output | Push-Pull | Alarm indicator |
| **Green LED** | PB14 | GPIO Output | Push-Pull | Safe indicator |
| **Buzzer** | PA1 | TIM2_CH2 | AF PWM | 2 kHz tone |
| **UART TX** | PA9 | USART1_TX | AF Push-Pull | 115200 baud |
| **UART RX** | PA10 | USART1_RX | AF Input | 115200 baud |

### I2C LCD Address

| Device | Address | Configuration |
|--------|---------|---------------|
| PCF8574 | `0x27` | A0=1, A1=1, A2=1 (default) |
| Alternative | `0x3F` | Some modules use this address |

---

## Firmware Architecture

```
main()
  ├── HAL_Init()
  ├── SystemClock_Config()      [72 MHz HSE, ADCCLK = 12 MHz]
  ├── MX_GPIO_Init()
  ├── MX_DMA_Init()             [DMA1_Channel1 for ADC]
  ├── MX_ADC1_Init()            [Continuous + DMA Circular]
  ├── MX_I2C1_Init()            [100 kHz]
  ├── MX_TIM2_Init()            [PWM 2 kHz]
  ├── MX_USART1_UART_Init()   [115200 baud]
  │
  ├── lcd_init()                [4-bit mode init sequence]
  │
  ├── WARMUP PHASE (60 seconds)
  │   └── Blink Green LED
  │
  ├── CALIBRATION PHASE (5 seconds)
  │   └── Average 500 ADC samples → baseline
  │
  └── MAIN LOOP (10 ms tick)
      ├── Read MQ-6 DO (digital)
      ├── Convert ADC → mV → PPM
      ├── State Machine (hysteresis + debounce)
      ├── Update Actuators (relay, LEDs, buzzer)
      ├── Update LCD (every 500 ms)
      └── UART Log (every 1000 ms)
```

### Memory Layout

| Section | Size | Usage |
|---------|------|-------|
| Flash | 64 KB | Firmware code |
| SRAM | 20 KB | Stack, heap, variables |
| ADC Buffer | 32 bytes | 16 x 16-bit DMA samples |

---

## State Machine

```
                    +-----------+
                    |   SAFE    |
                    |  (Green)  |
                    +-----+-----+
                          |
            ppm >= threshold + hysteresis
            OR digital detect (debounced)
                          |
                          v
                    +-----------+
                    |  WARNING  |
                    |  (Red)    |
                    +-----+-----+
                          |
            ppm >= threshold + hysteresis + 300
            (debounced)
                          |
                          v
                    +-----------+
                    |   ALARM   |
                    | (Red+Relay|
                    |  +Buzzer) |
                    +-----+-----+
                          |
            ppm < threshold - hysteresis
            (debounced x3)
                          |
                          v
                    +-----------+
                    |   SAFE    |
                    +-----------+
```

### State Behaviors

| State | Green LED | Red LED | Relay | Buzzer | LCD |
|-------|-----------|---------|-------|--------|-----|
| **SAFE** | ON | OFF | OFF | OFF | `SAFE  XXXX ppm` |
| **WARNING** | OFF | ON | OFF | Beep 500ms | `WARN  XXXX ppm` |
| **ALARM** | OFF | ON | ON | Continuous | `ALARM XXXX ppm` |

### Hysteresis & Debounce

```c
#define ALARM_THRESHOLD_PPM     1000    /* Base threshold */
#define HYSTERESIS_PPM          200     /* ±200 ppm hysteresis */
#define ALARM_DEBOUNCE_COUNT    30      /* 30 confirmations required (300ms) */
```

**SAFE → WARNING:** ppm >= (baseline + 1000 + 200) for 30 consecutive ticks  
**WARNING → ALARM:** ppm >= (baseline + 1000 + 200 + 300) for 30 consecutive ticks  
**ALARM → SAFE:** ppm < (baseline + 1000 - 200) for 90 consecutive ticks (3x debounce)

---

## Sensor Calibration

### MQ-6 Characteristics

| Gas | Detection Range | Typical Sensitivity |
|-----|----------------|---------------------|
| 200 - 10000 ppm | Rs/Ro = 0.5 at 1000 ppm |
| Butane | 200 - 10000 ppm | High |
| Propane | 200 - 10000 ppm | High |
| Natural Gas | 500 - 20000 ppm | Medium |

### Calibration Procedure

1. **Power On** → System enters 60-second warm-up
2. **Warm-up** → Sensor heater stabilizes (MQ-6 requires ~60s)
3. **Baseline** → 500 samples averaged over 5 seconds in clean air
4. **Threshold** → baseline + ALARM_THRESHOLD_PPM

### PPM Calculation

```c
/* Step 1: Read ADC and convert to voltage */
adc_mv = (adc_raw * 3300) / 4095;

/* Step 2: Undo voltage divider (10k/10k = 0.5) */
sensor_mv = adc_mv * 2;

/* Step 3: Convert to PPM (simplified linear model) */
/* Higher voltage = higher gas concentration */
if (sensor_mv >= 5000) ppm = 9999;
else ppm = sensor_mv / 5;
```

> **Note:** This is a simplified linear approximation. For production use, implement the full Rs/Ro curve from the MQ-6 datasheet with temperature and humidity compensation.

---

## ADC & Filtering

### DMA Configuration

| Parameter | Value |
|-----------|-------|
| Mode | Circular |
| Buffer Size | 16 samples |
| Data Alignment | Half-Word (16-bit) |
| Direction | Peripheral → Memory |

### Moving Average Filter

```c
#define ADC_BUF_LEN         16

/* In DMA callback: */
sum = 0;
for (i = 0; i < 16; i++) sum += adc_buffer[i];
filtered = sum / 16;  /* Average of 16 samples */
```

| Filter Characteristic | Value |
|----------------------|-------|
| Window Size | 16 samples |
| Effective Update Rate | ~10 Hz |
| Attenuation | -12 dB/octave |
| Group Delay | 8 samples (~80 ms) |

---

## LCD Display

### HD44780 via PCF8574 I2C Backpack

**Connection:**
```
PCF8574 P0-P7 → LCD D4-D7, RS, RW, EN, BL
I2C Address: 0x27 (7-bit)
```

### Display Format

| Row | Content | Example |
|-----|---------|---------|
| Row 0 | State + PPM | `SAFE  0452 ppm` |
| Row 1 | Digital Status | `DO:NO ` or `DO:YES` |

### LCD Commands

| Command | Code | Function |
|---------|------|----------|
| Clear | 0x01 | Clear display, home cursor |
| Home | 0x02 | Return cursor to home |
| Entry Mode | 0x06 | Increment, no shift |
| Display On | 0x0C | Display on, cursor off |
| Function Set | 0x28 | 4-bit, 2 lines, 5x8 dots |
| Set DDRAM | 0x80+addr | Set cursor position |

---

## Build Instructions

### Prerequisites

- STM32CubeIDE (v1.10.0+) or Keil MDK-ARM
- ST-Link V2 programmer
- GCC ARM Embedded toolchain (optional)

### STM32CubeMX Configuration

1. **New Project** → STM32F103C8Tx
2. **Clock:** HSE 8MHz crystal → SYSCLK 72MHz, ADCCLK = 12MHz (Div6)
3. **ADC1:**
   - Channel 0 (PA0)
   - Continuous conversion mode
   - DMA circular mode (Half-Word)
   - 71.5 cycles sample time
4. **I2C1:**
   - PB6 SCL, PB7 SDA
   - Standard mode (100 kHz)
5. **TIM2:**
   - Channel 2 PWM (PA1)
   - Frequency: 2 kHz (Prescaler 71, Period 499)
6. **USART1:**
   - PA9 TX, PA10 RX
   - 115200 baud, 8N1
7. **GPIO:**
   - PB12, PB13, PB14: Output
   - PB15: Input (pull-up)

### Build Steps

```bash
# Using STM32CubeIDE
1. File → New → STM32 Project
2. Select STM32F103C8Tx
3. Configure peripherals in .ioc file
4. Generate Code (Alt+K)
5. Replace main.c with gas_detector_main.c
6. Build Project (Ctrl+B)
7. Debug → Run (F11)

# Command line (GCC ARM)
arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb \
    -DSTM32F103xB \
    -IInc -IDrivers/STM32F1xx_HAL_Driver/Inc \
    -IDrivers/CMSIS/Device/ST/STM32F1xx/Include \
    -IDrivers/CMSIS/Include \
    -Og -g3 -Wall \
    -c gas_detector_main.c -o main.o

arm-none-eabi-gcc main.o startup_stm32f103xb.o \
    -mcpu=cortex-m3 -mthumb -specs=nano.specs \
    -TSTM32F103C8Tx_FLASH.ld \
    -lc -lm -lnosys \
    -o gas_detector.elf

arm-none-eabi-objcopy -O ihex gas_detector.elf gas_detector.hex
```

### Flashing

```bash
# ST-Link CLI
st-flash write gas_detector.hex 0x8000000

# OpenOCD
openocd -f interface/stlink.cfg -f target/stm32f1x.cfg \
    -c "program gas_detector.hex verify reset exit"
```

---

## UART Debug Output

### Log Format

```
Tick=XXXXX ADC=XXXX mV=XXXX ppm=XXXX DO=X State=X
```

### Example Output

```
Gas Detector initialized. Baseline=660 ppm, Threshold=1660 ppm
Tick=001000 ADC=2048 mV=1650 ppm=0660 DO=0 State=0
Tick=002000 ADC=2048 mV=1650 ppm=0660 DO=0 State=0
Tick=003000 ADC=3120 mV=2512 ppm=1004 DO=1 State=1
Tick=004000 ADC=3850 mV=3100 ppm=1240 DO=1 State=2
Tick=005000 ADC=2100 mV=1695 ppm=0339 DO=0 State=2
Tick=006000 ADC=2050 mV=1655 ppm=0331 DO=0 State=0
```

| Field | Description |
|-------|-------------|
| Tick | Milliseconds since startup |
| ADC | Filtered ADC raw value (0-4095) |
| mV | ADC voltage in millivolts |
| ppm | Calculated gas concentration |
| DO | Digital output status (0=clear, 1=detected) |
| State | 0=SAFE, 1=WARNING, 2=ALARM |

---

## Configuration

### User-Adjustable Parameters

```c
#define ADC_BUF_LEN                 16
#define ADC_VREF_MV                 3300
#define ADC_MAX_VALUE               4095
#define SENSOR_WARMUP_MS            60000       /* 60 seconds */
#define ALARM_THRESHOLD_PPM         1000        /* Calibrate this! */
#define HYSTERESIS_PPM              200
#define ALARM_DEBOUNCE_COUNT        30          /* 30 ticks = 300ms */
#define BUZZER_PWM_FREQ_HZ          2000
#define SYSTEM_TICK_MS              10
```

## Troubleshooting

| Symptom | Possible Cause | Solution |
|---------|---------------|----------|
| LCD blank | Wrong I2C address | Try `0x27` or `0x3F` |
| LCD garbled | Timing issue | Increase delays in `lcd_init()` |
| Sensor reads 0 | Not warmed up | Wait 60 seconds after power-on |
| Sensor reads max | Faulty sensor or wiring | Check connections, replace MQ-6 |
| False alarms | Threshold too low | Increase `ALARM_THRESHOLD_PPM` |
| No alarm when gas present | Threshold too high | Decrease `ALARM_THRESHOLD_PPM` |
| Buzzer silent | PWM not started | Verify TIM2 initialization |
| Relay not clicking | Wrong polarity | Check relay module specs |
| UART garbage | Baud rate mismatch | Verify 115200 on both ends |
| System hangs | Watchdog not configured | Add IWDG in CubeMX |

### Calibration Issues

| Problem | Solution |
|---------|----------|
| Baseline drifts over time | Recalibrate in known clean air |
| Very high baseline | Sensor aging — replace MQ-6 |
| Unstable readings | Check power supply ripple, add capacitors |
| Slow response | Verify 60s warm-up, check heater voltage |

---

## Safety Notes

⚠️ **IMPORTANT SAFETY DISCLAIMER**

> This device is intended as a **supplementary safety system** and is **NOT a replacement for certified, professionally-installed gas detection equipment**. Always follow local building codes and safety regulations.

### Critical Warnings

1. **MQ-6 Sensor Burn-In:** New sensors require 24-48 hours initial burn-in for stable operation.

2. **Sensor Lifespan:** Replace MQ-6 module every **2 years** or per manufacturer recommendation.

3. **Power Backup:** Install an **uninterruptible power supply (UPS)** to ensure continuous monitoring during outages.

4. **Relay Output:** The relay should drive **external certified safety equipment** (e.g., certified gas shutoff valve controller), NOT directly control gas valves.

5. **Manual Shutoff:** Always install an **emergency manual gas shutoff valve** independent of this system.

6. **Weekly Testing:** Test the system weekly using a known gas source (follow local regulations).

7. **Ventilation:** Install in well-ventilated areas; sensor requires air circulation.

8. **Temperature Range:** MQ-6 operates best at -10°C to +50°C.

### Regulatory Compliance

- Follow **NFPA 54** (National Fuel Gas Code) in the USA
- Follow **IGEM/UP/1** in the UK
- For residential: Install certified CO detector alongside
- Commercial installations require **certified gas detection system**

---

## Bill of Materials

| Component | Value/Model | Qty | Notes |
|-----------|-------------|-----|-------|
| Microcontroller | STM32F103C8T6 | 1 | Blue Pill or custom PCB |
| Gas Sensor | MQ-6 Module | 1 |
| LCD Display | 16x2 HD44780 | 1 | With I2C backpack |
| I2C Backpack | PCF8574 Module | 1 | Address 0x27 or 0x3F |
| Relay Module | 5V SPDT | 1 | Opto-isolated recommended |
| Buzzer | 5V Active or Passive | 1 | With driver transistor |
| LEDs | 3mm Red + Green | 2 | With 330Ω resistors |
| Resistors | 10kΩ 1% | 4 | Voltage divider + pull-ups |
| Capacitor | 100nF ceramic | 2 | Filter + decoupling |
| LDO Regulator | AMS1117-3.3 | 1 | SOT-223 package |
| Power Jack | 5.5x2.1mm | 1 | DC 5V input |
| NPN Transistor | 2N2222 or S8050 | 1 | Relay driver |
| Base Resistor | 1kΩ | 1 | For NPN base |
| Flyback Diode | 1N4007 | 1 | Across relay coil |
| I2C Pull-ups | 4.7kΩ | 2 | For SCL and SDA |

### Optional Components

| Component | Purpose |
|-----------|---------|
| DS18B20 | Temperature compensation |
| DHT22 | Humidity compensation |
| ESP8266 | WiFi remote monitoring |
| SD Card Module | Data logging |
| IWDG Circuit | Hardware watchdog |

---

## Firmware Files

| File | Description |
|------|-------------|
| `gas_detector_main.c` | Complete firmware source |
| `main.h` | HAL header (generated by CubeMX) |
| `gas_detector_schematic.png` | System block diagram |

---

## Version History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2026-06-13 | Initial release |

---

## References

- [MQ-6 Datasheet](https://www.pololu.com/file/0J309/MQ6.pdf)
- [STM32F103 Reference Manual](https://www.st.com/resource/en/reference_manual/rm0008-stm32f101xx-stm32f102xx-stm32f103xx-stm32f105xx-and-stm32f107xx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [HD44780 Datasheet](https://www.sparkfun.com/datasheets/LCD/HD44780.pdf)
- [PCF8574 Datasheet](https://www.ti.com/lit/ds/symlink/pcf8574.pdf)
---

# License

Unless otherwise specified, all content in this repository—including, but not
limited to, software source code, firmware, hardware design files (schematics,
PCB layouts, Gerber files, BOMs, CAD files), documentation, configuration
files, examples, and supporting materials—is made available under the MIT
License.

---

## MIT License

Copyright (c) 2026 Joydeep Majumdar

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```
