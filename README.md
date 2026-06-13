Gas Detection & Safety System — README
1. System Overview
A production-grade STM32F103C8-based gas leak detector with MQ-6 sensor, I2C LCD, relay-driven safety shutoff, and PWM buzzer. Includes KiCad schematic, HAL firmware, and schematic diagram.

2. Hardware Architecture
MCU: STM32F103C8Tx (72 MHz Cortex-M3)

Sensor: MQ-6 module with AO/DO outputs

Display: 16×2 LCD + PCF8574 I2C backpack

Power: 5V DC input → AMS1117-3.3 LDO for MCU

Protection: 10kΩ/10kΩ voltage divider + 100nF filter on ADC input

3. Firmware Features
State Machine: SAFE → WARNING → ALARM with hysteresis and debouncing

ADC: DMA circular mode, 16-sample moving average

Calibration: 60-second warm-up + 5-second baseline acquisition

Outputs: LCD refresh (500ms), UART log (1s), LEDs, relay, PWM buzzer

4. Pin Mapping
Signal	Pin	Function
MQ-6 AO	PA0	ADC1_IN0
MQ-6 DO	PB15	GPIO Input
I2C SCL	PB6	I2C1_SCL
I2C SDA	PB7	I2C1_SDA
Relay	PB12	GPIO Output
Red LED	PB13	GPIO Output
Green LED	PB14	GPIO Output
Buzzer	PA1	TIM2_CH2 PWM
UART TX	PA9	USART1_TX
UART RX	PA10	USART1_RX
5. Build Instructions
Create STM32CubeIDE project for STM32F103C8Tx

Configure ADC1 (DMA), I2C1, TIM2 (PWM), USART1, GPIO

Replace main.c with gas_detector_main.c

Flash via ST-Link through SWD header (J2)

6. Calibration
Baseline is auto-acquired in clean air after 60s warm-up

Adjust ALARM_THRESHOLD_PPM based on observed sensor response

7. Block daigram
┌──────────────────────────────────────────────┐
│                 5V DC INPUT                  │
└────────────────┬─────────────────────────────┘
                 │
                 ├──────────────► MQ-6 Sensor
                 │               (5V Supply)
                 │
                 ├──────────────► Relay Module
                 │               (5V Coil Supply)
                 │
                 ├──────────────► Active Buzzer
                 │               (5V Supply)
                 │
                 ▼
      ┌──────────────────────┐
      │   AMS1117-3.3V REG   │
      └──────────┬───────────┘
                 │
                 ▼
      ┌──────────────────────┐
      │    STM32F103C8       │
      │      Blue Pill       │
      └──────────┬───────────┘
                 │
     ┌───────────┼───────────┬─────────────┬─────────────┐
     │           │           │             │             │
     ▼           ▼           ▼             ▼             ▼

┌─────────┐ ┌─────────┐ ┌─────────┐ ┌──────────┐ ┌─────────┐
│ MQ-6 AO │ │ MQ-6 DO │ │ LCD I2C │ │ USART1   │ │ Buzzer  │
│ Analog  │ │ Digital │ │ 16x2    │ │ Debug    │ │ Driver  │
└────┬────┘ └────┬────┘ └────┬────┘ └────┬─────┘ └────┬────┘
     │           │           │           │            │
     ▼           ▼           ▼           ▼            ▼

 PA0 ADC1     PB15       PB6/PB7      PA9/PA10      PA1
              Input       I2C1          UART1      TIM2 CH2

     │
     ▼
┌─────────────────────────────┐
│ ADC Sampling + DMA Buffer   │
│ Moving Average Filter       │
│ Voltage → PPM Conversion    │
└──────────────┬──────────────┘
               │
               ▼
┌─────────────────────────────┐
│ Gas Detection State Machine │
├─────────────────────────────┤
│ STATE_SAFE                  │
│ STATE_WARNING               │
│ STATE_ALARM                 │
└──────────────┬──────────────┘
               │
     ┌─────────┼─────────┐
     │         │         │
     ▼         ▼         ▼

┌─────────┐ ┌─────────┐ ┌─────────┐
│ Relay   │ │ LEDs    │ │ LCD     │
│ Control │ │ Status  │ │ Display │
└────┬────┘ └────┬────┘ └────┬────┘
     │           │           │
     ▼           ▼           ▼

 PB12        PB13       I2C LCD
 Relay       Red LED    Status
 Driver

 PB14
 Green LED


STATE_SAFE
──────────
Green LED ON
Relay OFF
Buzzer OFF
LCD → SAFE

STATE_WARNING
─────────────
Red LED ON
Relay OFF
Buzzer Intermittent
LCD → WARNING

STATE_ALARM
───────────
Red LED ON
Relay ON
Buzzer Continuous
LCD → ALARM
****
8. Signal flow
9. MQ-6 Sensor
     │
     ▼
ADC + DO Input
     │
     ▼
Filtering
     │
     ▼
PPM Calculation
     │
     ▼
State Machine
     │
     ├──► LCD Update
     ├──► Relay Control
     ├──► Buzzer Control
     ├──► Red LED
     └──► Green LED
