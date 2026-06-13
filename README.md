Gas Detection & Safety System
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

7. Block diagram
   <img width="1536" height="1024" alt="image" src="https://github.com/user-attachments/assets/11a026a4-3103-4f6c-8963-d291717f7f74" />
