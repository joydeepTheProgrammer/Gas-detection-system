
/*
 * LPG Gas Detection System
 * Target: STM32F103C8 (Blue Pill)
 * Sensor: MQ-6 LPG Gas Sensor Module
 * Peripherals:
 *   - ADC1 (PA0) for MQ-6 AO (analog output)
 *   - GPIO (PB15) for MQ-6 DO (digital output)
 *   - I2C1 (PB6=SCL, PB7=SDA) for LCD 16x2 with PCF8574 backpack
 *   - GPIO (PB12) for Relay control (via NPN transistor)
 *   - TIM2 CH2 (PA1) for Buzzer PWM (or GPIO toggle)
 *   - GPIO (PB13) Red LED, (PB14) Green LED
 *   - USART1 (PA9=TX, PA10=RX) for debug logging
 *
 * Power:
 *   - 5V DC input -> AMS1117-3.3 for MCU
 *   - MQ-6 powered from 5V rail
 *   - Relay and Buzzer powered from 5V rail, driven by 3.3V GPIO via NPN
 *
 * Design notes:
 *   - ADC input is protected by a voltage divider (10k+10k) so MQ-6 AO (0-5V)
 *     is scaled to 0-2.5V safe for STM32 3.3V ADC. A 100nF capacitor filters noise.
 *   - Hysteresis prevents relay chattering around threshold.
 *   - Warm-up delay of 60 seconds ensures sensor stabilization (MQ-6 datasheet).
 *   - Moving average filter smooths ADC readings.
 *   - State machine for alarm logic: SAFE -> WARNING -> ALARM -> (reset to SAFE).
 */

#include "main.h"
#include <stdio.h>
#include <string.h>

/* ======================== USER CONFIGURATION ========================== */
#define ADC_BUF_LEN                 16
#define ADC_VREF_MV                 3300
#define ADC_MAX_VALUE               4095
#define SENSOR_WARMUP_MS            60000
#define ALARM_THRESHOLD_PPM         1000    /* Calibrate this value */
#define HYSTERESIS_PPM              200
#define MOVING_AVG_SHIFT            4       /* divide by 16 */
#define ALARM_DEBOUNCE_COUNT        3
#define BUZZER_PWM_FREQ_HZ          2000
#define SYSTEM_TICK_MS              10

/* ========================== PIN MAPPING =============================== */
/* ADC: PA0 (MQ-6 AO) */
/* DO:  PB15 (MQ-6 DO) */
/* I2C: PB6 (SCL), PB7 (SDA) */
/* Relay: PB12 */
/* Red LED: PB13 */
/* Green LED: PB14 */
/* Buzzer PWM: PA1 (TIM2 CH2) */
/* UART: PA9 (TX), PA10 (RX) */

/* ========================== GLOBAL HANDLES ============================ */
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;

/* LCD I2C backpack (PCF8574) address: usually 0x27 or 0x3F */
#define LCD_I2C_ADDR                0x27

/* ========================== STATE MACHINE ============================= */
typedef enum {
    STATE_SAFE = 0,
    STATE_WARNING,
    STATE_ALARM
} SystemState_t;

static volatile SystemState_t g_state = STATE_SAFE;
static volatile uint32_t g_adc_raw = 0;
static volatile uint32_t g_adc_filtered = 0;
static volatile uint32_t g_baseline = 0;
static volatile uint32_t g_alarm_threshold = 0;
static volatile uint8_t g_dig_detected = 0;
static volatile uint8_t g_debounce_counter = 0;
static volatile uint32_t g_tick = 0;
static volatile uint8_t g_warmup_done = 0;

/* ========================== LCD I2C DRIVER ============================ */
/* Minimal HD44780 driver via PCF8574 I2C backpack */
#define LCD_RS_PIN                  0x01
#define LCD_RW_PIN                  0x02
#define LCD_EN_PIN                  0x04
#define LCD_BL_PIN                  0x08

static uint8_t lcd_backlight = LCD_BL_PIN;

static void lcd_write_nibble(uint8_t nibble, uint8_t rs) {
    uint8_t data = (nibble & 0xF0) | rs | lcd_backlight;
    uint8_t tx[2];
    tx[0] = data | LCD_EN_PIN;
    tx[1] = data;
    HAL_I2C_Master_Transmit(&hi2c1, LCD_I2C_ADDR << 1, tx, 2, 100);
}

static void lcd_write_byte(uint8_t byte, uint8_t rs) {
    lcd_write_nibble(byte & 0xF0, rs);
    lcd_write_nibble((byte << 4) & 0xF0, rs);
}

static void lcd_send_cmd(uint8_t cmd) {
    lcd_write_byte(cmd, 0);
    HAL_Delay(2);
}

static void lcd_send_data(uint8_t data) {
    lcd_write_byte(data, LCD_RS_PIN);
}

static void lcd_init(void) {
    HAL_Delay(50);
    lcd_write_nibble(0x30, 0);
    HAL_Delay(5);
    lcd_write_nibble(0x30, 0);
    HAL_Delay(5);
    lcd_write_nibble(0x30, 0);
    HAL_Delay(1);
    lcd_write_nibble(0x20, 0); /* 4-bit mode */
    lcd_send_cmd(0x28);        /* 2 lines, 5x8 font */
    lcd_send_cmd(0x0C);        /* Display on, cursor off, blink off */
    lcd_send_cmd(0x06);        /* Entry mode: increment, no shift */
    lcd_send_cmd(0x01);        /* Clear display */
    HAL_Delay(2);
}

static void lcd_clear(void) {
    lcd_send_cmd(0x01);
    HAL_Delay(2);
}

static void lcd_set_cursor(uint8_t row, uint8_t col) {
    uint8_t addr = (row == 0) ? (0x80 + col) : (0xC0 + col);
    lcd_send_cmd(addr);
}

static void lcd_print(const char *str) {
    while (*str) {
        lcd_send_data((uint8_t)(*str));
        str++;
    }
}

static void lcd_printf(const char *fmt, ...) {
    char buf[17];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    lcd_print(buf);
}

/* ========================== UART DEBUG ================================ */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE {
    HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* ========================== ADC / FILTER ============================== */
static uint32_t adc_buffer[ADC_BUF_LEN];
static uint32_t adc_sum = 0;
static uint8_t adc_idx = 0;

static uint32_t adc_to_mv(uint32_t adc_val) {
    return (adc_val * ADC_VREF_MV) / ADC_MAX_VALUE;
}

/* Apply voltage divider scaling: divider = 10k/(10k+10k) = 0.5 */
/* So actual sensor voltage = 2 * ADC measured voltage */
static uint32_t sensor_voltage_to_ppm(uint32_t mv) {
    /* MQ-6 calibration: simplistic mapping from voltage to ppm.
     * In production, use sensor datasheet Rs/Ro curves and temperature/humidity compensation.
     * This returns a representative value for demonstration. */
    uint32_t sensor_mv = mv * 2; /* undo divider */
    /* Heuristic: higher voltage = lower Rs = higher gas concentration */
    /* PPM ~ (Vc - Vout) * k / Vout. Here Vc = 5V (5000mV). */
    if (sensor_mv >= 4950) return 0;
    uint32_t ppm = (5000 - sensor_mv) / 5; /* rough linearized placeholder */
    if (ppm > 9999) ppm = 9999;
    return ppm;
}

static void filter_adc(void) {
    uint32_t raw = adc_buffer[adc_idx];
    adc_sum += raw;
    adc_sum -= adc_buffer[(adc_idx + 1) % ADC_BUF_LEN];
    adc_buffer[adc_idx] = raw;
    adc_idx = (adc_idx + 1) % ADC_BUF_LEN;
    g_adc_filtered = adc_sum >> MOVING_AVG_SHIFT;
}

/* ========================== HAL INITIALIZATION ======================== */
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);

int main(void) {
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_DMA_Init();
    MX_ADC1_Init();
    MX_I2C1_Init();
    MX_TIM2_Init();
    MX_USART1_UART_Init();

    lcd_init();
    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("LPG Detector");
    lcd_set_cursor(1, 0);
    lcd_print("Warming up...");

    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, ADC_BUF_LEN);

    /* Warm-up period: 60 seconds per MQ-6 datasheet */
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < SENSOR_WARMUP_MS) {
        HAL_Delay(100);
        /* Optionally blink green LED */
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14);
    }
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); /* Green on = safe */
    g_warmup_done = 1;

    /* Baseline calibration: average 500 samples over 5 seconds */
    uint32_t baseline_sum = 0;
    for (uint16_t i = 0; i < 500; i++) {
        HAL_Delay(10);
        baseline_sum += g_adc_filtered;
    }
    g_baseline = baseline_sum / 500;
    g_alarm_threshold = g_baseline + ALARM_THRESHOLD_PPM;
    if (g_alarm_threshold < g_baseline) g_alarm_threshold = g_baseline + 500;

    lcd_clear();
    lcd_set_cursor(0, 0);
    lcd_print("System Ready");
    printf("LPG Detector initialized. Baseline=%lu, Threshold=%lu\r\n", g_baseline, g_alarm_threshold);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    while (1) {
        HAL_Delay(SYSTEM_TICK_MS);
        g_tick += SYSTEM_TICK_MS;

        /* Read digital output */
        g_dig_detected = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET) ? 1 : 0;
        /* MQ-6 DO is active LOW when gas detected (verify module variant) */

        uint32_t adc_mv = adc_to_mv(g_adc_filtered);
        uint32_t ppm = sensor_voltage_to_ppm(adc_mv);

        /* Hysteresis logic */
        switch (g_state) {
            case STATE_SAFE:
                if (ppm >= (g_alarm_threshold + HYSTERESIS_PPM) || g_dig_detected) {
                    g_debounce_counter++;
                    if (g_debounce_counter >= ALARM_DEBOUNCE_COUNT) {
                        g_state = STATE_WARNING;
                        g_debounce_counter = 0;
                    }
                } else {
                    g_debounce_counter = 0;
                }
                break;

            case STATE_WARNING:
                if (ppm >= (g_alarm_threshold + HYSTERESIS_PPM + 300)) {
                    g_debounce_counter++;
                    if (g_debounce_counter >= ALARM_DEBOUNCE_COUNT) {
                        g_state = STATE_ALARM;
                        g_debounce_counter = 0;
                    }
                } else if (ppm < (g_alarm_threshold - HYSTERESIS_PPM)) {
                    g_debounce_counter++;
                    if (g_debounce_counter >= ALARM_DEBOUNCE_COUNT) {
                        g_state = STATE_SAFE;
                        g_debounce_counter = 0;
                    }
                } else {
                    g_debounce_counter = 0;
                }
                break;

            case STATE_ALARM:
                if (ppm < (g_alarm_threshold - HYSTERESIS_PPM)) {
                    g_debounce_counter++;
                    if (g_debounce_counter >= (ALARM_DEBOUNCE_COUNT * 3)) {
                        g_state = STATE_SAFE;
                        g_debounce_counter = 0;
                    }
                } else {
                    g_debounce_counter = 0;
                }
                break;
        }

        /* Actuators */
        if (g_state == STATE_ALARM) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);   /* Relay ON */
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);   /* Red LED ON */
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); /* Green LED OFF */
            /* Buzzer PWM at 2kHz */
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, htim2.Init.Period / 2);
        } else if (g_state == STATE_WARNING) {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); /* Relay OFF */
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);  /* Red LED ON */
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
            /* Intermittent buzzer: toggle every 500ms */
            uint32_t buzz = (g_tick / 500) % 2;
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, buzz ? (htim2.Init.Period / 2) : 0);
        } else {
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); /* Relay OFF */
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);/* Red LED OFF */
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); /* Green LED ON */
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);     /* Buzzer OFF */
        }

        /* LCD update every 500ms */
        if ((g_tick % 500) == 0) {
            lcd_clear();
            lcd_set_cursor(0, 0);
            if (!g_warmup_done) {
                lcd_print("Warming up...");
            } else {
                if (g_state == STATE_SAFE) {
                    lcd_printf("SAFE  %4lu ppm", ppm);
                } else if (g_state == STATE_WARNING) {
                    lcd_printf("WARN  %4lu ppm", ppm);
                } else {
                    lcd_printf("ALARM %4lu ppm", ppm);
                }
                lcd_set_cursor(1, 0);
                lcd_printf("DO:%s", g_dig_detected ? "YES" : "NO ");
            }
        }

        /* UART logging every 1 second */
        if ((g_tick % 1000) == 0) {
            printf("Tick=%lu ADC=%lu mV=%lu ppm=%lu DO=%d State=%d\r\n",
                   g_tick, g_adc_filtered, adc_mv, ppm, g_dig_detected, g_state);
        }
    }
}

/* ========================== ADC DMA CALLBACK ========================== */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    if (hadc == &hadc1) {
        /* Simple moving average */
        uint32_t sum = 0;
        for (int i = 0; i < ADC_BUF_LEN; i++) {
            sum += adc_buffer[i];
        }
        g_adc_raw = sum >> MOVING_AVG_SHIFT;
        g_adc_filtered = g_adc_raw; /* update filtered value */
        HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, ADC_BUF_LEN);
    }
}

/* ========================== HAL MXC GENERATED ========================= */

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_GPIO_Init(void) {
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* PA0: ADC input (analog) */
    /* PA1: TIM2 CH2 (alternate function push-pull) */
    /* PA9/PA10: USART1 (AF push-pull) */
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_9 | GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* PB6, PB7: I2C1 (alternate function open-drain) */
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PB12: Relay (output) */
    /* PB13: Red LED (output) */
    /* PB14: Green LED (output) */
    /* PB15: MQ-6 DO (input) */
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* PB15 input mode override */
    GPIO_InitStruct.Pin = GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* Initial states */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14, GPIO_PIN_RESET);
}

static void MX_DMA_Init(void) {
    __HAL_RCC_DMA1_CLK_ENABLE();
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

void DMA1_Channel1_IRQHandler(void) {
    HAL_DMA_IRQHandler(&hdma_adc1);
}

static void MX_ADC1_Init(void) {
    __HAL_RCC_ADC1_CLK_ENABLE();
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK) {
        Error_Handler();
    }

    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }

    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
        Error_Handler();
    }
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
}

static void MX_I2C1_Init(void) {
    __HAL_RCC_I2C1_CLK_ENABLE();
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
}

static void MX_TIM2_Init(void) {
    __HAL_RCC_TIM2_CLK_ENABLE();
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 71;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 999; /* 1kHz timer base */
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK) {
        Error_Handler();
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK) {
        Error_Handler();
    }
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK) {
        Error_Handler();
    }
    HAL_TIM_MspPostInit(&htim2);
}

static void MX_USART1_UART_Init(void) {
    __HAL_RCC_USART1_CLK_ENABLE();
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK) {
        Error_Handler();
    }
}

void Error_Handler(void) {
    __disable_irq();
    while (1) {
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_13);
        HAL_Delay(100);
    }
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM2) {
        __HAL_RCC_GPIOA_CLK_ENABLE();
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    }
}

/* Required HAL stubs */
void HAL_MspInit(void) {
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line) {
    (void)file; (void)line;
}
#endif
