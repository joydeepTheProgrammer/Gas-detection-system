# Traffic Management System

## Embedded Firmware v2.0

**Date:** 2026-06-13  
**Target:** ARM Cortex-M4 / POSIX / FreeRTOS  
**Language:** C (C99)  
**Build:** GCC with `-Wall -Wextra -Werror -O2`

---

## Table of Contents

1. [Overview](#overview)
2. [System Architecture](#system-architecture)
3. [Hardware Abstraction Layer](#hardware-abstraction-layer)
4. [Communication Protocol](#communication-protocol)
5. [Task Scheduler](#task-scheduler)
6. [Watchdog & Safety](#watchdog--safety)
7. [Building](#building)
8. [Porting to Real Hardware](#porting-to-real-hardware)
9. [API Reference](#api-reference)
10. [File Structure](#file-structure)

---

## Overview

A real-time adaptive traffic management system for 4-way intersections with multi-sensor fusion, emergency vehicle preemption, pedestrian crossing management, congestion detection, and fault recovery.

### Key Features

| Feature | Description |
|---------|-------------|
| **Multi-Sensor Fusion** | IR, Ultrasonic, Inductive Loop, Camera/AI Vision, V2X, Weather |
| **Adaptive Timing** | Green time dynamically adjusts based on traffic density history |
| **Emergency Preemption** | Immediate green for emergency vehicles; all others forced RED |
| **Pedestrian Management** | Queue-based crossing requests processed during green phases |
| **Congestion Detection** | System-wide load monitoring with automatic mode switching |
| **Night Mode** | Auto-activates 22:00-05:00 with blinking yellow |
| **Fault Recovery** | Sensor confidence monitoring, stuck-light watchdog, auto-reset |
| **Thread Safety** | Mutex-protected shared state across all tasks |
| **OTA Updates** | Firmware update protocol with CRC verification |

---

## System Architecture

```
+------------------------------------------------------------------+
|                        SENSOR LAYER                               |
|  IR  |  Ultrasonic  |  Inductive  |  Camera  |  V2X  |  Weather  |
+------------------------------------------------------------------+
|                    DATA ACQUISITION UNIT (DAQ)                    |
|                    Sensor Fusion & Filtering                       |
+------------------------------------------------------------------+
|                     COMMUNICATION LAYER                           |
|  ZigBee/LoRa  |  Ethernet/Fiber  |  4G/5G  |  V2X (DSRC/C-V2X)  |
+------------------------------------------------------------------+
|                  CENTRAL INTELLIGENCE CONTROLLER                  |
|  +----------------+ +----------------+ +----------------+         |
|  | Traffic Flow   | | Adaptive Timing| | Priority Queue |         |
|  | Analyzer       | | Algorithm      | | Manager        |         |
|  +----------------+ +----------------+ +----------------+         |
|  +----------------+ +----------------+ +----------------+         |
|  | Congestion     | | Real-time OS   | | Database       |         |
|  | Predictor      | | (FreeRTOS)     | | (SQLite)       |         |
|  +----------------+ +----------------+ +----------------+         |
+------------------------------------------------------------------+
|                        ACTUATOR LAYER                            |
|  Traffic LEDs  |  VMS Signs  |  Dynamic Lane  |  Pedestrian       |
+------------------------------------------------------------------+
|                        POWER SYSTEM                                |
|  Main AC  |  UPS Battery  |  Solar + Charge Controller  |  PMU    |
+------------------------------------------------------------------+
```

### State Machine

```
GREEN --> YELLOW (3s) --> ALL-RED (2s) --> [Select Next] --> GREEN
```

Next direction selected by priority scoring:
```
Priority = vehicle_count + (pedestrian_waiting * 2) + emergency_bonus
```

---

## Hardware Abstraction Layer

### GPIO Pin Mapping

| Signal | Port | Pin | Direction |
|--------|------|-----|-----------|
| NORTH_RED | A | 0 | Output |
| NORTH_YELLOW | A | 1 | Output |
| NORTH_GREEN | A | 2 | Output |
| SOUTH_RED | A | 3 | Output |
| SOUTH_YELLOW | A | 4 | Output |
| SOUTH_GREEN | A | 5 | Output |
| EAST_RED | A | 6 | Output |
| EAST_YELLOW | A | 7 | Output |
| EAST_GREEN | A | 8 | Output |
| WEST_RED | A | 9 | Output |
| WEST_YELLOW | A | 10 | Output |
| WEST_GREEN | A | 11 | Output |

### Sensor Pin Mapping

| Sensor | Port | Pin | Type |
|--------|------|-----|------|
| IR_NORTH | B | 0 | Digital Input |
| IR_SOUTH | B | 1 | Digital Input |
| IR_EAST | B | 2 | Digital Input |
| IR_WEST | B | 3 | Digital Input |
| US_TRIG_NORTH | B | 4 | Digital Output |
| US_ECHO_NORTH | B | 5 | Digital Input |
| US_TRIG_SOUTH | B | 6 | Digital Output |
| US_ECHO_SOUTH | B | 7 | Digital Input |
| IND_NORTH | B | 8 | Analog (ADC) |
| IND_SOUTH | B | 9 | Analog (ADC) |
| IND_EAST | B | 10 | Analog (ADC) |
| IND_WEST | B | 11 | Analog (ADC) |

### HAL API

```c
/* GPIO */
int hal_gpio_init(uint32_t pin, uint8_t mode, uint8_t speed);
int hal_gpio_write(uint32_t pin, bool value);
bool hal_gpio_read(uint32_t pin);

/* UART */
int hal_uart_init(uint8_t uart_id, uint32_t baud, uint8_t parity, uint8_t stop_bits);
int hal_uart_send(uint8_t uart_id, const uint8_t *data, uint16_t len);
int hal_uart_receive(uint8_t uart_id, uint8_t *data, uint16_t len, uint32_t timeout_ms);

/* ADC */
int hal_adc_init(uint8_t channel, uint8_t resolution);
int hal_adc_read(uint8_t channel, uint16_t *value);
float hal_adc_read_voltage(uint8_t channel);

/* Timer */
uint32_t hal_get_tick_ms(void);
void hal_delay_ms(uint32_t ms);

/* System */
void hal_system_reset(void);
uint32_t hal_get_unique_id(void);
```

---

## Communication Protocol

### Frame Format

```
+--------+--------+--------+--------+--------+-----+--------+--------+--------+--------+
| 0x55   | 0xAA   | LEN_HI | LEN_LO | CMD    | ... | CRC31  | CRC23  | CRC15  | CRC07  |
| SYNC1  | SYNC2  | Payload Length (2 bytes)    | Payload (N bytes)    | CRC32 (4 bytes)    |
+--------+--------+--------+--------+--------+-----+--------+--------+--------+--------+
```

### Command Reference

| Code | Command | Direction | Description |
|------|---------|-----------|-------------|
| 0x01 | HEARTBEAT | Bidir | Keep-alive ping |
| 0x02 | STATUS_REQ | Host->Device | Request system status |
| 0x03 | STATUS_RESP | Device->Host | System status response |
| 0x04 | RESET | Host->Device | System reset |
| 0x05 | SET_MODE | Host->Device | Set operation mode |
| 0x10 | SET_LIGHT | Host->Device | Force light state |
| 0x11 | GET_LIGHT | Host->Device | Read light state |
| 0x12 | SET_TIMING | Host->Device | Configure timing parameters |
| 0x13 | GET_TIMING | Host->Device | Read timing parameters |
| 0x14 | FORCE_PHASE | Host->Device | Force phase transition |
| 0x20 | SENSOR_READ | Host->Device | Read sensor value |
| 0x21 | SENSOR_CONFIG | Host->Device | Configure sensor |
| 0x22 | SENSOR_CALIBRATE | Host->Device | Calibrate sensor |
| 0x30 | EMERGENCY_TRIGGER | Host->Device | Trigger emergency preemption |
| 0x31 | EMERGENCY_CLEAR | Host->Device | Clear emergency preemption |
| 0x40 | PEDESTRIAN_REQUEST | Host->Device | Request pedestrian crossing |
| 0x41 | PEDESTRIAN_CLEAR | Host->Device | Clear pedestrian request |
| 0x50 | LOG_READ | Host->Device | Read system log |
| 0x51 | LOG_CLEAR | Host->Device | Clear system log |
| 0x52 | LOG_CONFIG | Host->Device | Configure logging |
| 0x60 | FW_UPDATE_START | Host->Device | Start firmware update |
| 0x61 | FW_UPDATE_DATA | Host->Device | Firmware data chunk |
| 0x62 | FW_UPDATE_VERIFY | Host->Device | Verify firmware CRC |
| 0x63 | FW_UPDATE_COMMIT | Host->Device | Commit firmware update |
| 0xFF | ERROR | Device->Host | Error response |

### Response Codes

| Code | Meaning |
|------|---------|
| 0x00 | OK |
| 0x01 | Invalid Command |
| 0x02 | Invalid Parameter |
| 0x03 | CRC Error |
| 0x04 | Timeout |
| 0x05 | Busy |
| 0x06 | Not Supported |
| 0x07 | System Fault |
| 0x08 | Buffer Overflow |

---

## Task Scheduler

### Cooperative Multitasking

| Task | Priority | Period | Critical | Function |
|------|----------|--------|----------|----------|
| Control | CRITICAL | 100ms | Yes | Traffic light state machine |
| Sensor | HIGH | 500ms | Yes | Multi-sensor data acquisition |
| Communication | HIGH | 50ms | Yes | UART protocol handling |
| Diagnostics | NORMAL | 1000ms | No | Health checks & fault recording |
| Simulation | LOW | 100ms | No | Traffic generation (test only) |

### IPC Primitives

```c
/* Mutex */
int mutex_init(SimpleMutex *mutex);
int mutex_lock(SimpleMutex *mutex, uint32_t timeout_ms);
int mutex_unlock(SimpleMutex *mutex);

/* Event Flags */
int event_init(EventFlags *event);
int event_set(EventFlags *event, uint32_t flags);
int event_wait(EventFlags *event, uint32_t flags, uint32_t timeout_ms);

/* Message Queue */
int queue_init(MsgQueue *queue, uint16_t item_size);
int queue_send(MsgQueue *queue, const void *item, uint32_t timeout_ms);
int queue_receive(MsgQueue *queue, void *item, uint32_t timeout_ms);
```

---

## Watchdog & Safety

### Per-Task Monitoring

```c
wdg_register_task(TASK_CONTROL, "control", 100, true);   /* Critical */
wdg_register_task(TASK_SENSOR, "sensor", 500, true);    /* Critical */
wdg_pet(TASK_CONTROL);  /* Called every task iteration */
```

### Fault Types

| Type | Trigger | Action |
|------|---------|--------|
| WATCHDOG_TIMEOUT | Task misses 3 deadlines | Record fault, reset if critical |
| STACK_OVERFLOW | Stack pointer violation | Immediate reset |
| MEMORY_CORRUPTION | CRC mismatch in RAM | Record fault, attempt recovery |
| SENSOR_FAILURE | Confidence < 30% | Degrade gracefully, use backup sensors |
| COMM_TIMEOUT | No heartbeat for 5s | Enter safe mode (all red) |
| POWER_LOW | Voltage < 10V | Reduce power, notify central |
| OVER_TEMPERATURE | Temperature > 80C | Throttle, emergency shutdown if > 100C |
| STUCK_LIGHT | Green > max + 10s | Force yellow, record fault |
| INVALID_STATE | Illegal state transition | Reset to safe state |

### Auto-Reset Policy

- 3 consecutive critical faults → System reset
- 5 total faults in 60s → System reset
- Power brownout → Immediate reset with brownout flag

---

## Building

### Prerequisites

- GCC (>= 9.0) or ARM GCC
- POSIX environment (Linux/macOS) or ARM toolchain
- `make` (optional)

### Build Commands

**Single-file version (quick test):**
```bash
gcc -o traffic_system traffic_management_system.c \
    -lpthread -lm -Wall -Wextra -Werror -O2
./traffic_system
```

**Modular firmware (production):**
```bash
cd firmware
gcc -o build/traffic_firmware \
    src/main.c src/utils.c src/hal_sim.c \
    src/scheduler.c src/protocol.c src/watchdog.c \
    -I include -lpthread -lm -Wall -Wextra -Werror -O2
./build/traffic_firmware
```

**With debug symbols:**
```bash
gcc ... -g -O0 -DDEBUG
```

**Static analysis:**
```bash
cppcheck --enable=all --inconclusive src/
```

---

## Porting to Real Hardware

### Step 1: Replace HAL Simulation

Create `hal_stm32.c` implementing all HAL functions using STM32 HAL library:

```c
/* Example: GPIO write for STM32 */
int hal_gpio_write(uint32_t pin, bool value) {
    uint16_t gpio_pin = pin & 0xFFFF;
    GPIO_TypeDef *port = get_gpio_port(pin >> 16);
    HAL_GPIO_WritePin(port, gpio_pin, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
    return 0;
}
```

### Step 2: Integrate FreeRTOS

Replace cooperative scheduler with FreeRTOS tasks:

```c
/* FreeRTOS task creation */
xTaskCreate(task_control, "control", 512, NULL, 3, NULL);
xTaskCreate(task_sensor, "sensor", 256, NULL, 2, NULL);
vTaskStartScheduler();
```

### Step 3: Hardware Watchdog

Use independent watchdog (IWDG):

```c
/* STM32 IWDG */
HAL_IWDG_Init(&hiwdg);
HAL_IWDG_Refresh(&hiwdg);  /* In task loop */
```

### Step 4: DMA UART

Replace polling UART with DMA + interrupts:

```c
/* STM32 HAL UART DMA */
HAL_UART_Receive_DMA(&huart1, rx_buffer, RX_BUFFER_SIZE);
HAL_UART_Transmit_DMA(&huart1, tx_buffer, tx_len);
```

### Step 5: Flash Storage

Use internal flash for configuration and logs:

```c
/* STM32 Flash write */
HAL_FLASH_Unlock();
HAL_FLASH_Program(TYPEPROGRAM_WORD, addr, data);
HAL_FLASH_Lock();
```

### Memory Map (STM32F407)

| Region | Address | Size | Purpose |
|--------|---------|------|---------|
| Flash | 0x0800_0000 | 1 MB | Firmware + Config |
| RAM | 0x2000_0000 | 128 KB | Runtime data |
| CCM RAM | 0x1000_0000 | 64 KB | Stack + Critical |

---

## API Reference

### System Initialization

```c
int traffic_system_init(void);
void traffic_system_shutdown(void);
```

### Traffic Control

```c
void set_light_state(Direction dir, TrafficLightState state);
void execute_phase_transition(void);
uint32_t calculate_adaptive_green_time(Direction dir);
```

### Queue Management

```c
bool enqueue_vehicle(Direction dir, VehicleType type, uint32_t priority);
bool dequeue_vehicle(Direction dir);
void request_pedestrian_crossing(Direction dir);
void process_pedestrian_crossing(Direction dir);
```

### Emergency Handling

```c
void trigger_emergency_preemption(Direction dir, uint32_t vehicle_id);
void clear_emergency_preemption(Direction dir);
```

### Protocol

```c
int protocol_init(void);
int protocol_encode_frame(ProtocolFrame *frame, uint8_t cmd, 
                          const uint8_t *payload, uint16_t len);
int protocol_decode_frame(const uint8_t *raw, uint16_t raw_len, 
                          ProtocolFrame *frame);
int protocol_send_response(uint8_t uart_id, uint8_t cmd, 
                           ResponseCode code, const uint8_t *data, uint16_t len);
```

### Watchdog

```c
int wdg_init(uint32_t timeout_ms);
int wdg_register_task(uint8_t task_id, const char *name, 
                      uint32_t period_ms, bool critical);
int wdg_pet(uint8_t task_id);
int wdg_check_all(void);
void wdg_force_reset(const char *reason);
```

---

## File Structure

```
traffic_management_firmware/
├── traffic_management_block_diagram.png    # System architecture diagram
├── traffic_management_system.c             # Single-file version
└── firmware/
    ├── include/
    │   ├── traffic_system.h               # Core definitions
    │   ├── hal.h                          # Hardware abstraction
    │   ├── protocol.h                     # Communication protocol
    │   ├── scheduler.h                    # Task scheduler
    │   └── watchdog.h                     # Safety monitor
    ├── src/
    │   ├── main.c                         # Entry point & tasks
    │   ├── utils.c                        # Globals & utilities
    │   ├── hal_sim.c                      # POSIX HAL simulation
    │   ├── scheduler.c                    # Scheduler implementation
    │   ├── protocol.c                     # Protocol implementation
    │   └── watchdog.c                     # Watchdog implementation
    └── build/
        └── traffic_firmware               # Compiled executable
```

