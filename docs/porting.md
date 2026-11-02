---
layout: default
title: "🔧 Platform Integration"
description: "How to implement the SPI/UART interface for your platform"
nav_order: 4
parent: "📚 Documentation"
permalink: /docs/platform_integration/
---

# Platform Integration

The TMC9660 driver uses a hardware-agnostic communication interface pattern (CRTP - Curiously Recurring Template Pattern) to abstract platform-specific SPI or UART implementations. This allows the same driver code to work across different MCU platforms.

## Understanding the Interface Pattern

The driver uses **CRTP (Curiously Recurring Template Pattern)** for compile-time polymorphism. This provides:

- ✅ **Zero Runtime Overhead**: No virtual function calls, all resolved at compile time
- ✅ **Compile-Time Optimization**: Inlining and dead code elimination
- ✅ **Memory Efficiency**: No vtable overhead
- ✅ **Type Safety**: Compile-time interface checking

### Interface Architecture

```cpp
CommInterface (Abstract Base)
├── SpiCommInterface (SPI Implementation)
└── UartCommInterface (UART Implementation)
```

## SPI Interface Implementation

### Base Interface

```cpp
#include "inc/tmc9660_comm_interface.hpp"

class MySPI : public tmc9660::SpiCommInterface<MySPI> {
public:
    CommMode mode() const noexcept {
        return CommMode::SPI;
    }
    
    bool spiTransferTMCL(std::array<uint8_t,8>& tx,
                         std::array<uint8_t,8>& rx) noexcept {
        // Your SPI transfer implementation
        // Exchange 8 bytes: tx -> TMC9660, rx <- TMC9660
        return true; // Return false on error
    }
    
    bool spiTransferBootloader(std::array<uint8_t,5>& tx,
                               std::array<uint8_t,5>& rx) noexcept {
        // Your SPI bootloader transfer implementation
        // Exchange 5 bytes: tx -> TMC9660, rx <- TMC9660
        return true; // Return false on error
    }
    
    bool gpioSet(TMC9660CtrlPin pin, GpioSignal signal) noexcept {
        // Your GPIO set implementation
        return true;
    }
    
    bool gpioRead(TMC9660CtrlPin pin, GpioSignal& signal) noexcept {
        // Your GPIO read implementation
        return true;
    }
    
    void debugLog(int level, const char* tag, const char* format, va_list args) noexcept {
        // Your debug logging implementation (optional)
    }
    
    void delayMs(uint32_t ms) noexcept {
        // Your millisecond delay implementation
    }
    
    void delayUs(uint32_t us) noexcept {
        // Your microsecond delay implementation
    }
};
```

### ESP32 Example

```cpp
#include "driver/spi_master.h"
#include "inc/tmc9660_comm_interface.hpp"

class ESP32SPI : public tmc9660::SpiCommInterface<ESP32SPI> {
private:
    spi_device_handle_t spi_;
    
public:
    ESP32SPI(spi_host_device_t host, gpio_num_t mosi, gpio_num_t miso, 
              gpio_num_t sclk, gpio_num_t cs) 
        : SpiCommInterface<ESP32SPI>(true, true, false, false) { // RST: HIGH, DRV_EN: HIGH, WAKE: LOW, FAULTN: LOW
        spi_bus_config_t bus_cfg = {
            .mosi_io_num = mosi,
            .miso_io_num = miso,
            .sclk_io_num = sclk,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
        };
        spi_bus_initialize(host, &bus_cfg, SPI_DMA_CH_AUTO);
        
        spi_device_interface_config_t dev_cfg = {
            .clock_speed_hz = 4 * 1000 * 1000, // 4 MHz
            .mode = 3, // TMC9660 requires SPI mode 3
            .spics_io_num = cs,
            .queue_size = 1,
        };
        spi_bus_add_device(host, &dev_cfg, &spi_);
    }
    
    CommMode mode() const noexcept {
        return CommMode::SPI;
    }
    
    bool spiTransferTMCL(std::array<uint8_t,8>& tx,
                         std::array<uint8_t,8>& rx) noexcept {
        spi_transaction_t trans = {
            .length = 64, // 8 bytes * 8 bits
            .tx_buffer = tx.data(),
            .rx_buffer = rx.data(),
        };
        
        esp_err_t ret = spi_device_transmit(spi_, &trans);
        return (ret == ESP_OK);
    }
    
    bool spiTransferBootloader(std::array<uint8_t,5>& tx,
                               std::array<uint8_t,5>& rx) noexcept {
        spi_transaction_t trans = {
            .length = 40, // 5 bytes * 8 bits
            .tx_buffer = tx.data(),
            .rx_buffer = rx.data(),
        };
        
        esp_err_t ret = spi_device_transmit(spi_, &trans);
        return (ret == ESP_OK);
    }
    
    bool gpioSet(TMC9660CtrlPin pin, GpioSignal signal) noexcept {
        // Implementation using gpio_set_level
        return true;
    }
    
    bool gpioRead(TMC9660CtrlPin pin, GpioSignal& signal) noexcept {
        // Implementation using gpio_get_level
        return true;
    }
    
    void debugLog(int level, const char* tag, const char* format, va_list args) noexcept {
        // Route to ESP-IDF logging
    }
    
    void delayMs(uint32_t ms) noexcept {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
    
    void delayUs(uint32_t us) noexcept {
        esp_rom_delay_us(us);
    }
};

// Usage
ESP32SPI spi(SPI2_HOST, GPIO_NUM_7, GPIO_NUM_2, GPIO_NUM_6, GPIO_NUM_18);
tmc9660::TMC9660<ESP32SPI> driver(spi);
```

### STM32 HAL Example

```cpp
#include "stm32f4xx_hal.h"
#include "inc/tmc9660_comm_interface.hpp"

class STM32SPI : public tmc9660::SpiCommInterface<STM32SPI> {
private:
    SPI_HandleTypeDef* hspi_;
    GPIO_TypeDef* cs_port_;
    uint16_t cs_pin_;
    
public:
    STM32SPI(SPI_HandleTypeDef* hspi, GPIO_TypeDef* cs_port, uint16_t cs_pin)
        : SpiCommInterface<STM32SPI>(true, true, false, false), // RST: HIGH, DRV_EN: HIGH, WAKE: LOW, FAULTN: LOW
          hspi_(hspi), cs_port_(cs_port), cs_pin_(cs_pin) {
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
    }
    
    CommMode mode() const noexcept {
        return CommMode::SPI;
    }
    
    bool spiTransferTMCL(std::array<uint8_t,8>& tx,
                         std::array<uint8_t,8>& rx) noexcept {
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
        
        HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(
            hspi_, tx.data(), rx.data(), 8, 100);
        
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
        
        return (result == HAL_OK);
    }
    
    bool spiTransferBootloader(std::array<uint8_t,5>& tx,
                               std::array<uint8_t,5>& rx) noexcept {
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_RESET);
        
        HAL_StatusTypeDef result = HAL_SPI_TransmitReceive(
            hspi_, tx.data(), rx.data(), 5, 100);
        
        HAL_GPIO_WritePin(cs_port_, cs_pin_, GPIO_PIN_SET);
        
        return (result == HAL_OK);
    }
    
    bool gpioSet(TMC9660CtrlPin pin, GpioSignal signal) noexcept {
        // Implementation using HAL_GPIO_WritePin
        return true;
    }
    
    bool gpioRead(TMC9660CtrlPin pin, GpioSignal& signal) noexcept {
        // Implementation using HAL_GPIO_ReadPin
        return true;
    }
    
    void debugLog(int level, const char* tag, const char* format, va_list args) noexcept {
        // Implementation using printf or HAL logging
    }
    
    void delayMs(uint32_t ms) noexcept {
        HAL_Delay(ms);
    }
    
    void delayUs(uint32_t us) noexcept {
        // Implementation using DWT or timer
    }
};

// Usage
STM32SPI spi(&hspi1, GPIOA, GPIO_PIN_4);
tmc9660::TMC9660<STM32SPI> driver(spi);
```

### Arduino Example

```cpp
#include <SPI.h>
#include "inc/tmc9660_comm_interface.hpp"

class ArduinoSPI : public tmc9660::SpiCommInterface<ArduinoSPI> {
private:
    int cs_pin_;
    SPISettings spi_settings_;
    
public:
    ArduinoSPI(int cs_pin, uint32_t frequency = 1000000) 
        : SpiCommInterface<ArduinoSPI>(true, true, false, false), // RST: HIGH, DRV_EN: HIGH, WAKE: LOW, FAULTN: LOW
          cs_pin_(cs_pin), spi_settings_(frequency, MSBFIRST, SPI_MODE3) { // TMC9660 requires mode 3
        pinMode(cs_pin_, OUTPUT);
        digitalWrite(cs_pin_, HIGH);
        SPI.begin();
    }
    
    CommMode mode() const noexcept {
        return CommMode::SPI;
    }
    
    bool spiTransferTMCL(std::array<uint8_t,8>& tx,
                         std::array<uint8_t,8>& rx) noexcept {
        SPI.beginTransaction(spi_settings_);
        digitalWrite(cs_pin_, LOW);
        
        for (size_t i = 0; i < 8; i++) {
            rx[i] = SPI.transfer(tx[i]);
        }
        
        digitalWrite(cs_pin_, HIGH);
        SPI.endTransaction();
        return true;
    }
    
    bool spiTransferBootloader(std::array<uint8_t,5>& tx,
                               std::array<uint8_t,5>& rx) noexcept {
        SPI.beginTransaction(spi_settings_);
        digitalWrite(cs_pin_, LOW);
        
        for (size_t i = 0; i < 5; i++) {
            rx[i] = SPI.transfer(tx[i]);
        }
        
        digitalWrite(cs_pin_, HIGH);
        SPI.endTransaction();
        return true;
    }
    
    bool gpioSet(TMC9660CtrlPin pin, GpioSignal signal) noexcept {
        // Implementation using digitalWrite
        return true;
    }
    
    bool gpioRead(TMC9660CtrlPin pin, GpioSignal& signal) noexcept {
        // Implementation using digitalRead
        return true;
    }
    
    void debugLog(int level, const char* tag, const char* format, va_list args) noexcept {
        // Implementation using Serial
    }
    
    void delayMs(uint32_t ms) noexcept {
        delay(ms);
    }
    
    void delayUs(uint32_t us) noexcept {
        delayMicroseconds(us);
    }
};

// Usage
ArduinoSPI spi(10); // CS on pin 10
tmc9660::TMC9660<ArduinoSPI> driver(spi);
```

## UART Interface Implementation

### Base Interface

```cpp
#include "inc/tmc9660_comm_interface.hpp"

class MyUART : public tmc9660::UartCommInterface<MyUART> {
public:
    CommMode mode() const noexcept {
        return CommMode::UART;
    }
    
    bool uartSendTMCL(const std::array<uint8_t,9>& data) noexcept {
        // Your UART send implementation
        // Send 9-byte frame
        return true; // Return false on error
    }
    
    bool uartReceiveTMCL(std::array<uint8_t,9>& data) noexcept {
        // Your UART receive implementation
        // Receive 9-byte reply
        return true; // Return false on error
    }
    
    bool uartTransferBootloader(const std::array<uint8_t,8>& tx,
                                 std::array<uint8_t,8>& rx) noexcept {
        // Your UART bootloader transfer implementation
        return true;
    }
    
    bool gpioSet(TMC9660CtrlPin pin, GpioSignal signal) noexcept {
        // Your GPIO set implementation
        return true;
    }
    
    bool gpioRead(TMC9660CtrlPin pin, GpioSignal& signal) noexcept {
        // Your GPIO read implementation
        return true;
    }
    
    void debugLog(int level, const char* tag, const char* format, va_list args) noexcept {
        // Your debug logging implementation (optional)
    }
    
    void delayMs(uint32_t ms) noexcept {
        // Your millisecond delay implementation
    }
    
    void delayUs(uint32_t us) noexcept {
        // Your microsecond delay implementation
    }
};
```

**TMCL UART receive contract:** `uartReceiveTMCL` must return `true` only after the buffer contains a **complete 9-byte** TMCL reply. Returning early with fewer bytes leaves the driver parsing stale or garbage data (wrong TMCL status, checksum failures, or intermittent faults). Block until nine bytes arrive (extend timeouts if the link is slow), and flush the RX FIFO before a bootloader or TMCL sequence when your firmware might leave stray bytes in the UART.

**SPI TMCL inter-frame pacing:** the chip uses a two-transaction protocol — TX1 carries the new TMCL command and TX2 (NO_OP) drains its reply. At typical SPI clocks (>=1 MHz) two back-to-back transfers leave the chip ~80 µs to load the parameter-mode command into its TMCL parser, which is **shorter** than the parser actually needs on parameter-mode firmware. The chip then returns `SPI_STATUS=OK` (0xFF) but `TMCL_STATUS=REPLY_INVALID_CMD` (0x02) on the second SPI transaction — a race that disappears as soon as the host inserts even a few hundred microseconds of pacing (e.g. an `ESP_LOGI` between `spiTransferTMCL` calls is enough to mask it during debugging). Implementations should add a short, deterministic post-transfer delay inside `spiTransferTMCL` (~150 µs is sufficient on the Vortex V1 reference, well below the natural pacing of a 9-byte UART frame at 115200 baud, ~780 µs / frame). UART implementations do **not** need this delay because the wire-time of a 9-byte frame already paces the chip.

### ESP32 UART Example

```cpp
#include "driver/uart.h"
#include "inc/tmc9660_comm_interface.hpp"

class ESP32UART : public tmc9660::UartCommInterface<ESP32UART> {
private:
    uart_port_t uart_num_;
    uint8_t address_;
    
public:
    ESP32UART(uart_port_t uart_num, gpio_num_t tx, gpio_num_t rx, 
               uint8_t address = 0, int baud_rate = 115200) 
        : UartCommInterface<ESP32UART>(true, true, false, false), // RST: HIGH, DRV_EN: HIGH, WAKE: LOW, FAULTN: LOW
          uart_num_(uart_num), address_(address) {
        uart_config_t uart_config = {
            .baud_rate = baud_rate,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        };
        uart_param_config(uart_num_, &uart_config);
        uart_set_pin(uart_num_, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        uart_driver_install(uart_num_, 1024, 1024, 0, NULL, 0);
    }
    
    CommMode mode() const noexcept {
        return CommMode::UART;
    }
    
    bool uartSendTMCL(const std::array<uint8_t,9>& data) noexcept {
        int bytes_written = uart_write_bytes(uart_num_, data.data(), data.size());
        uart_wait_tx_done(uart_num_, portMAX_DELAY);
        return (bytes_written == static_cast<int>(data.size()));
    }
    
    bool uartReceiveTMCL(std::array<uint8_t,9>& data) noexcept {
        int bytes_read = uart_read_bytes(uart_num_, data.data(), data.size(), 
                                         pdMS_TO_TICKS(100));
        return (bytes_read == static_cast<int>(data.size()));
    }
    
    bool uartTransferBootloader(const std::array<uint8_t,8>& tx,
                                std::array<uint8_t,8>& rx) noexcept {
        uart_flush_input(uart_num_);
        uart_write_bytes(uart_num_, tx.data(), tx.size());
        uart_wait_tx_done(uart_num_, portMAX_DELAY);
        int bytes_read = uart_read_bytes(uart_num_, rx.data(), rx.size(), 
                                         pdMS_TO_TICKS(100));
        return (bytes_read == static_cast<int>(rx.size()));
    }
    
    bool gpioSet(TMC9660CtrlPin pin, GpioSignal signal) noexcept {
        // Implementation using gpio_set_level
        return true;
    }
    
    bool gpioRead(TMC9660CtrlPin pin, GpioSignal& signal) noexcept {
        // Implementation using gpio_get_level
        return true;
    }
    
    void debugLog(int level, const char* tag, const char* format, va_list args) noexcept {
        // Route to ESP-IDF logging
    }
    
    void delayMs(uint32_t ms) noexcept {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
    
    void delayUs(uint32_t us) noexcept {
        esp_rom_delay_us(us);
    }
};

// Usage
ESP32UART uart(UART_NUM_1, GPIO_NUM_5, GPIO_NUM_4, 0); // address = 0
tmc9660::TMC9660<ESP32UART> driver(uart);
```

## GPIO Control Interface

The TMC9660 requires GPIO control for RST, DRV_EN, WAKE, and FAULTN pins. These methods are required when implementing the CRTP interface:

```cpp
class MySPI : public tmc9660::SpiCommInterface<MySPI> {
public:
    // ... other required methods ...
    
    // GPIO control (required)
    bool gpioSet(TMC9660CtrlPin pin, GpioSignal signal) noexcept {
        // Set pin to signal state (ACTIVE or INACTIVE)
        // Use signalToGpioLevel(pin, signal) helper to convert signal to physical level
        return true;
    }
    
    bool gpioRead(TMC9660CtrlPin pin, GpioSignal& signal) noexcept {
        // Read pin state and convert to signal using gpioLevelToSignal(pin, level)
        return true;
    }
};
```

**Note:** The base `CommInterface` class provides helper methods:
- `signalToGpioLevel(pin, signal)` - Convert signal state to physical GPIO level
- `gpioLevelToSignal(pin, level)` - Convert physical GPIO level to signal state

These helpers handle board-specific active level configurations automatically.

## Testing Your Implementation

### Basic Communication Test

```cpp
bool testCommunication(tmc9660::TMC9660& driver) {
    // Try to read a known register
    auto result = driver.bootloaderGetVersion();
    return (result.has_value());
}
```

## Best Practices

1. **Error Handling**: Always return `false` from transfer methods on communication errors
2. **CS Management**: Properly assert/deassert CS for SPI
3. **Timing**: Respect SPI/UART timing requirements
4. **Thread Safety**: Make implementations thread-safe if used in multi-threaded environments
5. **No Exceptions**: Use `noexcept` and return error codes instead of throwing

## Next Steps

- **[Quick Start](quickstart.md)** - Get a minimal example running
- **[Hardware Setup](hardware_setup.md)** - Complete wiring guide

