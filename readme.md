nRF24L01+ Driver (ESP32)

A compact C++ driver for the nRF24L01+ 2.4 GHz transceiver, written for the ESP32 using the ESP-IDF
framework. The SPI layer is separated so the core radio logic can be reused on other MCU platforms with
minimal changes to the SPI command logic.

This project was built to understand SPI communication and radio register control at a low level (no
external NRF libraries are used; everything is derived from the datasheet).

---

## Highlights

- Minimal, datasheet-driven implementation
- Explicit CE/CSN pin control for clear timing
- RX/TX mode switching with FIFO management
- Optional full register dump for diagnostics
- Thread-safe SPI wrapper (mutex-based)

---

## Project Structure

- `spi_object.*` — SPI initialization and transaction wrapper
- `nRF24L01P.*` — radio driver (register setup, RX/TX handling)

---

## Hardware Setup

This driver expects a standard nRF24L01+ module connected to the ESP32 SPI peripheral.

Default SPI pin assignments in `spi_object`:

- **MOSI** → GPIO 13
- **MISO** → GPIO 12
- **SCK** → GPIO 14
- **CSN** → GPIO 5 (SPI device select)

Pins that are user-defined via `Pins_T` in `nRF24L01P`:

- **CE** → configurable GPIO (radio enable)
- **IRQ** → optional GPIO (interrupt line)

Minimum wiring:

- **VCC** → 3.3V (do not use 5V)
- **GND** → GND
- **MOSI/MISO/SCK** → ESP32 SPI pins
- **CSN** → GPIO 5 by default (or update `spi_object`)
- **CE** → configurable GPIO (radio enable, set in `Pins_T`)
- **IRQ** → optional GPIO (used for interrupts if desired, set in `Pins_T`)

Recommended: place a 10µF capacitor across VCC/GND on the module for stability.

---

## Current Status

This is a learning-focused driver. It aims to be readable and easy to
extend, rather than exhaustive. Areas that are intentionally stubbed or minimal:

- Dynamic payloads
- Auto-ack and retransmit tuning
- Encryption and higher-level protocols
- Robust interrupt-driven RX handling

---

## Troubleshooting Notes

- If you see unreliable RX/TX, double-check wiring, power stability, and antenna orientation.
- nRF24L01+ modules are sensitive to power noise; the decoupling capacitor helps a lot.
- SPI timing is platform-specific; some boards need slower SPI clock rates.
