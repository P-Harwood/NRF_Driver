nRF24L01+ Driver (ESP32)

Small C++ driver for the nRF24L01+ 2.4 GHz transceiver, written for the ESP32 using the ESP-IDF framework.
The code is largely compatible with other micro controller boards, just needs the SPI command logic to change. In future I intend to abstract the hardware logic.

This project was built to understand SPI communication and radio register control at a low level — no external NRF libraries are used, this was just from the datasheet.

Features

Basic SPI communication layer (spi_object wrapper with mutex)

Manual CE/CSN pin control

Initialization and configuration of key registers

Transmit and receive modes with mode switching

RX/TX FIFO management and diagnostic logging

Optional full register dump for debugging

Structure

spi_object.* — sets up and manages SPI transactions

nRF24L01P.* — driver for the radio (register setup, RX/TX handling)

Notes

Written for ESP-IDF (C++), tested with FreeRTOS tasks.

Designed for clarity and experimentation, not production.

Dynamic payloads and encryption sections are stubbed for later expansion.