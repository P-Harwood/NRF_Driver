
#pragma once


extern "C" {
    #include <stdint.h>
    #include "driver/spi_master.h"
    #include "driver/gpio.h"

}
using u8 = uint8_t;


class spi_object{
    private:
    SemaphoreHandle_t spi_mutex_;   // guard for all SPI access

    public:
    spi_bus_config_t config;
    spi_device_handle_t device_handle_;


    spi_object();
    ~spi_object();


    esp_err_t send_data(size_t data_size, const u8* tx_data, u8* rx_data = nullptr);


};