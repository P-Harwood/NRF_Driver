#include "spi_object.hpp"
#include <cstdio> // for printf

spi_object::spi_object(){
    printf("[SPI_OBJECT] Initializing SPI bus config\n");
    spi_bus_config_t config = {};
    config.mosi_io_num = 13;
    config.miso_io_num = 12;
    config.sclk_io_num = 14; 
    config.quadwp_io_num = -1;
    config.quadhd_io_num = -1;
    config.data_io_default_level = false;
    config.max_transfer_sz = 60; 
    config.isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO;

    printf("[SPI_OBJECT] Calling spi_bus_initialize...\n");
    esp_err_t bus_init_result = spi_bus_initialize(SPI2_HOST, &config, SPI_DMA_CH_AUTO);
    if (bus_init_result != ESP_OK) {
        printf("❌ spi_bus_initialize failed with error: %d\n", bus_init_result);
    } else {
        printf("✅ spi_bus_initialize successful\n");
    }

    printf("[SPI_OBJECT] Configuring SPI device interface\n");
    spi_device_interface_config_t device_config = {};
    device_config.command_bits = 0;
    device_config.address_bits = 0;
    device_config.dummy_bits = 0;
    device_config.mode = 1;
    device_config.clock_source = SPI_CLK_SRC_DEFAULT;
    device_config.duty_cycle_pos = 0;
    device_config.clock_speed_hz = 1 * 1000 * 1000;
    device_config.input_delay_ns = 0;
    device_config.spics_io_num = 5;
    device_config.queue_size = 10;
    device_config.pre_cb = nullptr;
    device_config.post_cb = nullptr;
    device_config.flags = SPI_DEVICE_NO_DUMMY;

    // TODO: Annotate
    spi_mutex_ = xSemaphoreCreateMutex();
    configASSERT(spi_mutex_); // crash early if creation failed

    device_handle_ = nullptr;

    printf("[SPI_OBJECT] Adding SPI device...\n");
    esp_err_t add_device_result = spi_bus_add_device(SPI2_HOST, &device_config, &device_handle_);
    if (add_device_result != ESP_OK) {
        printf("❌ spi_bus_add_device failed with error: %d\n", add_device_result);
    } else {
        printf("✅ spi_bus_add_device successful, device handle acquired\n");
    }
}

spi_object::~spi_object() {
    if (spi_mutex_) {
        vSemaphoreDelete(spi_mutex_);
        spi_mutex_ = nullptr;
    }
    // cleanup spi handle if needed
}


esp_err_t spi_object::send_data(size_t data_size, const uint8_t* tx_data, uint8_t* rx_data) {
    configASSERT(spi_mutex_ != nullptr);
    configASSERT(!xPortInIsrContext()); // don’t call from ISR

    if (data_size == 0) return ESP_ERR_INVALID_SIZE;

    // Take mutex (1s timeout)
    if (xSemaphoreTake(spi_mutex_, pdMS_TO_TICKS(1000)) != pdTRUE) {
        printf("timeout taking SPI mutex\n");
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t result;
    do {
        if (!device_handle_) { result = ESP_ERR_INVALID_STATE; break; }

        spi_transaction_t t{};
        t.length    = data_size * 8;   // bits
        t.tx_buffer = tx_data;         // may be nullptr
        t.rx_buffer = rx_data;         // may be nullptr
        // no manual CS toggling; driver will assert/deassert CS

        result = spi_device_transmit(device_handle_, &t);
    } while (0);

    xSemaphoreGive(spi_mutex_);

    if (result != ESP_OK) {
        printf("❌ spi_device_transmit failed: 0x%x\n", result);
    }
    return result;
}
