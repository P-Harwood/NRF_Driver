

#pragma once


#include "spi_object.hpp"

extern "C" {
    #include <stdlib.h>
    #include <stdint.h>
    #include <stdio.h>
    #include <string.h>
    #include "driver/gpio.h"
    #include "esp_timer.h"
    #include <rom/ets_sys.h>

}

using u8 = uint8_t;

enum class Antenna_Mode :u8{
    Transmit,
    Recieve
};

struct Pins_T{
    const gpio_num_t CE;
    const gpio_num_t CSN;
    const gpio_num_t CLK;
    const gpio_num_t MISO;
    const gpio_num_t MOSI;
    const gpio_num_t IRQ;
}



class NRF24{

    private:
        const spi_object spi_;

        const Pins_T pins_layout;

        const u8 buffer_size_;


        /**
         * @brief maximun size the fifo buffer can be with the nrf24l01, used commonly when dynamic payloads is disabled
         */
        inline constexpr size_t max_buffer_size = 32;
        inline constexpr u8 fifo_empty_size = 0;


        /** 
         * @brief Drops the voltage down on the CE line
         * 
         * @return void
         */
        void drop_ce_pin() const;/** Drops CE pin so configuration can be executed */

        /** 
         * @brief Raises the voltage down on the CE line
         * 
         * @return void
         */
        void raise_ce_pin() const;



        /** 
         * @brief Clears the RX_DR status flag
         * 
         * @return void
         */
        void clear_RxDR() const ;

        /** 
         * @brief Reads the FIFO Status and checks if there is data in the rx buffer.
         * 
         * @return bool
         * @retval true if data is inside the rx fifo
         * @retval false if there is no data in the rx fifo
         */
        bool check_rx_buffer_has_data();


        /**
         * @brief gets the size of the current data inside the rx fifo
         * 
         * @return const int - size of data in rx buffer
         */

        const u8 get_rx_size()const ;


        /**
         * @brief reads the data inside the fifo buffer and puts it inside databuffer parameter.
         * 
         * @param databuffer buffer passed to the method which the data inside the NRF rx buffer will be written to
         * @param data_bytes_length length of data expected to be read
         * 
         * @return bool
         * @retval true if success
         * @retval false if errorneous
         */
        const bool read_rx_payload(u8* databuffer, const u8& data_bytes_length) const;

        bool change_antenna_mode(const Antenna_Mode& rx_mode);

        /**
         * 
         */
        const u8* read_register(const u8& register_address, const u8& expected_data_length) const;

        /**
         * @brief function which will use spi to write data to a given register address
         * 
         * @param register_address which register to write data to
         * @param data_bytes_length length of data bytes to write
         * @param databytes data to write
         * 
         * @retval unsigned int 8 bit pointer
         * @return pointer to reponse from SPI
         */

        const u8* write_register(const u8& register_address, const u8& data_bytes_length, const u8* databytes) const ;


        /**
         * @brief wrapper for register command instructions, pure instruction commands. Handles errors and correct buffer sizes
         * 
         * @param register_Address address to modify
         * @param data_bytes_length length of data bytes to be executed
         * @param databytes pointer to data to execute
         * 
         * @retval bool
         * @return True if Successful
         * @return false if errorneous
         */
        const bool spi_command_wrapper(const u8& register_address, const u8& data_bytes_length, const u8* databytes)const;



        /**
         * @brief Configures and initilaises the default registers
         * 
         * @param antenna_mode desired mode for the radio to enter, transmit or recieve
         * 
         * @return bool
         * @retval true if succesfull
         * @retval false if error
         */
        const bool setup_config(const Antenna_Mode& antenna_mode );



        /**
         * @brief changes the antennas current mode to transmit
         * 
         *  @return bool
         * @retval true if executed succesfully
         * @retval false if execution fails
         */

        bool switch_to_transmit();

        /**
         * @brief Pulses the CE pin voltage high and then brings it back down
         *  
         * @return void
         */

        void pulse_ce() const;
       
        /** 
         * @brief Flushes data from the component's tx buffer
         * 
         * @return void
         */
        void flush_tx_buffer() const;

        /** 
         * @brief Flushes data from the component's rx buffer
         * 
         * @return void
         */
        void flush_rx_buffer() const;


        bool write_spi_command(u8* transmit_buffer, u8* recieve_buffer, u8 buffer_length);
    public: 

        static constexpr u8 fifo_max_size = 32;

        void rx_process(u8* rx_buffer);
        void dump_all_registers();

    
        bool switch_to_recieve();
        Antenna_Mode mode_;

        u8 get_status();

        void clear_rx();
        

     

        
        
        NRF24(const spi_object& spi, const Pins_T& pins, u8 buffer_size);

        bool transmit_data(u8* databuffer, u8 data_bytes_length);





};




namespace NRF_regs{
    inline constexpr u8 config_register_address = 0x00;
    inline constexpr u8 auto_acknowledge_config_address = 0x01;
    inline constexpr u8 enable_rx_pipes_address = 0x02;
    inline constexpr u8 address_width_address = 0x03;
    inline constexpr u8 retransmit_details_address = 0x04;

    inline constexpr u8 frequency_register_address = 0x05;
    inline constexpr u8 rf_setup_address = 0x06;
    inline constexpr u8 status_register_address = 0x07;

    inline constexpr u8 rx_pipe_zero_address = 0x0A;
    inline constexpr u8 tx_pipe_zero_address = 0x10;   
    inline constexpr u8 rx_width_Address = 0x11;

    inline constexpr u8 dynamic_payload_address = 0x1C;
    inline constexpr u8 features_address = 0x1D;

    inline constexpr u8 fifo_status_address = 0x17;
    
}

namespace commands{
    inline constexpr u8 read_rx_buffer_command =0x61;
    inline constexpr u8 flush_tx_command =0xE1;
    inline constexpr u8 flush_rx_command =0xE2;
    inline constexpr u8 write_tx_command =0xA0;
    inline constexpr u8 get_rx_size_command = 0x60;
}

enum class voltage_flow :int{low = 0, high = 1};