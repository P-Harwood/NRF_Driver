#include "nRF24L01P.hpp"



// Constructor with logging
NRF24::NRF24(const spi_object& spi, const Pins_T& pins, u8 buffer_size):
    spi_(spi), pins_layout(pins_data), buffer_size_(buffer_size)
{
   
    drop_ce_pin();
    setup_config(Antenna_Mode::Recieve);
    raise_ce_pin();



    vTaskDelay(pdMS_TO_TICKS(5));
    //leave_standby();
    printf("[NRF24] Initialization complete\n");
}



const bool NRF24::spi_command_wrapper(const u8& register_address, const u8& data_bytes_length, const u8* databytes)const{

    const u8 full_buffer_size = data_bytes_length + sizeof(register_address);
    const u8 *return_buffer = write_register(register_address, data_bytes_length, databytes);

    if(return_buffer!= nullptr){
        printf("[NRF24] Write_register to address: %d returned: 0x%02X\n",register_address, return_buffer[0]);
        memset(return_buffer, 0x00, full_buffer_size);
        free(return_buffer);
        return true;
    } else {
        printf("[NRF24] Error: write_register returned nullptr to address: %d\n",register_address);
        return false;
    }
}


bool NRF24::change_antenna_mode(const Antenna_Mode& requested_mode){
    u8 config = 0;
    printf("[NRF24::change_antenna_mode] Changing register 0x00 \n");
    if(requested_mode == Antenna_Mode::Recieve){
        printf("[NRF24::change_antenna_mode] Setting config to recieve\n");
        config =  0b00000011;
    }else if (requested_mode == Antenna_Mode::Transmit){
        printf("[NRF24::change_antenna_mode] Setting config to transmit\n");
        config = 0b00000010;
    } else{
        return false;
    }

    const u8 config_register_address = 0x00;

    bool antenna_mode_changed_successfully = spi_command_wrapper(config_register_address, sizeof(config), &config);
    return antenna_mode_changed_successfully;
}


const bool NRF24::setup_config(const Antenna_Mode& antenna_mode) {
    // 1. Ensure CE LOW

    // 2. CONFIG: power up & PRIM_RX (1=RX, 0=TX)
    u8 config_register_values =  0b00000011;  // PWR_UP=1, PRIM_RX=1 (RX mode)
    spi_command_wrapper(NRF_regs::config_register_address, sizeof(config_register_values), &config_register_values);
    vTaskDelay(pdMS_TO_TICKS(200));  // allow oscillator to stabilize


    // 3. Address width & retries
    constexpr u8 address_width_values = 0b01;  // 5-byte addresses
    spi_command_wrapper(NRF_regs::address_width_address, sizeof(address_width_values), &address_width_values);   // SETUP_AW

    constexpr u8 retransmission_values = 0xF3;  // 1500us, 3 retries
    spi_command_wrapper(NRF_regs::retransmit_details_address, sizeof(retransmission_values), &retransmission_values); // SETUP_RETR

    // 4. Addresses

    constexpr u8 address_width = 3;// must align with address_width_values, however their literal values will not match
    constexpr u8 pipe_zero_values[address_width] = {0x03,0x03,0x03}; // experimenting (tx is odd and seems prone to changing)

    spi_command_wrapper(NRF_regs::rx_pipe_zero_address, address_width, pipe_zero_values);
    spi_command_wrapper(NRF_regs::tx_pipe_zero_address, address_width, pipe_zero_values);

    // 5. RF setup
    constexpr u8 rf_setup_values = 0b00100100;  // 1Mbps, 0dBm
    spi_command_wrapper(NRF_regs::rf_setup_address, sizeof(rf_setup_values), &rf_setup_values);  // RF_SETUP

    constexpr u8 frequency_channel = 0x02;
    spi_command_wrapper(NRF_regs::frequency_register_address, sizeof(frequency_channel), &frequency_channel);      // RF_CH



    /*

    // 6. Feature & dynamic payload
    constexpr u8 features_address = 0x1D;
    constexpr u8 features_values = 0b00000100;   // enable DPL
    spi_command_wrapper(features_address, sizeof(features_values), &features_values);       // FEATURE

    constexpr u8 dynamic_payload_address = 0x1C;    // pipe0 DPL
    constexpr u8 dynamic_payload_value = 0b00000001;    // pipe0 DPL
    spi_command_wrapper(dynamic_payload_address, sizeof(dynamic_payload_value), &dynamic_payload_value);        // DYNPD
    */

    // 6. Disable dynamic payload & features
    constexpr u8 features_values = 0x00;   // Disable EN_DPL, EN_ACK_PAY, EN_DYN_ACK
    spi_command_wrapper(NRF_regs::features_address, sizeof(features_values), &features_values);

    constexpr u8 dynamic_payload_value = 0x00;      // Disable DPL for all pipes
    spi_command_wrapper(NRF_regs::dynamic_payload_address, sizeof(dynamic_payload_value), &dynamic_payload_value);

    constexpr u8 fixed_payload_size = fifo_max_size;  // fixed size in bytes
    spi_command_wrapper(NRF24::rx_width_Address, sizeof(fixed_payload_size), &fixed_payload_size);





    // 7. Enable pipe0, disable auto‑ack
    constexpr u8 enabled_rx_pipes_values = 0b00000011;
    spi_command_wrapper(NRF24::enable_rx_pipes_address, sizeof(enabled_rx_pipes_values), &enabled_rx_pipes_values);      // EN_RXADDR

    constexpr u8 auto_acknowledge_value = 0x00;
    spi_command_wrapper(NRF24::auto_acknowledge_config_address, sizeof(auto_acknowledge_value), &auto_acknowledge_value);     // EN_AA

    // 8. Flush FIFOs & clear interrupts
    // STATUS register bit definitions
    constexpr u8 STATUS_RX_DR = 1 << 6; // Data Ready RX FIFO interrupt
    constexpr u8 STATUS_TX_DS = 1 << 5; // Data Sent TX FIFO interrupt
    constexpr u8 STATUS_MAX_RT = 1 << 4; // Max number of TX retransmits interrupt

    const u8 clear_flags = STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT;
    constexpr u8 status_register_address = 0x07;
    spi_command_wrapper(status_register_address, sizeof(clear_flags), &clear_flags);

    flush_rx_buffer();

    // Set mode for your logic
    mode_ = antenna_mode;
    return true;
}


void NRF24::drop_ce_pin() const{
    gpio_set_level(pins_layout.CE , voltage_flow::low);
    return;
}

void NRF24::raise_ce_pin() const{
    gpio_set_level(pins_layout.CE , voltage_flow::high);
    return;
}








void NRF24::pulse_ce()const {
    printf("\n\nPulsing CE \n\n");
    // pulse to transmit then return to standby
    gpio_set_level(pins_layout.CE , voltage_flow::high);
    ets_delay_us(150);     // pulse ≥10 µs
    gpio_set_level(pins_layout.CE , voltage_flow::low);
    return;
}

void NRF24::flush_tx_buffer() const{
    u8 command_size(1);
    u8 dummy_rx[command_size] = {}; // TODO more inuitive command size
    write_spi_command(&commands::flush_tx_command, dummy_rx, command_size);
    return;
}

void NRF24::flush_rx_buffer() const{
    u8 command_size(1);
    u8 dummy_rx[command_size] = {};
    write_spi_command(flush_rx_command, dummy_rx, command_size);
    return;
}

bool NRF24::transmit_data(u8* databuffer, u8 data_bytes_length) {



    printf("[NRF24::transmit_data] Data to transmit (databuffer): ");
    for (size_t i = 0; i < fifo_max_size; i++) {
        printf("0x%02X ", databuffer[i]);
    }
    printf("\n");

    drop_ce_pin();
    printf("\n\n [NRF24::transmit_data] Starting Transmission \n\n");
    if (databuffer == nullptr) {
        printf("[NRF24::transmit_data] Passed nullptr to transmit_data\n");
        return false;
    }





    printf("[NRF24::transmit_data] Flushing tx buffer\n");
    //flush_tx_buffer();
    printf("[NRF24::transmit_data] Flush complete\n");


    u8 data_packet[fifo_max_size] = {};

    data_packet[0] = commands::write_tx_command; // W_TX_PAYLOAD command
    memcpy(data_packet + sizeof(commands::write_tx_command), databuffer, data_bytes_length);

    printf("[NRF24::transmit_data] Data to transmit: ");
    for (size_t i = 0; i < fifo_max_size; i++) {
        printf("0x%02X ", data_packet[i]);
    }
    printf("\n");


    u8 recieve_data[fifo_max_size] = {};


    printf("[NRF24::transmit_data] Writing spi command to send packet\n");
    if (!write_spi_command(data_packet, recieve_data, fifo_max_size)) {
        printf("failure writing spi command in NRF24::transmit_data\n");
        return false;
    }
    printf("[NRF24::transmit_data]  write_spi_command returned: %s\n", success ? "true" : "false");



    ets_delay_us(200);

    printf("[NRF24::transmit_data]  transmit_data called with %d bytes\n", data_bytes_length);
    if (mode_ != Antenna_Mode::Transmit) {
        if (!switch_to_transmit()) {
            printf("[NRF24::transmit_data]  Error occured trying to change to transmit mode");
            return false;
        }

    }

    printf("[NRF24::transmit_data] Passed antenna mode check \n");

    pulse_ce(); // this pulses CE pin on then off

    // NOP -> STATUS
    {
        constexpr u8 cmd_len = 2;
        u8 cmd[cmd_len] = { 0xFF, 0x00 };
        u8 resp[cmd_len] = {};
        write_spi_command(cmd, resp, cmd_len);
        printf("[DIAG TX] NOP STATUS = 0x%02X\n", resp[0]);
    }

    // FIFO_STATUS (reg 0x17)
    {
        constexpr u8 cmd_len = 2;
        u8 cmd[cmd_len] = { 0x17, 0x00 }; // R_REGISTER 0x17
        u8 resp[cmd_len] = {};
        write_spi_command(cmd, resp, cmd_len);
        printf("[DIAG TX] FIFO_STATUS = 0x%02X\n", resp[1]); // resp[0] is STATUS snapshot, resp[1] is reg value
    }

    // TX_ADDR (reg 0x10, 5 bytes)
    {
        constexpr u8 cmd_len = 6;
        u8 cmd[cmd_len] = { 0x10, 0,0,0,0,0 }; // R_REGISTER 0x10 + 5 dummy
        u8 resp[cmd_len] = {};
        write_spi_command(cmd, resp, cmd_len);
        printf("[DIAG TX] TX_ADDR = %02X %02X %02X %02X %02X\n",
            resp[1], resp[2], resp[3], resp[4], resp[5]);
    }

    // RX_ADDR_P0 (reg 0x0A, 5 bytes)
    {
        constexpr u8 cmd_len = 6;
        u8 cmd[cmd_len] = { 0x0A, 0,0,0,0,0 };
        u8 resp[cmd_len] = {};
        write_spi_command(cmd, resp, cmd_len);
        printf("[DIAG TX] RX_ADDR_P0 = %02X %02X %02X %02X %02X\n",
            resp[1], resp[2], resp[3], resp[4], resp[5]);
    }

    printf("[NRF24::transmit_data] Pulsed CE pin. \n");



    ets_delay_us(200);  // 200 µs gives plenty of time



    constexpr u8 command_size = 2;
    u8 status_command[command_size] = { 0xFF, 0x00 }; // NOP + dummy
    u8 status_response[command_size] = {};

    printf("[NRF24::transmit_data] transmitting check status command \n");

    vTaskDelay(pdMS_TO_TICKS(1)); // tiny settle delay
    bool ok = write_spi_command(status_command, status_response, command_size);


    if (!ok) {
        printf("[transmit_status] SPI failed\n");
        return false;
    }



    u8 status = status_response[0];
    printf("[NRF24::transmit_data] Status after transmission: 0x%02X", status);
    if (status & (1 << 5)) {  // TX_DS
        printf(" Transmission successful\n");
        u8 clear_val = 0b00100000;
        spi_command_wrapper(NRF_regs::status_register_address, 1, &clear_val);
    }
    else if (status & (1 << 4)) { // MAX_RT
        printf(" Transmission failed (MAX_RT)\n");
        u8 clear_val = 0b00010000;
        spi_command_wrapper(NRF_regs::status_register_address, 1, &clear_val);
        flush_tx_buffer();
    }
    else {
        printf(" Transmission still pending or no event yet\n");
    }

    switch_to_receive();
    return true;
}

bool NRF24::switch_to_recieve() {

    

    if (mode_ == Antenna_Mode::Transmit) {
        drop_ce_pin();
        printf("[NRF24::switch_to_recieve] Was in transmit mode, switching to recieve\n");
        if(!change_antenna_mode(Antenna_Mode::Recieve)){
            printf("[NRF24::switch_to_recieve] Unkown error, change antenna returned false? \n");
            return false;
        }
        //clear_rx_buffer();

        mode_ = Antenna_Mode::Recieve;
        printf("[NRF24::switch_to_recieve] Now in RECEIVE mode, CE = HIGH\n");
        raise_ce_pin();
    } else if(mode_ == Antenna_Mode::Recieve){
        printf("[NRF24::switch_to_recieve] Already in RECEIVE mode, no action taken\n");
    } else {
        printf("[NRF24::switch_to_recieve] Unknown mode?? <---- \n\n\n");
    }
    raise_ce_pin();
    return true;
}


bool NRF24::switch_to_transmit(){

    if (mode_ == Antenna_Mode::Recieve){
        drop_ce_pin();
        printf("[NRF24::switch_to_transmit] Was in Recieve mode, switching to transmit\n");
        if(!change_antenna_mode(Antenna_Mode::Transmit)){
            printf("Unkown error, change antenna returned false? \n");
            return false;
        }
        mode_ = Antenna_Mode::Transmit;
    } else  if(mode_ == Antenna_Mode::Transmit){
        printf("[NRF24::switch_to_transmit] Already in TRANSMIT mode, no action taken\n");
    }else {
        printf("[NRF24::switch_to_transmit] Unknown mode?? <---- \n\n\n");
    }
    drop_ce_pin();
    return true;
}





void NRF24::rx_process(u8* return_buffer){


    constexpr u8 clear_all_flags   = 0b01110000; // bits 6,5,4

    spi_command_wrapper(status_register_address,
                        sizeof(clear_all_flags),
                        &clear_all_flags);
        

    auto reset_registers_and_return = [&]() {
        flush_rx_buffer();
        clear_RxDR();
        constexpr u8 clear_all_flags   = 0b01110000; // bits 6,5,4

        spi_command_wrapper(status_register_address,
                            sizeof(clear_all_flags),
                            &clear_all_flags);
            
    };


    switch_to_recieve();

    memset(return_buffer,0x00, fifo_max_size);
    printf("[NRF24::rx_process] Switch to recieve passed\n");
    if(!check_rx_buffer_has_data()){
        reset_registers_and_return();
        return;
    }
    printf("[NRF24::rx_process] Fifo_Status suggests data in rx buffer \n");
    

    /*
    u8 rx_buffer_length = get_rx_size();

    if (rx_buffer_length == fifo_empty_size || rx_buffer_length>fifo_max_size){
        printf("[NRF24::rx_process] Rx_size is out of bounds (%d), Aborting the rx_process method \n", rx_buffer_length);
        reset_registers_and_return();
        return;
    }
    */
    u8 rx_buffer_length = fifo_max_size;
    printf("[NRF24::rx_process] Rx_buffer size has been provided: %d \n", rx_buffer_length);

    if(!read_rx_payload(return_buffer, rx_buffer_length)){
        printf("[NRF24::rx_process] Failure reading rx buffer\n");
        reset_registers_and_return();
        return;
    }
    printf("[NRF24::rx_process]Received payload: ");
    for (u8 i = 0; i < rx_buffer_length; ++i) {
        printf("%02x", return_buffer[i]);
    }
    printf("\n");

    drop_ce_pin();
    reset_registers_and_return();
    raise_ce_pin();
    return;
}

void NRF24::clear_RxDR() const{
    printf("[NRF24::clear_RxDR] Clearing RX_DR flag\n");
    u8 clear = 0x40;
    bool write_status = spi_command_wrapper(status_register_address, sizeof(clear), &clear);
    return;
}

const u8 NRF24::get_rx_size()const{



    constexpr u8 command_size = 2; 
    u8 check_rx_data_size_command[command_size] = {get_rx_size_command,0x00};
    u8 fifo_data[command_size] ={};

    gpio_set_level(pins_layout.CSN, voltage_flow::high);
    vTaskDelay(pdMS_TO_TICKS(5));
    bool check_status = write_spi_command(check_rx_data_size_command, fifo_data, command_size);
    gpio_set_level(pins_layout.CSN, voltage_flow::low);

    if(!check_status){
        printf("[NRF24::get_rx_size] Failed trying to check rx fifo data size \n");
        return 0;
    }
    if(fifo_data[1] > fifo_max_size || fifo_data[1] ==fifo_empty_size ){
        flush_rx_buffer();
        printf("[NRF24::get_rx_size] Fifo size out of range: %d\n", fifo_data[1]);
        printf("[NRF24::get_rx_size] Returned Data: 0x%02X, 0x%02X \n", fifo_data[0], fifo_data[1]);
        return 0;
    }


    return fifo_data[1];
}


bool NRF24::check_rx_buffer_has_data() {
    //printf("\n\nChecking the rx buffer\n\n");
    // Prepare the “FIFO_STATUS” register read command
    u8 command_data_size = 1;
    u8* fifo_data = nullptr;

    //printf("[NRF24] check_rx_fifo: about to send command 0x17 to read FIFO_STATUS\n");
    //printf("[NRF24] check_rx_fifo: transmit buffer: 0x%02X 0x%02X\n",
    //       fifo_check_command[0], fifo_check_command[1]);




    // Issue the SPI transaction
    fifo_data = read_register(fifo_status_address,
        command_data_size);


    if (fifo_data == nullptr) {
        printf("[NRF24] check_rx_fifo: SPI transaction FAILED\n");
        return false;
    }

    // Log the raw SPI response
    printf("[NRF24::check_rx_buffer_has_data] check_rx_fifo: SPI transaction successful, received buffer: ");
    printf("0x%02X 0x%02X\n", fifo_data[0], *(fifo_data+1));

    // fifo_data[0] is the STATUS byte, fifo_data[1] is the FIFO_STATUS register
    bool rx_fifo_empty = (fifo_data[1] & 0x01);
    printf("[NRF24::check_rx_buffer_has_data]  Fifo Data byte 0 : (0x%02X)\n", fifo_data[0]);
    printf("[NRF24::check_rx_buffer_has_data]  Fifo Data byte 1 : (0x%02X)\n", fifo_data[1]);

    free(fifo_data);
    if (rx_fifo_empty) {
        printf("[NRF24] check_rx_fifo: no data available (RX FIFO empty)\n");
       return false;
    }

    
    //printf("[NRF24] check_rx_fifo: data available in RX FIFO!\n");
    return true;
}

const bool NRF24::read_rx_payload( u8* databuffer, const u8& data_bytes_length)const {


    u8 transmit_data[max_buffer_size] = {};
    u8 receive_data[max_buffer_size] = {};


    transmit_data[0] = read_rx_buffer_command;
 
    memset(transmit_data + sizeof(read_rx_buffer_command), 0x00, data_bytes_length); // TODO : check why i put oxff?

    drop_ce_pin();
    bool success = write_spi_command(transmit_data, receive_data, max_buffer_size);
    raise_ce_pin();

    if (!success) {
        printf("[NRF24] SPI read RX payload failed\n");
        return false;
    }

    printf("[NRF24::read_rx_payload] Raw RX buffer: ");
        for (u8 i = 0; i < 1 + data_bytes_length; ++i) {
            printf("%02X ", receive_data[i]);
        }
    printf("\n");


    memcpy(databuffer, receive_data + 1, data_bytes_length); // skip status byte
    return true;
}

u8* NRF24::read_register(const u8& register_address, const u8& data_bytes_length) const{
    if (data_bytes_length == 0) {
        printf("[NRF24] Error: data_bytes_length must be > 0\n");
        return nullptr;
    }

    const u8 full_buffer_size = sizeof(register_address) + data_bytes_length;

    printf("\n \n Creating buffer of size: %d \n", full_buffer_size);

    u8* rx_buffer = (u8*)malloc(full_buffer_size);
    if (rx_buffer == nullptr) {
        printf("[NRF24] Memory allocation failed in read_register\n");
        return nullptr;
    }
    u8* tx_buffer = (u8*)malloc(full_buffer_size);
    if (tx_buffer == nullptr) {
        printf("[NRF24] Memory allocation failed in read_register\n");
        return nullptr;
    }

    tx_buffer[0] = register_address; // Read command – MSBs should already be 0

    // Fill rest of transmit_data with 0x00 (dummy bytes for reading)
    memset(tx_buffer + sizeof(uint8_t), 0x00, data_bytes_length);
    
    printf("[NRF24] read_register called: reg=0x%02X, len=%d, cmd=", register_address, data_bytes_length);
    for (u8 i = 0; i < full_buffer_size; ++i) {
        printf("0x%02X ", tx_buffer[i]);
    }
    printf("\n");
    
    gpio_set_level(pins_layout.CSN, voltage_flow::low);
    

    vTaskDelay(pdMS_TO_TICKS(5));
    bool result = write_spi_command(tx_buffer, rx_buffer, full_buffer_size);
    gpio_set_level(pins_layout.CSN, voltage_flow::high);

    printf("Pin after attempting to Raise it: ");
    if (gpio_get_level(pins_layout.CSN) == voltage_flow::high) {
        printf("Pin is HIGH\n");
    } else {
        printf("Pin is LOW\n");
    }


    //printf("[NRF24] write_spi_command result: %s\n", result ? "true" : "false");
    free(tx_buffer);
    if (!result) {
        free(rx_buffer);
        return nullptr;
    }
    
    printf("[NRF24] SPI read returned: ");
    for (u8 i = 0; i < full_buffer_size; ++i) {
        printf("0x%02X ", rx_buffer[i]);
    }

    printf("\n\n");
    return rx_buffer;
}




const u8* NRF24::write_register(const u8& register_address, const u8& data_bytes_length, const u8* databytes) const {


    printf("[NRF24::write_register] write_register called: reg=0x%02X, len=%d, data=0x", register_address, data_bytes_length);
    for (u8 i = 0; i < data_bytes_length; ++i) {
        printf("%02X ", databytes[i]);
    }
    printf("\n");

    u8 full_buffer_size = data_bytes_length+sizeof(register_address);

    u8* rx_buffer = (u8*)malloc(full_buffer_size);

    if(rx_buffer == nullptr){
        printf("[NRF24::write_register] failed allocating memory for return_buffer in NRF::24.write_register(). \n");
        return nullptr;
    }
    

    u8 command_data[max_buffer_size] = {};


    u8 write_register_prefix = 0x20;

    command_data[0] = write_register_prefix | register_address;//TODO : consider changing this, it limits register address but can create obsfucated bugs
    memcpy(command_data + sizeof(write_register_prefix), databytes, data_bytes_length);//TODO Last byte likely ignored

    bool result = write_spi_command(command_data, rx_buffer, full_buffer_size);
    printf("[NRF24::write_register] write_spi_command result: %s\n", result ? "true" : "false");

    if(!result){
        printf("[NRF24::write_register] Returning null buffer from Write Register.\n");
        free(rx_buffer);
        return nullptr;
    }

    return rx_buffer;

}


void NRF24::clear_rx(){
    drop_ce_pin();

    u8 command_size(1);
    u8 dummy_rx[command_size] = {};

    write_spi_command(commands::flush_rx_command, dummy_rx, command_size);

    u8 clear = 0x40;
    spi_command_wrapper(NRF_regs::status_register_address, sizeof(clear), &clear);


    raise_ce_pin(); // TODO : Return ce to original state
    return;
}







bool NRF24::write_spi_command(u8* transmit_buffer, u8* recieve_buffer, u8 buffer_length) {
    // printf("[NRF24] write_spi_command called with buffer_length: %d\n", buffer_length);

    if (buffer_length == 0 || !transmit_buffer) {
        printf("[NRF24::write_spi_command] Error: invalid buffer_length (%d) or transmit_buffer is null\n", buffer_length);
        return false;
    }
    esp_err_t result = spi_->send_data(buffer_length, transmit_buffer, recieve_buffer);


    if (result != ESP_OK) {
        printf("[NRF24::write_spi_command] SPI transfer failed with error: 0x%x\n", result);
        return false;
    }
    return true;
}

u8 NRF24::get_status(){
    constexpr u8 command_size = 1;
    u8 nop_command[command_size] = {0xFF};
    u8 status_response[command_size] = {};

    bool ok = write_spi_command(nop_command, status_response, command_size);
    if (!ok) {
        printf("[NRF24::get_status] SPI transaction failed\n");
        return 0xFF; // invalid status
    }

    printf("[NRF24::get_status] STATUS = 0x%02X\n", status_response[0]);
    return status_response[0];
}


void NRF24::dump_all_registers() {
    printf("=================================\n");
    printf("=================================\n");
    printf("=================================\n");
    static const struct {
        const char* name;
        u8 addr;
        u8 len;
    } regs[] = {
        { "CONFIG",       0x00, 1 },
        { "EN_AA",        0x01, 1 },
        { "EN_RXADDR",    0x02, 1 },
        { "SETUP_AW",     0x03, 1 },
        { "SETUP_RETR",   0x04, 1 },
        { "RF_CH",        0x05, 1 },
        { "RF_SETUP",     0x06, 1 },
        { "STATUS",       0x07, 1 },
        { "OBSERVE_TX",   0x08, 1 },
        { "RPD",          0x09, 1 },
        { "RX_ADDR_P0",   0x0A, 5 },
        { "RX_ADDR_P1",   0x0B, 5 },
        { "RX_ADDR_P2",   0x0C, 1 },
        { "RX_ADDR_P3",   0x0D, 1 },
        { "RX_ADDR_P4",   0x0E, 1 },
        { "RX_ADDR_P5",   0x0F, 1 },
        { "TX_ADDR",      0x10, 5 },
        { "RX_PW_P0",     0x11, 1 },
        { "RX_PW_P1",     0x12, 1 },
        { "RX_PW_P2",     0x13, 1 },
        { "RX_PW_P3",     0x14, 1 },
        { "RX_PW_P4",     0x15, 1 },
        { "RX_PW_P5",     0x16, 1 },
        { "FIFO_STATUS",  0x17, 1 },
        { "DYNPD",        0x1C, 1 },
        { "FEATURE",      0x1D, 1 },
    };

    printf("==== NRF24L01+ Register Dump ====\n");
    printf("(First byte is STATUS; following are the actual register bytes.)\n\n");

    for (const auto& r : regs) {
        u8* buf = read_register(r.addr, r.len);
        if (!buf) {
            printf("%-12s (0x%02X): <ERR>\n", r.name, r.addr);
            continue;
        }

        // buf[0] = STATUS, buf[1..r.len] = real register data
        printf("%-12s (0x%02X): STATUS=0x%02X, DATA=", r.name, r.addr, buf[0]);
        for (int i = 0; i < r.len; ++i) {
            printf(" 0x%02X", buf[1 + i]);
        }
        printf("\n");
    
        free(buf);
    }
    printf("=================================\n");
    printf("=================================\n");
    printf("=================================\n");
    return;
}
