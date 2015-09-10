#include "dfu_spi_master_lib.h"
//#include "spi_master.h"
#include "spi_master_com_interface.h"
#include <string.h>

#define SPI_MASTER_TX_BUF_SIZE      144

uint16_t softdevice_list_default[] = {0xFFFE};

static uint8_t m_spi_master_tx_buffer[SPI_MASTER_TX_BUF_SIZE];

void spi_init()
{
    spi_master_com_init_t spi_com_init;
    spi_com_init.pin_sck = 1;
    spi_com_init.pin_miso = 2;
    spi_com_init.pin_mosi = 3;
    spi_com_init.pin_cs = 4;
    spi_com_init.pin_reqn = 5;
    spi_com_init.pin_rdyn = 6;
    spi_master_com_init(&spi_com_init);
}

void set_header_byte(uint8_t *buf, uint8_t header_byte)
{
    buf[0] = header_byte;
    buf[1] = buf[2] = buf[3] = 0;
}

uint32_t dfu_spi_init(void)
{
	spi_init(); 
    return 0;    
}
 
uint32_t dfu_spi_send_start_packet(dfu_start_packet_t *packet)
{
    while(spi_master_com_busy());
    
    //set_header_byte(m_spi_master_tx_buffer, START_PACKET);
    memcpy(m_spi_master_tx_buffer + 0, packet, 16);
    
    //return spi_master_send_recv(SPI_MASTER_0, m_spi_master_tx_buffer, 20, 0, 0);
    return spi_master_com_tx(0x80 + START_PACKET, m_spi_master_tx_buffer, 16);
}

uint32_t dfu_spi_send_init_packet(dfu_init_packet_t *init_packet)
{
    while(spi_master_com_busy());
    
    uint32_t sd_num = init_packet->valid_softdevice_list_length;
    //set_header_byte(m_spi_master_tx_buffer, INIT_PACKET);
    memcpy(m_spi_master_tx_buffer + 0, init_packet, 10);
    memcpy(m_spi_master_tx_buffer + 10, init_packet->valid_softdevice_list, sd_num * 2);
    memcpy(m_spi_master_tx_buffer + 10 + sd_num * 2, (uint8_t*)init_packet + 16, 4);
    
    //return spi_master_send_recv(SPI_MASTER_0, m_spi_master_tx_buffer, 18 + sd_num * 2, 0, 0);
    return spi_master_com_tx(0x80 + INIT_PACKET, m_spi_master_tx_buffer, 14 + sd_num * 2);
}

uint32_t dfu_spi_send_data_packet(uint8_t *data_ptr, uint32_t data_length)
{
    if(data_length > 128) return 0xFFFFFFFF;
    if((data_length % 4) != 0) return 0xFFFFFFFF;
    
    while(spi_master_com_busy());  

    //set_header_byte(m_spi_master_tx_buffer, DATA_PACKET);
    memcpy(m_spi_master_tx_buffer + 0, data_ptr, data_length);
    
    //return spi_master_send_recv(SPI_MASTER_0, m_spi_master_tx_buffer, 4 + data_length, 0, 0);    
    return spi_master_com_tx(0x80 + DATA_PACKET, m_spi_master_tx_buffer, 0 + data_length);
}
 
uint32_t dfu_spi_send_stop_packet()
{
    while(spi_master_com_busy());   

    //set_header_byte(m_spi_master_tx_buffer, STOP_DATA_PACKET);
    
    //return spi_master_send_recv(SPI_MASTER_0, m_spi_master_tx_buffer, 4, 0, 0);   
    return spi_master_com_tx(0x80 + STOP_DATA_PACKET, m_spi_master_tx_buffer, 0);
}
