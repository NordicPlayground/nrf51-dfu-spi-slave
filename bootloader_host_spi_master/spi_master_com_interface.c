#include "spi_master_com_interface.h"
#include "spi_master.h"
#include "nrf_gpio.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "crc16.h"
#include <string.h>

// Current TX packet details
static uint8_t *m_spim_com_tx_packet;
static uint32_t m_spim_com_tx_packet_current_seq_num = 0, m_spim_com_tx_packet_length;
static uint32_t m_spim_com_tx_packet_fragment_num = 0;

static spi_master_com_pck_t spim_com_tx_transaction, spim_com_rx_transaction;
static uint32_t m_spim_com_pin_rdyn, m_spim_com_pin_reqn;
static volatile bool m_spi_master_busy = false;

uint16_t calc_crc16(uint8_t *buf, uint32_t length)
{
    return 0;
}

static void send_single_packet()
{
    uint16_t crc;
    uint32_t data_offset;
    
    // Include the length
    spim_com_tx_transaction.fields.len_lsb  = (uint8_t)m_spim_com_tx_packet_length;
    spim_com_tx_transaction.fields.len_msb  = (uint8_t)(m_spim_com_tx_packet_length >> 8);
    
    data_offset = (uint32_t)m_spim_com_tx_packet_fragment_num * SPI_COM_PCK_DATA_SIZE;
    
    if((m_spim_com_tx_packet_length - data_offset) > SPI_COM_PCK_DATA_SIZE)
    {
        memcpy(spim_com_tx_transaction.fields.data, m_spim_com_tx_packet + data_offset, SPI_COM_PCK_DATA_SIZE);
    }
    else
    {
        memcpy(spim_com_tx_transaction.fields.data, m_spim_com_tx_packet + data_offset, m_spim_com_tx_packet_length - data_offset);
        memset(spim_com_tx_transaction.fields.data + (m_spim_com_tx_packet_length - data_offset), 0, SPI_COM_PCK_DATA_SIZE - (m_spim_com_tx_packet_length - data_offset));
    }
    
    // Calculate and include the CRC
    spim_com_tx_transaction.fields.crc_lsb  = 0;
    spim_com_tx_transaction.fields.crc_msb  = 0;
    crc = crc16_compute(spim_com_tx_transaction.data, SPI_COM_MAX_TRANSACTION_SIZE, 0);
    spim_com_tx_transaction.fields.crc_lsb  = (uint8_t)crc;
    spim_com_tx_transaction.fields.crc_msb  = (uint8_t)(crc >> 8);
       
    // Assert the RDY signal
    nrf_gpio_pin_clear(m_spim_com_pin_reqn);
    
    // Wait for the REQ line to be asserted by the slave
    while(nrf_gpio_pin_read(m_spim_com_pin_rdyn) != 0);
    
    // De-assert the RDY signal
    nrf_gpio_pin_set(m_spim_com_pin_reqn);

    m_spim_com_tx_packet_fragment_num++; 
    
    spi_master_send_recv(SPI_MASTER_0, spim_com_tx_transaction.data, SPI_COM_MAX_TRANSACTION_SIZE, spim_com_rx_transaction.data, SPI_COM_MAX_TRANSACTION_SIZE);  
}

void spi_master_event_handler(spi_master_evt_t spi_master_evt)
{
    switch(spi_master_evt.evt_type)
    {
        case SPI_MASTER_EVT_TRANSFER_STARTED:
            break;
        
        case SPI_MASTER_EVT_TRANSFER_COMPLETED:
            if((m_spim_com_tx_packet_fragment_num * SPI_COM_PCK_DATA_SIZE) >= m_spim_com_tx_packet_length)
            {
                // Increment the sequence number in time for the next packet
                m_spim_com_tx_packet_current_seq_num++;
                
                // Transaction complete, clear busy flag
                m_spi_master_busy = false;
            }
            else
            {
                // Transaction incomplete, send the next fragment
                send_single_packet();
            }
            break;
            
        case SPI_MASTER_EVT_TYPE_MAX:
            break;
    }
}
uint32_t spi_master_com_init(spi_master_com_init_t *init)
{
    spi_master_config_t spim_cfg = {.SPI_Freq = SPI_FREQUENCY_FREQUENCY_M2,
                                    .SPI_Pin_SCK = init->pin_sck, 
                                    .SPI_Pin_MISO = init->pin_miso, 
                                    .SPI_Pin_MOSI = init->pin_mosi, 
                                    .SPI_Pin_SS = init->pin_cs, 
                                    .SPI_PriorityIRQ = 3,
                                    .SPI_CONFIG_ORDER = 0, 
                                    .SPI_CONFIG_CPOL = 0, 
                                    .SPI_CONFIG_CPHA = 0,   
                                    .SPI_DisableAllIRQ = 0};
    
    m_spim_com_pin_rdyn = init->pin_rdyn;
    m_spim_com_pin_reqn= init->pin_reqn;
    
    nrf_gpio_cfg_output(m_spim_com_pin_reqn);
    nrf_gpio_pin_set(m_spim_com_pin_reqn);
    nrf_gpio_cfg_input(m_spim_com_pin_rdyn, NRF_GPIO_PIN_NOPULL);  

    nrf_delay_ms(2);
                                    
    spi_master_open(SPI_MASTER_0, &spim_cfg);
    spi_master_evt_handler_reg(SPI_MASTER_0, spi_master_event_handler);
                                    
    return 0;
}

uint32_t spi_master_com_tx(uint8_t id, uint8_t *p_data, uint32_t data_length)
{
    
    if(data_length > SPI_COM_MAX_PACKET_LENGTH) return 0xFFFFFFFF;
    
    while(m_spi_master_busy);
      
    m_spi_master_busy = true;
    
    m_spim_com_tx_packet = p_data;
    m_spim_com_tx_packet_fragment_num = 0;
    m_spim_com_tx_packet_length = data_length;
    spim_com_tx_transaction.fields.id = id;
    spim_com_tx_transaction.fields.seq_nr = m_spim_com_tx_packet_current_seq_num;
    
    send_single_packet();
    
    return 0;
}

bool spi_master_com_busy(void)
{
    return m_spi_master_busy;
}
