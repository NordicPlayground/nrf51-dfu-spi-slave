#include "spi_slave_com_interface.h"
#include "spi_slave.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "nrf_drv_gpiote.h"
#include "crc16.h"
#include <string.h>

static spi_slave_com_pck_t m_spis_com_tx_packet, m_spis_com_rx_packet;
static uint8_t m_spis_com_rx_data_buffer[SPI_COM_MAX_PACKET_LENGTH];
static uint32_t m_spis_com_rx_fragment_counter = 0;
static uint8_t  m_spis_com_rx_packet_previous_seq_num = 0xFF;
static bool m_crc_ok = true;
static spi_slave_com_event_t m_spis_event;


static uint32_t m_spis_com_pin_rdyn, m_spis_com_pin_reqn;

static spi_slave_com_callback_t m_callback;

void spi_slave_event_handler(spi_slave_evt_t event)
{
    uint32_t pck_length, pck_seq_num;
    uint16_t pck_crc, calculated_crc;
    
    switch(event.evt_type)
    {
        /**< Memory buffer set event. Memory buffers have been set successfully to the SPI slave device and SPI transactions can be done. */
        case SPI_SLAVE_BUFFERS_SET_DONE:
            nrf_gpio_pin_clear(m_spis_com_pin_rdyn);
            break;
        
        case SPI_SLAVE_XFER_DONE:
            nrf_gpio_pin_set(m_spis_com_pin_rdyn);
            pck_length  = (uint32_t)m_spis_com_rx_packet.fields.len_lsb | (uint32_t)m_spis_com_rx_packet.fields.len_msb << 8;
            pck_seq_num = (uint32_t)m_spis_com_rx_packet.fields.seq_nr;
            pck_crc     = (uint16_t)m_spis_com_rx_packet.fields.crc_lsb | (uint16_t)m_spis_com_rx_packet.fields.crc_msb << 8;
            m_spis_com_rx_packet.fields.crc_lsb = m_spis_com_rx_packet.fields.crc_msb = 0;
            calculated_crc = crc16_compute(m_spis_com_rx_packet.data, SPI_COM_MAX_TRANSACTION_SIZE, 0);
            if(calculated_crc != pck_crc) 
            {
                m_crc_ok = false;
            }
        
            if(pck_seq_num != m_spis_com_rx_packet_previous_seq_num)
            {
                m_spis_com_rx_fragment_counter = 0;
                m_spis_com_rx_packet_previous_seq_num = pck_seq_num;
            }
            else 
            {
                m_spis_com_rx_fragment_counter++;
            }
            
            if(pck_length <= SPI_COM_MAX_PACKET_LENGTH && (m_spis_com_rx_fragment_counter * SPI_COM_PCK_DATA_SIZE) < SPI_COM_MAX_PACKET_LENGTH)
            {
                if((pck_length - m_spis_com_rx_fragment_counter * SPI_COM_PCK_DATA_SIZE) > SPI_COM_PCK_DATA_SIZE)
                {
                    memcpy(m_spis_com_rx_data_buffer + m_spis_com_rx_fragment_counter * SPI_COM_PCK_DATA_SIZE, m_spis_com_rx_packet.fields.data, SPI_COM_PCK_DATA_SIZE);
                }
                else
                {
                    memcpy(m_spis_com_rx_data_buffer + m_spis_com_rx_fragment_counter * SPI_COM_PCK_DATA_SIZE, m_spis_com_rx_packet.fields.data, pck_length - m_spis_com_rx_fragment_counter * SPI_COM_PCK_DATA_SIZE);
                }
            }
            if(((m_spis_com_rx_fragment_counter + 1) * SPI_COM_PCK_DATA_SIZE) >= pck_length)
            {
                m_spis_event.type = SPI_SLAVE_COM_EVENT_RX;
                m_spis_event.p_data = m_spis_com_rx_data_buffer;
                m_spis_event.data_length = pck_length;
                //m_dfu_buf_length = pck_length;
                m_spis_event.id = m_spis_com_rx_packet.fields.id;
                m_spis_event.crc_ok = m_crc_ok;
                m_crc_ok = true;
                if(pck_length > 128)
                {
                    NRF_GPIO->OUTSET = 0;
                }
                if(m_callback)
                {
                    m_callback(&m_spis_event);
                }              
            }

            break;
        default:
            break;
    }
}

static void gpiote_spi_slave_event_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    uint32_t err_code;
    if(pin == m_spis_com_pin_reqn)
    {
        if (nrf_drv_gpiote_in_is_set(pin))
        {
        }
        else
        {
            err_code = spi_slave_buffers_set(m_spis_com_tx_packet.data, m_spis_com_rx_packet.data, SPI_COM_MAX_TRANSACTION_SIZE, SPI_COM_MAX_TRANSACTION_SIZE);
            APP_ERROR_CHECK(err_code);           
        }
    }
}

uint32_t spi_slave_com_init(spi_slave_com_init_t *spi_slave_com_init)
{
    uint32_t err_code; 

    spi_slave_config_t spi_slave_config;
    spi_slave_config.pin_sck = spi_slave_com_init->pin_sck;
    spi_slave_config.pin_mosi = spi_slave_com_init->pin_mosi;
    spi_slave_config.pin_miso = spi_slave_com_init->pin_miso;
    spi_slave_config.pin_csn = spi_slave_com_init->pin_cs;    
    spi_slave_config.mode = SPI_MODE_0;
    spi_slave_config.bit_order = SPIM_MSB_FIRST;
    spi_slave_config.def_tx_character = 0xFF;
    spi_slave_config.orc_tx_character = 0xFF;
    err_code = spi_slave_init(&spi_slave_config);
    APP_ERROR_CHECK(err_code);
    
    m_callback = spi_slave_com_init->callback;
    
    m_spis_com_pin_rdyn = spi_slave_com_init->pin_rdyn;
    m_spis_com_pin_reqn = spi_slave_com_init->pin_reqn;
 
    nrf_gpio_cfg_output(m_spis_com_pin_rdyn);
    nrf_gpio_pin_set(m_spis_com_pin_rdyn);
    
    if (!nrf_drv_gpiote_is_init())
    {
        err_code = nrf_drv_gpiote_init();
        if (err_code != NRF_SUCCESS)
        {
            return err_code;
        }
    }  
    
    nrf_drv_gpiote_in_config_t cts_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(false);
    err_code = nrf_drv_gpiote_in_init(m_spis_com_pin_reqn, &cts_config, gpiote_spi_slave_event_handler);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }
    nrf_drv_gpiote_in_event_enable(m_spis_com_pin_reqn, true);
                                           
    // Register callback to be run when commands have been received by the transport layer.
    err_code = spi_slave_evt_handler_register(spi_slave_event_handler);   
    APP_ERROR_CHECK(err_code);
    
    memset(m_spis_com_tx_packet.data, 0, sizeof(m_spis_com_tx_packet));
    
    err_code = spi_slave_buffers_set(m_spis_com_tx_packet.data, m_spis_com_rx_packet.data, SPI_COM_MAX_TRANSACTION_SIZE, SPI_COM_MAX_TRANSACTION_SIZE);
    APP_ERROR_CHECK(err_code);
    return 0;
}

uint32_t spi_slave_com_tx(uint8_t id, uint8_t *p_data, uint32_t data_length)
{
    return 0;
}
