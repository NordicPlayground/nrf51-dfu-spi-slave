/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include "dfu_transport.h"
#include <stddef.h>
#include <string.h>
#include "dfu.h"
#include <dfu_types.h>
#include "app_error.h"
#include "app_util.h"
#include "hci_transport.h"
#include "app_timer.h"
#include "app_scheduler.h"
//#include "spi_slave.h"
#include "spi_slave_com_interface.h"
#include "nrf_gpio.h"
#include "bsp.h"

#define MAX_BUFFERS          4u                                                      /**< Maximum number of buffers that can be received queued without being consumed. */

/**
 * defgroup Data Packet Queue Access Operation Macros
 * @{
 */

/** Provides status showing if the queue is full or not. */
#define DATA_QUEUE_FULL()                                                                         \
        (((MAX_BUFFERS -1) == m_data_queue.count) ? true : false)

/** Provides status showing if the queue is empty or not */
#define DATA_QUEUE_EMPTY()                                                                        \
        ((0 == m_data_queue.count) ? true : false)

/** Initializes an element of the data queue. */
#define DATA_QUEUE_ELEMENT_INIT(i)                                                                \
        m_data_queue.data_packet[(i)].packet_type = INVALID_PACKET

/** Sets the packet type of an element of the data queue. */
#define DATA_QUEUE_ELEMENT_SET_PTYPE(i, t)                                                         \
        m_data_queue.data_packet[(i)].packet_type = (t)

/** Copies a data packet pointer of an element of the data queue. */
#define DATA_QUEUE_ELEMENT_COPY_PDATA(i, dp)                                                       \
        m_data_queue.data_packet[(i)].params.data_packet.p_data_packet = (uint32_t *)(dp)

/** Sets the packet length of an element in the data queue. */
#define DATA_QUEUE_ELEMENT_SET_PLEN(i, l)                                                          \
        m_data_queue.data_packet[(i)].params.data_packet.packet_length = (l)

/** Gets a data packet pointer of an element in the data queue. */
#define DATA_QUEUE_ELEMENT_GET_PDATA(i)                                                           \
        (m_data_queue.data_packet[(i)].params.data_packet.p_data_packet)

/** Gets the packet type of an element in the data queue. */
#define DATA_QUEUE_ELEMENT_GET_PTYPE(i)                                                           \
        m_data_queue.data_packet[(i)].packet_type

/* @} */

#define                         SPI_SLAVE_RX_BUF_LENGTH 196
static uint8_t                  m_dfu_buf[SPI_SLAVE_RX_BUF_LENGTH];
static uint32_t                 m_dfu_buf_length;
static uint8_t                  m_dfu_packet_type;

/**@brief       Function for handling the callback events from the dfu module.
 *              Callbacks are expected when \ref dfu_data_pkt_handle has been executed.
 *
 * @param[in]   packet  Packet type for which this callback is related. START_PACKET, DATA_PACKET.
 * @param[in]   result  Operation result code. NRF_SUCCESS when a queued operation was successful.
 * @param[in]   p_data  Pointer to the data to which the operation is related.
 */
static void dfu_cb_handler(uint32_t packet, uint32_t result, uint8_t * p_data)
{
    APP_ERROR_CHECK(result);
}


static void process_dfu_packet(void * p_event_data, uint16_t event_size)
{
    uint32_t              retval;
    dfu_update_packet_t   packet;
    
    //packet.packet_type = *((uint32_t*)m_dfu_buf);

    if (INVALID_PACKET != m_dfu_packet_type)//packet.packet_type)
    {
        switch (m_dfu_packet_type)
        {
            case DATA_PACKET:
                packet.params.data_packet.packet_length = (m_dfu_buf_length) / 4;
                packet.params.data_packet.p_data_packet = (uint32_t*)(m_dfu_buf);
                retval = dfu_data_pkt_handle(&packet);
                if(retval != NRF_ERROR_INVALID_LENGTH)
                {
                    APP_ERROR_CHECK(retval);
                }  
                break;

            case START_PACKET:
                packet.params.start_packet = (dfu_start_packet_t*)(m_dfu_buf);
                retval = dfu_start_pkt_handle(&packet);
                APP_ERROR_CHECK(retval);
                break;

            case INIT_PACKET:
                packet.params.data_packet.packet_length = (m_dfu_buf_length) / 4;
                packet.params.data_packet.p_data_packet = (uint32_t*)(m_dfu_buf);
                retval = dfu_init_pkt_handle(&packet);
                retval = dfu_init_pkt_complete();
                APP_ERROR_CHECK(retval);
                break;

            case STOP_DATA_PACKET:
                (void)dfu_image_validate();
                (void)dfu_image_activate();
                // Break the loop by returning.
                return;
            
            default:
                // No implementation needed.
                break;
        }
    }
}

void spi_slave_com_callback(spi_slave_com_event_t *event)
{
    switch(event->type)
    {
        case SPI_SLAVE_COM_EVENT_RX:
            if(event->crc_ok == false)
            {
                // TODO: Add handling of incorrect CRC here, if needed (not strictly necessary since the DFU has it's own CRC checking algorithm)
            }
            
            m_dfu_packet_type = event->id - 0x80;
            memcpy(m_dfu_buf, event->p_data, event->data_length);
            m_dfu_buf_length = event->data_length;
            
            // Schedule event
            app_sched_event_put(NULL, 0, process_dfu_packet);
            break;
    }
}

uint32_t dfu_transport_update_start(void)
{
    dfu_register_callback(dfu_cb_handler);

    spi_slave_com_init_t spi_slave_init;
    spi_slave_init.pin_sck = 1;
    spi_slave_init.pin_miso = 2;
    spi_slave_init.pin_mosi = 3;
    spi_slave_init.pin_cs = 4;
    spi_slave_init.pin_reqn = 5;
    spi_slave_init.pin_rdyn = 6;    
    spi_slave_init.callback = spi_slave_com_callback;
    spi_slave_com_init(&spi_slave_init);

    return NRF_SUCCESS;
}


uint32_t dfu_transport_close(void)
{
    return 0; //hci_transport_close();
}

