#ifndef __SPI_SLAVE_COM_INTERFACE_H
#define __SPI_SLAVE_COM_INTERFACE_H

#include <stdint.h>
#include <stdbool.h>

#define SPI_COM_MAX_PACKET_LENGTH 512

#define SPI_COM_PCK_HEADER_SIZE   6
#define SPI_COM_PCK_DATA_SIZE     128
#define SPI_COM_MAX_TRANSACTION_SIZE (SPI_COM_PCK_HEADER_SIZE + SPI_COM_PCK_DATA_SIZE)

enum {SPI_SLAVE_COM_EVENT_RX};

typedef struct
{
    uint32_t type;
    uint32_t data_length;
    uint8_t  *p_data;
    uint8_t  id;
    bool     crc_ok;
}spi_slave_com_event_t;

typedef void (*spi_slave_com_callback_t)(spi_slave_com_event_t *event);

typedef union
{
    uint8_t data[SPI_COM_MAX_TRANSACTION_SIZE];
    struct
    {
        uint8_t len_lsb;
        uint8_t len_msb;
        uint8_t id;
        uint8_t seq_nr;
        uint8_t crc_lsb;
        uint8_t crc_msb;
        uint8_t data[SPI_COM_PCK_DATA_SIZE];
    }fields;
}spi_slave_com_pck_t;

typedef struct
{
    uint8_t pin_mosi;
    uint8_t pin_miso;
    uint8_t pin_sck;
    uint8_t pin_cs;
    uint8_t pin_rdyn;
    uint8_t pin_reqn;
    spi_slave_com_callback_t callback;
}spi_slave_com_init_t;

uint32_t spi_slave_com_init(spi_slave_com_init_t *spi_slave_com_init);

uint32_t spi_slave_com_tx(uint8_t id, uint8_t *p_data, uint32_t data_length);

#endif
