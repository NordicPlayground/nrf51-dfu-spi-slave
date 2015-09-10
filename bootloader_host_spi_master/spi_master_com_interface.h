#ifndef __SPI_MASTER_COM_INTERFACE_H
#define __SPI_MASTER_COM_INTERFACE_H

#define SPI_COM_MAX_PACKET_LENGTH 512

#define SPI_COM_PCK_HEADER_SIZE   6
#define SPI_COM_PCK_DATA_SIZE     128
#define SPI_COM_MAX_TRANSACTION_SIZE (SPI_COM_PCK_HEADER_SIZE + SPI_COM_PCK_DATA_SIZE)

#include <stdint.h>
#include <stdbool.h>

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
}spi_master_com_pck_t;

typedef struct
{
    uint8_t pin_mosi;
    uint8_t pin_miso;
    uint8_t pin_sck;
    uint8_t pin_cs;
    uint8_t pin_rdyn;
    uint8_t pin_reqn;
}spi_master_com_init_t;

uint32_t spi_master_com_init(spi_master_com_init_t *spi_master_com_init);

uint32_t spi_master_com_tx(uint8_t id, uint8_t *p_data, uint32_t data_length);

bool spi_master_com_busy(void);

#endif
