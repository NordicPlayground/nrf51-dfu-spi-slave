#ifndef __DFU_SPI_MASTER_LIB
#define __DFU_SPI_MASTER_LIB

#include <stdint.h>
#include "dfu_types.h"
#include "nrf.h"

extern uint16_t softdevice_list_default[];

#define DEFAULT_INIT_PACKET {0xFFFF, 0xFFFF, 0xFFFFFFFF, 0x0001, softdevice_list_default, 0x0000, 0x0000};

typedef struct
{
    uint16_t device_type;
    uint16_t device_revision;
    uint32_t application_version;
    uint16_t valid_softdevice_list_length;
    uint16_t *valid_softdevice_list;
    uint16_t image_crc;
    uint16_t filler; 
}dfu_init_packet_t;

uint32_t dfu_spi_init(void);

uint32_t dfu_spi_send_start_packet(dfu_start_packet_t *packet);

uint32_t dfu_spi_send_init_packet(dfu_init_packet_t *init_packet);

uint32_t dfu_spi_send_data_packet(uint8_t *data_ptr, uint32_t data_length);
 
uint32_t dfu_spi_send_stop_packet(void);

#endif
