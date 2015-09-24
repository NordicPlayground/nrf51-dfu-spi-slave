/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
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

/** @file
 *
 * @defgroup blinky_example_main main.c
 * @{
 * @ingroup blinky_example
 * @brief Blinky Example Application main file.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "spi_master.h"
#include "dfu_types.h"
#include "dfu_spi_master_lib.h"
#include "crc16.h"

#define HOST_SPI_MASTER_APP_START   0x0

#define HOST_SD_REGION_START        0x11000
#define HOST_SD_REGION_END          0x30000

#define HOST_APP_REGION_START       0x30000
#define HOST_APP_REGION_END         0x38000

#define HOST_BL_REGION_START        0x38000
#define HOST_BL_REGION_END          0x3C000

static uint8_t random_number[4]; // used to inject in BL image in order to change CRC from previous upload - used for testing

// Simple algorithm for finding the end of the firmware image in the flash
// Assumes that there is nothing stored in the flash after the image, otherwise it will fail
uint32_t find_image_size(uint32_t start_address, uint32_t end_address)
{
    uint32_t img_size = end_address - 4;
    while(img_size > start_address)
    {
        if(*((uint32_t*)img_size) != 0xFFFFFFFF) break;
        img_size -= 4;
    }
    img_size = img_size + 4 - start_address;   
    
    return img_size;    
}

// Calculates the 16-bit CRC over the specified region
uint16_t find_image_crc(uint32_t start_address, uint32_t image_size)
{
    return crc16_compute((const uint8_t *)start_address, image_size, 0);
}

void get_rand_num(uint8_t * rand_buffer)
{
    uint8_t i = 0;
    
    NRF_RNG->CONFIG = 1;
    NRF_RNG->TASKS_START = 1;
    
    do
    {
        while(NRF_RNG->EVENTS_VALRDY == 0);
        NRF_RNG->EVENTS_VALRDY = 0;
        rand_buffer[i++] = NRF_RNG->VALUE;
        
    }while(i<4);
    
    NRF_RNG->TASKS_STOP;
       
}

// This function will update an application, stored in local flash at the address APPLICATION_HEX_LOCATION
void test_update(uint8_t mode)
{
    // Send START packet
    dfu_start_packet_t start_packet = {.dfu_update_mode = mode,
                                       .sd_image_size = 0,
                                       .bl_image_size = 0,
                                       .app_image_size = 0};  
    
    uint32_t new_image_start, new_image_size;
    uint16_t new_image_crc;
    
    get_rand_num(random_number);
                                       
    nrf_gpio_pin_clear(21);
    
    if(mode == DFU_UPDATE_APP)
    {
        new_image_start = HOST_APP_REGION_START;
        
        // Find the size of the image programmed into the flash
        new_image_size = find_image_size(HOST_APP_REGION_START, HOST_APP_REGION_END);
        
        // Find the CRC of the image programmed into the flash
        new_image_crc  = find_image_crc(HOST_APP_REGION_START, new_image_size);
        
        start_packet.app_image_size = new_image_size;
    }
    
    else if(mode == DFU_UPDATE_BL)
    {
        new_image_start = HOST_BL_REGION_START;
        
        // Find the size of the image programmed into the flash
        new_image_size = find_image_size(HOST_BL_REGION_START, HOST_BL_REGION_END);
        
        // Find the CRC of the image programmed into the flash

        new_image_crc  = find_image_crc(HOST_BL_REGION_START, new_image_size);
        
        new_image_crc  = crc16_compute(random_number,4,&new_image_crc);
   
        
        start_packet.bl_image_size = new_image_size + 4;
    }
    else if(mode == DFU_UPDATE_SD)
    {
        new_image_start = HOST_SD_REGION_START;
        
       // Find the size of the image programmed into the flash
        new_image_size = find_image_size(HOST_SD_REGION_START, HOST_SD_REGION_END);
    
        // Find the CRC of the image programmed into the flash
        new_image_crc  = find_image_crc(HOST_SD_REGION_START, new_image_size);

        start_packet.sd_image_size = new_image_size;
    }
                                
    dfu_spi_send_start_packet(&start_packet);
                                       
    nrf_delay_ms(2500);
                 
    // Send INIT packet
    dfu_init_packet_t init_packet = DEFAULT_INIT_PACKET;
    init_packet.image_crc = new_image_crc;
    dfu_spi_send_init_packet(&init_packet);
    
    nrf_delay_ms(1500);
    
    // Send DATA
    uint8_t *app_image_addr = (uint8_t*)new_image_start;
    for(int i = 0; i < new_image_size; i += 128)
    {
        uint32_t bytes_to_write = new_image_size - i;
        if(bytes_to_write > 128) bytes_to_write = 128;
        dfu_spi_send_data_packet(app_image_addr + i, bytes_to_write);
        nrf_delay_ms(15);
    }
    
    if(mode == DFU_UPDATE_BL)
    {
       dfu_spi_send_data_packet(random_number,sizeof(uint32_t));
       nrf_delay_ms(15);        
    }
    
    // Send STOP
    dfu_spi_send_stop_packet();

    nrf_gpio_pin_set(21);    
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    nrf_gpio_range_cfg_input(17, 20, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_range_cfg_output(21, 24);
    NRF_GPIO->OUTSET = 0xF << 21;
    
    nrf_gpio_cfg_output(7);
    
    nrf_gpio_pin_clear(7);
    
    dfu_spi_init();
    
    while(1)
    {
      nrf_gpio_pin_clear(7);
      nrf_delay_ms(20);
      nrf_gpio_pin_set(7);
      test_update(DFU_UPDATE_BL);
      nrf_delay_ms(1000);
      nrf_gpio_pin_toggle(24);
      
      
    }
    
    //test_update(DFU_UPDATE_BL);
    
    /*
    while(1)
    {
        if(nrf_gpio_pin_read(BUTTON_1) == 0)
        {
            test_update(DFU_UPDATE_APP);
        }
        else if(nrf_gpio_pin_read(BUTTON_2) == 0)
        {
            test_update(DFU_UPDATE_SD);
        }
        else if(nrf_gpio_pin_read(BUTTON_3) == 0)
        {
            test_update(DFU_UPDATE_BL);
        }
        */
        //nrf_gpio_pin_toggle(24);
        //nrf_delay_ms(50);
    //}
}


/** @} */
