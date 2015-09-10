-------------------- SPI slave bootloader example ---------------------------------------

This is a simple modification to the serial bootloader example in the nRF51 SDK v8.1.0 to allow
an SPI master to update the flash in an nRF51 device using the SPI slave interface in the nRF51 
to receive the data. 
The example reuses as much of the existing DFU libraries as possible, only replacing the transport
layer to use SPI slave instead of UART/HCI. 

Included is an example host application allowing one nRF51 kit to flash another for demonstration 
and test purposes. 

The example host application can currently only program an application (not a bootloader or SoftDevice), 
and expects the application binary to be programmed into address 0x18000 of the flash. 
The host application itself runs from address 0x0000, and does not use a SoftDevice. 
To 'prime' the host application simply flash the application binary first, followed by flashing the host application itself.
The host application will blink an LED when running, and pressing button 1 will start a transfer. 

The bootloader application works similarly to the other bootloader examples. 
The bootloader will have to be programmed in after the SoftDevice is installed, and the bootloader can 
be reactivated at any time by holding down button 4 during reset of the device. 