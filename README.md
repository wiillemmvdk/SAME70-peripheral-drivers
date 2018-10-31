# SAME70-peripheral-drivers
 Drivers for multiple peripheral of the SAME70Q21 made for the XPLD board

 The drivers have been written without the use of ASF (Atmel software framework) although some recources have been used like the syscalls and the sd_mmc_protocol macro's

The files can be loaded into a new Atmel Studio project

## Supported peripherals
* PMC clock setup from external crystal
* USART in asynch mode; usage of printf works because of syscalls
* I2SC over DMA with the XDMA controller
* sdcard in SD mode with the HSMCI

