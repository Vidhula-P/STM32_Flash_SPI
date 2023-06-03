# STM32_Flash_SPI
Configure external flash, SST26VF064B to store data using STM32F107VCT6 microcontroller

Code generated using CubeMX software with the following settings-
SPI3 configuration:
  Mode - Full-duplex master
  Data size - 8b
  First bit - MSB
  Prescalar - 256
  (baud rate = 140.625Kbps)
GPIO configurations:
  PB5 - SPI3_MOSI
  PB4 - SPI3_MISO
  PB3 - SPI3_SCK
  PD7, PD6, PD5 - GPIO_output (output initially HIGH)
Timer 2 configuration:
  Prescalar - 71
  Counter Period - 4999
  (time period = 5ms)
