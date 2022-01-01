/*
 * spi.c
 *
 *  Created on: Dec 31, 2021
 *      Author: kosa
 */

#include "hwr.h"
#include "spi.h"
#include "gpio.h"

void spi_start(void)
{

	gpio_setup(&SPI_MOSI_PORT, SPI_MOSI_PIN, true, true, true); // output,push-pull,fast
	gpio_setup(&SPI_SCK_PORT, SPI_SCK_PIN, true, true, true); // output,push-pull,fast
	gpio_setup(&SPI_SS_PORT, SPI_SS_PIN, true, true, true); // output,push-pull,fast
	gpio_setup(&SPI_MISO_PORT, SPI_MISO_PIN, false, true, false); //input, pullup, no-int
	gpio_set(&SPI_SS_PORT, SPI_SS_PIN,1);

	sfr_REMAP.SYSCFG_RMPCR1.SPI1_REMAP = 0;

	sfr_CLK.PCKENR1.PCKEN14 = 1;

	// SPI registers: First reset everything
	sfr_SPI1.CR1.byte = 0x00;
	sfr_SPI1.CR2.byte = 0x00;

	// SPI_CR1 LSBFIRST=0 (MSB is transmitted first)
	sfr_SPI1.CR1.LSBFIRST = 0;

	// Baud Rate Control: 0b001 = fmaster / 4 (4MHz)
	sfr_SPI1.CR1.BR = 1;

	// Clock Phase, The first clock transition is the first data capture edge
	sfr_SPI1.CR1.CPOL = 0;

	// Clock Polarity, SCK=0 when idle
	sfr_SPI1.CR1.CPHA = 0;

	sfr_SPI1.CR2.SSM  = 1;       // Software slave management, enabled
	sfr_SPI1.CR2.SSI  = 1;       // Internal slave select, Master mode
	sfr_SPI1.CR1.MSTR = 1;       // Master configuration.

}

void spi_transfer(uint8_t *buff, uint8_t len)
{
	gpio_set(&SPI_SS_PORT, SPI_SS_PIN,0);
	sfr_SPI1.CR1.MSTR = 1;       // Master device.
	sfr_SPI1.CR1.SPE  = 1;       // SPI Enable, Peripheral enabled
	for(uint8_t i=0;i<len;i++)
	{
		while (!sfr_SPI1.SR.TXE);     // wait until SPI not busy
		sfr_SPI1.DR.byte  = buff[i];    // send 1B
		while (!sfr_SPI1.SR.RXNE);     // wait until SPI not busy
		buff[i] = sfr_SPI1.DR.byte;     // read data
	}
	while (sfr_SPI1.SR.BSY);     // wait until SPI not busy
	sfr_SPI1.CR1.SPE  = 0;       // Disable SPI
	gpio_set(&SPI_SS_PORT, SPI_SS_PIN,1);

}

void spi_transfer_start(void)
{
	gpio_set(&SPI_SS_PORT, SPI_SS_PIN,0);
	sfr_SPI1.CR1.MSTR = 1;       // Master device.
	sfr_SPI1.CR1.SPE  = 1;       // SPI Enable, Peripheral enabled
}
uint8_t spi_transfer_cont(uint8_t data)
{
	while (!sfr_SPI1.SR.TXE);     // wait until SPI not busy
	sfr_SPI1.DR.byte  = data;    // send 1B
	while (!sfr_SPI1.SR.RXNE);     // wait until SPI not busy
	uint8_t ret = sfr_SPI1.DR.byte;     // read data
	return ret;
}
void spi_transfer_end(void)
{
	while (sfr_SPI1.SR.BSY);     // wait until SPI not busy
	sfr_SPI1.CR1.SPE  = 0;       // Disable SPI
	gpio_set(&SPI_SS_PORT, SPI_SS_PIN,1);
}



