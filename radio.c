/*
 * radio.c
 *
 *  Created on: Dec 31, 2021
 *      Author: kosa
 */

#include "radio.h"
#include "spi.h"
#include "hwr.h"
#include "uart.h"
#include "timer.h"
#include <math.h>

#include "ax_reg.h"
#include "ax_reg_values.h"
#include "ax_fifo.h"
#include "config_values.h"

#include <stdint.h>

uint8_t radio_read_u8(uint16_t addr)
{
	if(addr > 0x7F)
	{
		uint8_t buff[3];
		buff[0] = ((addr>>8)&0x0f) | 0x70;
		buff[1] = addr&0xFF;
		buff[2] = 0xff;
		spi_transfer(buff, 3);
#ifdef DUMP_REGSS_READ
		uart_put_raw(0x01);
		uart_put_raw((addr>>8)&0x0f);
		uart_put_raw(addr);
		uart_put_raw(buff[2]);
#endif
		return buff[2];
	}
	else
	{
		uint8_t buff[2];
		buff[0] = addr;
		buff[1] = 0xff;
		spi_transfer(buff, 2);
#ifdef DUMP_REGS_READ
		uart_put_raw(0x01);
		uart_put_raw((addr>>8)&0x0f);
		uart_put_raw(addr);
		uart_put_raw(buff[1]);
#endif
		return buff[1];
	}
}

uint16_t radio_read_u16(uint16_t addr)
{
	if(addr > 0x7F)
	{
		uint8_t buff[4];
		buff[0] = ((addr>>8)&0x0f) | 0x70;
		buff[1] = addr&0xFF;
		buff[2] = 0xff;
		buff[3] = 0xff;
		spi_transfer(buff, 4);
#ifdef DUMP_REGSS_READ
		uart_put_raw(0x02);
		uart_put_raw((addr>>8)&0x0f);
		uart_put_raw(addr);
		uart_put_raw(buff[2]);
		uart_put_raw(buff[3]);
#endif
		return (buff[2]<<8) | buff[3];
	}
	else
	{
		uint8_t buff[3];
		buff[0] = addr;
		buff[1] = 0xff;
		buff[2] = 0xff;
		spi_transfer(buff, 3);
#ifdef DUMP_REGSS_READ
		uart_put_raw(0x02);
		uart_put_raw((addr>>8)&0x0f);
		uart_put_raw(addr);
		uart_put_raw(buff[1]);
		uart_put_raw(buff[2]);
#endif
		return (buff[1]<<8) | buff[2];
	}
}

void radio_write_u8(uint16_t addr, uint8_t data)
{
#ifdef DUMP_REGS
	uart_put_raw(0x03);
	uart_put_raw((addr>>8)&0x0f);
	uart_put_raw(addr);
	uart_put_raw(data);
#endif
	if(addr > 0x7F)
	{
		uint8_t buff[3];
		buff[0] = ((addr>>8)&0x0f) | 0xF0;
		buff[1] = addr&0xFF;
		buff[2] = data;
		spi_transfer(buff, 3);
	}
	else
	{
		uint8_t buff[2];
		buff[0] = addr | 0x80;
		buff[1] = data;
		spi_transfer(buff, 2);
	}
#ifdef DUMP_REGS
	if(data != radio_read_u8(addr))
		uart_put_raw(0x07);
#endif
}

void radio_write_u16(uint16_t addr, uint16_t data)
{
#ifdef DUMP_REGS
	uart_put_raw(0x04);
	uart_put_raw((addr>>8)&0x0f);
	uart_put_raw(addr);
	uart_put_raw(data>>8);
	uart_put_raw(data);
#endif
	if(addr > 0x7F)
	{
		uint8_t buff[4];
		buff[0] = ((addr>>8)&0x0f) | 0xF0;
		buff[1] = addr&0xFF;
		buff[2] = data>>8;
		buff[3] = data;
		spi_transfer(buff, 4);
	}
	else
	{
		uint8_t buff[3];
		buff[0] = addr | 0x80;
		buff[1] = data>>8;
		buff[2] = data;
		spi_transfer(buff, 3);
	}
#ifdef DUMP_REGS
	if(data != radio_read_u16(addr))
		uart_put_raw(0x07);
#endif
}

void radio_write_u24(uint16_t addr, uint32_t data)
{
#ifdef DUMP_REGS
	uart_put_raw(0x05);
	uart_put_raw((addr>>8)&0x0f);
	uart_put_raw(addr);
	uart_put_raw(0);
	uart_put_raw(data>>16);
	uart_put_raw(data>>8);
	uart_put_raw(data);
#endif
	if(addr > 0x7F)
	{
		uint8_t buff[5];
		buff[0] = ((addr>>8)&0x0f) | 0xF0;
		buff[1] = addr&0xFF;
		buff[2] = data>>16;
		buff[3] = data>>8;
		buff[4] = data;
		spi_transfer(buff, 5);
	}
	else
	{
		uint8_t buff[4];
		buff[0] = addr | 0x80;
		buff[1] = data>>16;
		buff[2] = data>>8;
		buff[3] = data;
		spi_transfer(buff, 4);
	}
#ifdef DUMP_REGS
	uint32_t read = ((uint32_t)radio_read_u16(addr) << 8) | radio_read_u8(addr+2);
	if(data != read)
		uart_put_raw(0x07);
#endif
}

void radio_write_u32(uint16_t addr, uint32_t data)
{
#ifdef DUMP_REGS
	uart_put_raw(0x06);
	uart_put_raw((addr>>8)&0x0f);
	uart_put_raw(addr);
	uart_put_raw(data>>24);
	uart_put_raw(data>>16);
	uart_put_raw(data>>8);
	uart_put_raw(data);
#endif
	if(addr > 0x7F)
	{
		uint8_t buff[6];
		buff[0] = ((addr>>8)&0x0f) | 0xF0;
		buff[1] = addr&0xFF;
		buff[2] = data>>24;
		buff[3] = data>>16;
		buff[4] = data>>8;
		buff[5] = data;
		spi_transfer(buff, 6);
	}
	else
	{
		uint8_t buff[5];
		buff[0] = addr | 0x80;
		buff[1] = data>>24;
		buff[2] = data>>16;
		buff[3] = data>>8;
		buff[4] = data;
		spi_transfer(buff, 5);
	}
#ifdef DUMP_REGS
	uint32_t read = ((uint32_t)radio_read_u16(addr) << 8) | radio_read_u8(addr+2);
	if(data != read)
		uart_put_raw(0x07);
#endif
}

void radio_set_freq(uint32_t freq_hz)
{
	uint8_t radiostate;
	/* wait for current operations to finish */

	do {
		radiostate = radio_read_u8(AX_REG_RADIOSTATE) & 0xF;
	} while (radiostate == AX_RADIOSTATE_TX);

	radio_write_u8(AX_REG_PWRMODE, AX_PWRMODE_STANDBY | 0x60);

	uint32_t freq;
	/* we choose to always set the LSB to avoid spectral tones */
	freq = (uint32_t)(((uint64_t)freq_hz * (1UL << 23)) /
			(uint32_t)F_CLK);
	freq = (freq << 1) | 1;
	radio_write_u32(AX_REG_FREQA,freq);
	radio_write_u32(AX_REG_FREQB,freq);

	/* Manual VCO current, 27 = 1350uA VCO1, 270uA VCO2 */
	radio_write_u8(AX_REG_PLLVCOI,AX_PLLVCOI_ENABLE_MANUAL | 27);

	/* Wait for oscillator to be stable */
	while (!(radio_read_u8(AX_REG_XTALSTATUS) & 1)) {
	};

	radio_write_u8(AX_REG_PLLLOOP,
			AX_PLLLOOP_FILTER_DIRECT |
			AX_PLLLOOP_INTERNAL_FILTER_BW_100_KHZ);
	radio_write_u8(AX_REG_PLLCPI,8);
	radio_write_u8(AX_REG_PLLVCODIV,
			AX_PLLVCODIV_DIVIDE_2 |
			AX_PLLVCODIV_RF_DIVIDER_DIV_TWO |
			AX_PLLVCODIV_RF_INTERNAL_VCO_EXTERNAL_INDUCTOR);
	radio_write_u8(0xF34, 0x08);

	radio_write_u8(AX_REG_PLLRANGINGA, AX_PLLRANGING_RNG_START | 8);

	uint8_t r;
	/* Wait for RNGSTART bit to clear */
	do {
		r = radio_read_u8(AX_REG_PLLRANGINGA);
	} while (r & AX_PLLRANGING_RNG_START);

	if (r & AX_PLLRANGING_RNGERR) {
		while(1); //TODO: error
	}

	radio_write_u8(AX_REG_PLLRANGINGB, AX_PLLRANGING_RNG_START | 8);

	/* Wait for RNGSTART bit to clear */
	do {
		r = radio_read_u8(AX_REG_PLLRANGINGB);
	} while (r & AX_PLLRANGING_RNG_START);

	if (r & AX_PLLRANGING_RNGERR) {
		while(1); //TODO: error
	}

	radio_write_u8(AX_REG_PWRMODE, AX_PWRMODE_POWERDOWN | 0x60);

	/*radio_write_u8(AX_REG_PLLLOOP,
			AX_PLLLOOP_FILTER_DIRECT |
			AX_PLLLOOP_INTERNAL_FILTER_BW_500_KHZ);
	radio_write_u8(AX_REG_PLLCPI,200);
	radio_write_u8(AX_REG_PLLVCODIV,
			AX_PLLVCODIV_RF_DIVIDER_DIV_TWO |
			AX_PLLVCODIV_RF_INTERNAL_VCO_EXTERNAL_INDUCTOR);
	radio_write_u8(0xF34, 0x08);*/

}


void radio_setup_modulation(register_pair *regs)
{
	while(regs->reg != 0x000)
	{
		radio_write_u8(regs->reg, regs->val);
		regs++;
	}
}

void radio_tx_on(void)
{
	radio_write_u8(AX_REG_PWRMODE, AX_PWRMODE_STANDBY | 0x60);
	radio_write_u8(AX_REG_PWRMODE, AX_PWRMODE_FIFOON | 0x60);

	radio_setup_modulation(config_tx);

	radio_write_u8(AX_REG_FIFOSTAT, AX_FIFOCMD_CLEAR_FIFO_DATA_AND_FLAGS);

	radio_write_u8(AX_REG_PWRMODE, AX_PWRMODE_FULLTX | 0x60);

	/* Wait for oscillator to be stable */
	while (!(radio_read_u8(AX_REG_XTALSTATUS) & 1)) {
	};
}

void radio_rx_on(void)
{
	radio_write_u8(AX_REG_PWRMODE, AX_PWRMODE_FULLRX | 0x60);

	radio_setup_modulation(config_rx);

	radio_write_u8(AX_REG_FIFOSTAT, AX_FIFOCMD_CLEAR_FIFO_DATA_AND_FLAGS);
}

uint8_t stat_old=0;


void radio_rx_data(void)
{
#ifdef DUMP_REGS
	uint8_t stat = radio_read_u8(AX_REG_RADIOSTATE) & 0xF;
	if(stat_old != stat)
	{
		uart_put_raw(0x8);
		uart_put_raw(stat);
		stat_old = stat;
	}
#endif
	while(1)
	{
		if(!radio_read_u16(AX_REG_FIFOCOUNT))
			return;
		uint8_t type = radio_read_u8(AX_REG_FIFODATA);
		if (type != 0xE1)
			while(1); // should not happen
		uint8_t len = radio_read_u8(AX_REG_FIFODATA)-1;
		uint8_t status = radio_read_u8(AX_REG_FIFODATA);
		if (status & 0x01)
			uart_put(0x00);
		spi_transfer_start();
		spi_transfer_cont(AX_REG_FIFODATA);
		for(uint8_t i=0;i<len;i++)
		{
			uart_put(spi_transfer_cont(0xFF));
		}
		spi_transfer_end();
		if(status & 0x02)
			uart_put_fend();
	}
}

void radio_tx_data_packet(uint8_t *data, uint16_t length)
{
	/* Ensure the SVMODEM bit (POWSTAT) is set high (See 3.1.1) */
	while (!(radio_read_u8(AX_REG_POWSTAT) & AX_POWSTAT_SVMODEM));
	uint8_t fifocount;
	uint8_t chunk_length;
	uint16_t rem_length;
	uint8_t pkt_end = 0;
	/* send remainder first */
	chunk_length = length % 200;
	rem_length = length - chunk_length;

	if (length <= 200) {           /* all in one go */
		pkt_end = AX_FIFO_TXDATA_PKTEND;
	}

	/* wait for enough space to contain both the preamble and chunk */
	do {
		fifocount = radio_read_u8(AX_REG_FIFOCOUNT);
	} while (fifocount > (256 - (chunk_length+20)));

	/* preamble */
	spi_transfer_start();
	spi_transfer_cont(AX_REG_FIFODATA|0x80);
	spi_transfer_cont(AX_FIFO_CHUNK_REPEATDATA);
	spi_transfer_cont(AX_FIFO_TXDATA_UNENC | AX_FIFO_TXDATA_RAW | AX_FIFO_TXDATA_NOCRC);
	spi_transfer_cont(9);
	spi_transfer_cont(0x7E);
	spi_transfer_end();

	spi_transfer_start();
	spi_transfer_cont(AX_REG_FIFODATA|0x80);
	spi_transfer_cont(AX_FIFO_CHUNK_DATA);
	spi_transfer_cont(chunk_length+1);         /* incl flags */
	spi_transfer_cont(AX_FIFO_TXDATA_PKTSTART | pkt_end);
	for(uint16_t i=0;i<chunk_length;i++)
		spi_transfer_cont(data[i]);
	spi_transfer_end();
	data += chunk_length;

	radio_write_u8(AX_REG_FIFOSTAT, AX_FIFOCMD_COMMIT);

	/* write subsequent data */
	while (rem_length) {
		if (rem_length > 200) {     /* send 200 bytes */
			chunk_length = 200; rem_length -= 200;
		} else {                    /* finish off */
			chunk_length = rem_length; rem_length = 0;
			pkt_end = AX_FIFO_TXDATA_PKTEND;
		}

		/* wait for enough space for chunk */
		do {
			fifocount = radio_read_u8(AX_REG_FIFOCOUNT);
		} while (fifocount > (256 - (chunk_length+10)));

		/* write chunk */
		spi_transfer_start();
		spi_transfer_cont(AX_REG_FIFODATA|0x80);
		spi_transfer_cont(AX_FIFO_CHUNK_DATA);
		spi_transfer_cont(chunk_length+1);         /* incl flags */
		spi_transfer_cont(pkt_end);
		for(uint16_t i=0;i<chunk_length;i++)
			spi_transfer_cont(data[i]);
		spi_transfer_end();
		data += chunk_length;
		radio_write_u8(AX_REG_FIFOSTAT, AX_FIFOCMD_COMMIT);      /* commit */
	}

	uint8_t radiostate;
	do {
		radiostate = radio_read_u8(AX_REG_RADIOSTATE) & 0xF;
	} while (radiostate != AX_RADIOSTATE_IDLE);
}


void radio_start(void)
{
	spi_start();
	while(1)
	{
		if(radio_read_u8(AX_REG_SCRATCH) == AX_SCRATCH)
			break;
	}
	radio_write_u8(AX_REG_PWRMODE, AX_PWRMODE_RST);

	Timer rst_dly;
	timer_init(&rst_dly,2);
	while(!timer_async(&rst_dly));

	radio_write_u8(AX_REG_PWRMODE, AX_PWRMODE_POWERDOWN | 0x60);

	radio_write_u8(AX_REG_XTALCAP, 0x00);
	radio_write_u8(AX_REG_XTALOSC, 0x04);
	radio_write_u8(AX_REG_XTALAMPL, 0x00);
	if(XTALDIV == 2)
		radio_write_u8(0xF35, 0x11);
	else
		radio_write_u8(0xF35, 0x10);
	//radio_write_u8(AX_REG_PINFUNCSYSCLK,0x04);

	radio_write_u8(AX_REG_PINFUNCPWRAMP,0x05);
	radio_write_u8(AX_REG_DACCONFIG,(1<<6)| 0x08);
	radio_write_u16(AX_REG_DACVALUE, 0x0C);

	radio_set_freq(144800000);
	radio_setup_modulation(config_common);
	radio_rx_on();
}
