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

	radio_write_u8(AX_REG_PLLLOOP,
			AX_PLLLOOP_FILTER_DIRECT |
			AX_PLLLOOP_INTERNAL_FILTER_BW_500_KHZ);
	radio_write_u8(AX_REG_PLLCPI,200);
	radio_write_u8(AX_REG_PLLVCODIV,
			AX_PLLVCODIV_DIVIDE_2 |
			AX_PLLVCODIV_RF_DIVIDER_DIV_TWO |
			AX_PLLVCODIV_RF_INTERNAL_VCO_EXTERNAL_INDUCTOR);
	radio_write_u8(0xF34, 0x08);

}

static uint8_t ax_value_to_mantissa_exp_4_4(uint32_t value)
{
	uint8_t exp = 0;

	while (value > 15 && exp < 15) {
		value >>= 1; exp++;
	}

	return ((value & 0xF) << 4) | exp; /* mantissa, exponent */
}

static uint8_t ax_value_to_exp_mantissa_3_5(uint32_t value)
{
	uint8_t exp = 0;

	while (value > 31 && exp < 7) {
		value >>= 1; exp++;
	}

	return ((exp & 0x7) << 5) | value; /* exponent, mantissa */
}

inline uint8_t log2i(uint32_t x)
{
	uint8_t log2Val = 0 ;
	// Count push off bits to right until 0
	// 101 => 10 => 1 => 0
	// which means hibit was 3rd bit, its value is 2^3
	while( x>>=1 ) log2Val++;  // div by 2 until find log2.  log_2(63)=5.97, so
	// take that as 5, (this is a traditional integer function!)
	// eg x=63 (111111), log2Val=5 (last one isn't counted by the while loop)
	return log2Val ;
}

#define M_LOG2E 1.44269504088896340736

uint32_t decimation;
uint8_t afskshift;

void radio_setup_modulation(void)
{
	uint32_t deviation = 3000;
	uint32_t bitrate = 1200;

	float m = 2.0 * deviation/ bitrate;
	//uint32_t rx_bandwith = deviation;
	uint32_t f_baseband = 15000;//2*(deviation+2200);
	uint32_t if_frequency = f_baseband / 2;
	if(if_frequency < 3180)
		if_frequency = 3180;
	uint16_t iffreq =
			((uint64_t)if_frequency
					* (1UL << 20) * XTALDIV
					+ F_CLK/2) /* roundup*/
					/ F_CLK;
	uint32_t decdiv = (16 * XTALDIV * 4 /* flt */ * f_baseband);
	decimation = (F_CLK + decdiv/2)/decdiv;
	if(decimation > 127)
		decimation = 127;
	uint32_t rx_datadiv = XTALDIV * bitrate * decimation;
	uint32_t rx_data_rate = ((uint64_t)F_CLK * 128 + rx_datadiv/2) / rx_datadiv;
	uint32_t max_rf_offset = ((uint64_t)f_baseband * (1UL << 24) + F_CLK/2)/F_CLK;
	//uint32_t fskd = (uint32_t)(260 * m) & (~(uint32_t)1);
	uint32_t fskd = ((uint64_t)3 * 512 * deviation)/bitrate;

	uint32_t bw = F_CLK / ((uint32_t)32 * bitrate * XTALDIV * decimation);

	afskshift = (uint8_t)(2*log2i(bw));

	float ratio = (64.0 * PI * XTALDIV * bitrate/10) /
			(float)F_CLK;

	uint8_t agc_bw_product = (uint8_t)(-logf(1 - sqrtf(1 - ratio))* M_LOG2E);

	uint8_t agc_attack_initial = agc_bw_product;
	uint8_t agc_decay_initial = agc_bw_product + 7;
	if(agc_attack_initial > 0x08) agc_attack_initial = 0x08;
	if(agc_decay_initial > 0x0E) agc_decay_initial = 0x0E;
	uint8_t agc_attack_after = agc_attack_initial;
	uint8_t agc_decay_after = agc_decay_initial;
	uint8_t agc_attack_during = 0x0F;
	uint8_t agc_decay_during = 0x0F;

	uint32_t  time_gain_initial = rx_data_rate / 4;
	if(time_gain_initial >= rx_data_rate - (1UL<<12))
		time_gain_initial = rx_data_rate - (1UL<<12);
	uint32_t  time_gain_after = rx_data_rate / 16;
	if(time_gain_after >= rx_data_rate - (1UL<<12))
		time_gain_after = rx_data_rate - (1UL<<12);
	uint32_t  time_gain_during = rx_data_rate / 32;
	if(time_gain_during >= rx_data_rate - (1UL<<12))
		time_gain_during = rx_data_rate - (1UL<<12);

	uint32_t dr_gain_initial = rx_data_rate / 128;
	uint32_t dr_gain_after = rx_data_rate / 256;
	uint32_t dr_gain_during = rx_data_rate / 512;

	uint8_t filter_idx = 0x3;
	uint8_t phase_gain = 0x3;

	uint8_t baseband_rg_phase_det = 0xF; /* disable loop */
	uint8_t baseband_rg_freq_det = 0x1F; /* disable loop */

	uint8_t rffreq_rg = logf((float)F_CLK / (XTALDIV * 4 * bitrate))
																		*M_LOG2E + 0.5;
	uint8_t rffreq_rg_during = rffreq_rg + 4;
	if (rffreq_rg > 0xD) { rffreq_rg = 0xD; }
	if (rffreq_rg_during > 0xD) { rffreq_rg_during = 0xD; }

	uint8_t amplflags = AX_AMPLGAIN_AMPLITUDE_RECOVERY_PEAKDET;
	uint8_t amplgain = 6;

	uint16_t freq_dev_initial = 0;
	uint16_t freq_dev = (uint16_t)((deviation * 128 * 0.8) + 0.5); /* k_sf = 0.8 */

	uint8_t match1_threashold = 10;  /* maximum 15 */
	uint8_t match0_threashold = 28;  /* maximum 31 */

	uint8_t pkt_misc_flags = 0;

	/* tx pll boost time */
	uint16_t tx_pll_boost_time = 38;

	/* tx pll settle time */
	uint16_t tx_pll_settle_time = 20;

	/* rx pll boost time */
	uint16_t rx_pll_boost_time = 38;

	/* rx pll settle time */
	uint16_t rx_pll_settle_time = 20;

	/* rx agc coarse  */
	uint16_t rx_coarse_agc = 152;          /* 152 µs */

	uint16_t rx_agc_settling = 0;

	/* 3us rssi setting time */
	pkt_misc_flags |= AX_PKT_FLAGS_RSSI_UNITS_MICROSECONDS;
	uint16_t rx_rssi_settling = 3;

	uint16_t preamble_2_timeout = 23;      /* 23 bits, for 16-bit preamble */

	//all values calculated now setupping registers

	radio_write_u8(AX_REG_MODULATION,AX_MODULATION_AFSK);
	radio_write_u8(AX_REG_ENCODING,AX_ENC_NRZI);
	radio_write_u8(AX_REG_FRAMING,AX_FRAMING_MODE_HDLC | AX_FRAMING_CRCMODE_CCITT);
	radio_write_u8(0xF72, 0x00);
	radio_write_u8(AX_REG_WAKEUPXOEARLY,1);

	//RX
	radio_write_u16(AX_REG_IFFREQ, iffreq);
	radio_write_u8(AX_REG_DECIMATION,decimation);
	radio_write_u24(AX_REG_RXDATARATE,rx_data_rate);
	radio_write_u24(AX_REG_MAXDROFFSET,0);
	radio_write_u24(AX_REG_MAXRFOFFSET, AX_MAXRFOFFSET_FREQOFFSCORR_FIRST_LO | max_rf_offset);
	radio_write_u16(AX_REG_FSKDMAX, fskd & 0xFFFF);
	radio_write_u16(AX_REG_FSKDMIN, ~fskd & 0xFFFF);
	radio_write_u8(AX_REG_AMPLFILTER, 0);
	radio_write_u8(AX_REG_DIVERSITY,0);

	//patterns
	radio_write_u8(AX_REG_RXPARAMSETS,0xF4);

	radio_write_u8(AX_REG_RX_PARAMETER0 + AX_RX_AGCGAIN,
			((agc_decay_initial & 0xF) << 4) | (agc_attack_initial & 0xF));
	radio_write_u8(AX_REG_RX_PARAMETER1 + AX_RX_AGCGAIN,
			((agc_decay_after & 0xF) << 4) | (agc_attack_after & 0xF));
	radio_write_u8(AX_REG_RX_PARAMETER3 + AX_RX_AGCGAIN,
			((agc_decay_during & 0xF) << 4) | (agc_attack_during & 0xF));

	radio_write_u8(AX_REG_RX_PARAMETER0 + AX_RX_AGCTARGET, 0x84);
	radio_write_u8(AX_REG_RX_PARAMETER1 + AX_RX_AGCTARGET, 0x84);
	radio_write_u8(AX_REG_RX_PARAMETER3 + AX_RX_AGCTARGET, 0x84);

	radio_write_u8(AX_REG_RX_PARAMETER0 + AX_RX_AGCAHYST, 0x0);
	radio_write_u8(AX_REG_RX_PARAMETER1 + AX_RX_AGCAHYST, 0x0);
	radio_write_u8(AX_REG_RX_PARAMETER3 + AX_RX_AGCAHYST, 0x0);

	radio_write_u8(AX_REG_RX_PARAMETER0 + AX_RX_AGCMINMAX, 0x0);
	radio_write_u8(AX_REG_RX_PARAMETER1 + AX_RX_AGCMINMAX, 0x0);
	radio_write_u8(AX_REG_RX_PARAMETER3 + AX_RX_AGCMINMAX, 0x0);

	radio_write_u8(AX_REG_RX_PARAMETER0 + AX_RX_TIMEGAIN,
			ax_value_to_mantissa_exp_4_4(time_gain_initial));
	radio_write_u8(AX_REG_RX_PARAMETER1 + AX_RX_TIMEGAIN,
			ax_value_to_mantissa_exp_4_4(time_gain_after));
	radio_write_u8(AX_REG_RX_PARAMETER3 + AX_RX_TIMEGAIN,
			ax_value_to_mantissa_exp_4_4(time_gain_during));

	radio_write_u8(AX_REG_RX_PARAMETER0 + AX_RX_DRGAIN,
			ax_value_to_mantissa_exp_4_4(dr_gain_initial));
	radio_write_u8(AX_REG_RX_PARAMETER1 + AX_RX_DRGAIN,
			ax_value_to_mantissa_exp_4_4(dr_gain_after));
	radio_write_u8(AX_REG_RX_PARAMETER3 + AX_RX_DRGAIN,
			ax_value_to_mantissa_exp_4_4(dr_gain_during));

	radio_write_u8(AX_REG_RX_PARAMETER0 + AX_RX_PHASEGAIN,
			((filter_idx &0x3)<<6) | (phase_gain & 0xF));
	radio_write_u8(AX_REG_RX_PARAMETER1 + AX_RX_PHASEGAIN,
			((filter_idx &0x3)<<6) | (phase_gain & 0xF));
	radio_write_u8(AX_REG_RX_PARAMETER3 + AX_RX_PHASEGAIN,
			((filter_idx &0x3)<<6) | (phase_gain & 0xF));

	radio_write_u8(AX_REG_RX_PARAMETER0 + AX_RX_FREQUENCYGAINA,
			baseband_rg_phase_det);
	radio_write_u8(AX_REG_RX_PARAMETER1 + AX_RX_FREQUENCYGAINA,
			baseband_rg_phase_det);
	radio_write_u8(AX_REG_RX_PARAMETER3 + AX_RX_FREQUENCYGAINA,
			baseband_rg_phase_det);

	radio_write_u8(AX_REG_RX_PARAMETER0 + AX_RX_FREQUENCYGAINB,
			baseband_rg_freq_det);
	radio_write_u8(AX_REG_RX_PARAMETER1 + AX_RX_FREQUENCYGAINB,
			baseband_rg_freq_det);
	radio_write_u8(AX_REG_RX_PARAMETER3 + AX_RX_FREQUENCYGAINB,
			baseband_rg_freq_det);

	radio_write_u8(AX_REG_RX_PARAMETER0 + AX_RX_FREQUENCYGAINC,
			rffreq_rg);
	radio_write_u8(AX_REG_RX_PARAMETER1 + AX_RX_FREQUENCYGAINC,
			rffreq_rg);
	radio_write_u8(AX_REG_RX_PARAMETER3 + AX_RX_FREQUENCYGAINC,
			rffreq_rg_during);

	radio_write_u8(AX_REG_RX_PARAMETER0 + AX_RX_FREQUENCYGAIND,
			rffreq_rg);
	radio_write_u8(AX_REG_RX_PARAMETER1 + AX_RX_FREQUENCYGAIND,
			rffreq_rg);
	radio_write_u8(AX_REG_RX_PARAMETER3 + AX_RX_FREQUENCYGAIND,
			rffreq_rg_during);

	radio_write_u8(AX_REG_RX_PARAMETER0 + AX_RX_AMPLITUDEGAIN,
			amplflags | amplgain);
	radio_write_u8(AX_REG_RX_PARAMETER1 + AX_RX_AMPLITUDEGAIN,
			amplflags | amplgain);
	radio_write_u8(AX_REG_RX_PARAMETER3 + AX_RX_AMPLITUDEGAIN,
			amplflags | amplgain);

	radio_write_u16(AX_REG_RX_PARAMETER0 + AX_RX_FREQDEV,
			freq_dev_initial);
	radio_write_u16(AX_REG_RX_PARAMETER1 + AX_RX_FREQDEV,
			freq_dev);
	radio_write_u16(AX_REG_RX_PARAMETER3 + AX_RX_FREQDEV,
			freq_dev);

	radio_write_u8(AX_REG_RX_PARAMETER0 + AX_RX_FOURFSK, 0x16);
	radio_write_u8(AX_REG_RX_PARAMETER1 + AX_RX_FOURFSK, 0x16);
	radio_write_u8(AX_REG_RX_PARAMETER3 + AX_RX_FOURFSK, 0x16);

	radio_write_u8(AX_REG_RX_PARAMETER0 + AX_RX_BBOFFSRES, 0x0);
	radio_write_u8(AX_REG_RX_PARAMETER1 + AX_RX_BBOFFSRES, 0x0);
	radio_write_u8(AX_REG_RX_PARAMETER3 + AX_RX_BBOFFSRES, 0x0);

	//TX_params
	radio_write_u8(AX_REG_MODCFGF, AX_MODCFGF_FREQSHAPE_UNSHAPED);
	radio_write_u8(AX_REG_MODCFGA, AX_MODCFGA_AMPLSHAPE_RAISED_COSINE | AX_MODCFGA_TXDIFF);

	uint32_t fskdev = ((float)deviation * (1UL << 24) * 0.858785) / F_CLK + 0.5;
	radio_write_u24(AX_REG_FSKDEV, fskdev);
	uint32_t txrate = ((float)bitrate * (1UL << 24))/ F_CLK + 0.5;
	radio_write_u24(AX_REG_TXRATE, txrate);
	radio_write_u16(AX_REG_TXPWRCOEFFB,0x01ff); // TODO: power setup

	//PLL
	radio_write_u8(AX_REG_PLLVCOI, AX_PLLVCOI_ENABLE_MANUAL | 25);
	radio_write_u8(AX_REG_PLLRNGCLK, AX_PLLRNGCLK_DIV_2048);

	//BASEBAND
	radio_write_u8(AX_REG_BBTUNE, 0x0F);
	radio_write_u8(AX_REG_BBOFFSCAP, 0x77);

	//PACKET
	radio_write_u8(AX_REG_PKTADDRCFG, 0x01);
	radio_write_u8(AX_REG_PKTLENCFG, 0x80); // was 0x80
	radio_write_u8(AX_REG_PKTLENOFFSET, 0x00);
	radio_write_u8(AX_REG_PKTMAXLEN, 0xFF);

	//MATCH
	radio_write_u32(AX_REG_MATCH0PAT, 0x7E7E7E7E);
	radio_write_u8(AX_REG_MATCH0LEN, 0x00);
	radio_write_u8(AX_REG_MATCH0MAX,0x1F);

	radio_write_u16(AX_REG_MATCH1PAT, 0x7E7E);
	radio_write_u8(AX_REG_MATCH1LEN, 0x0A);
	radio_write_u8(AX_REG_MATCH1MAX,10);

	//PACKET_CTRL
	radio_write_u8(AX_REG_TMGTXBOOST,
			ax_value_to_exp_mantissa_3_5(tx_pll_boost_time));
	radio_write_u8(AX_REG_TMGTXSETTLE,
			ax_value_to_exp_mantissa_3_5(tx_pll_settle_time));
	radio_write_u8(AX_REG_TMGRXBOOST,
			ax_value_to_exp_mantissa_3_5(rx_pll_boost_time));
	radio_write_u8(AX_REG_TMGRXSETTLE,
			ax_value_to_exp_mantissa_3_5(rx_pll_settle_time));
	radio_write_u8(AX_REG_TMGRXOFFSACQ, 0x00);
	radio_write_u8(AX_REG_TMGRXCOARSEAGC,
			ax_value_to_exp_mantissa_3_5(rx_coarse_agc));
	radio_write_u8(AX_REG_TMGRXAGC,
			ax_value_to_exp_mantissa_3_5(rx_agc_settling));
	radio_write_u8(AX_REG_TMGRXRSSI,
			ax_value_to_exp_mantissa_3_5(rx_rssi_settling));
	radio_write_u8(AX_REG_TMGRXPREAMBLE2,
			ax_value_to_exp_mantissa_3_5(preamble_2_timeout));
	radio_write_u8(AX_REG_BGNDRSSITHR, 0x00);
	radio_write_u8(AX_REG_PKTCHUNKSIZE, AX_PKT_MAXIMUM_CHUNK_SIZE_240_BYTES);
	radio_write_u8(AX_REG_PKTMISCFLAGS, pkt_misc_flags);
	radio_write_u8(AX_REG_PKTSTOREFLAGS, AX_PKT_STORE_CRC_BYTES);
	radio_write_u8(AX_REG_PKTACCEPTFLAGS,
			AX_PKT_ACCEPT_MULTIPLE_CHUNKS |  /* (LRGP) */
			AX_PKT_ACCEPT_ADDRESS_FAILURES | /* (ADDRF) */
			AX_PKT_ACCEPT_RESIDUE |          /* (RESIDUE) */
			AX_PKT_ACCEPT_SIZE_FAILURES |
			0);

	//LPOSC
	//DAC
	//PERF TUNNING
	radio_write_u8(AX_REG_REF, 0x03); /* 0xF0D */
	radio_write_u8(0xF1C, 0x07); /* const */
	radio_write_u8(0xF21, 0x5c); /* !! */
	radio_write_u8(0xF22, 0x53); /* !! */
	radio_write_u8(0xF23, 0x76); /* !! */
	radio_write_u8(0xF26, 0x92); /* !! */
	radio_write_u8(0xF44, 0x25); /* !! */

	//SYNTH - done in tuning
}
void radio_tx_on(void)
{
	radio_write_u8(AX_REG_PWRMODE, AX_PWRMODE_STANDBY | 0x60);
	radio_write_u8(AX_REG_PWRMODE, AX_PWRMODE_FIFOON | 0x60);

	//AFSK
	uint16_t afskmark = ((uint64_t)1200 * (1UL << 18) * 2 + F_CLK/2)/F_CLK;
	uint16_t afskspace = ((uint64_t)2200 * (1UL << 18) * 2 + F_CLK/2)/F_CLK;
	radio_write_u16(AX_REG_AFSKMARK, afskmark);
	radio_write_u16(AX_REG_AFSKSPACE, afskspace);

	radio_write_u8(0xF00, 0x0F); /* const */
	radio_write_u8(0xF18, 0x06); /* ?? */

	radio_write_u8(AX_REG_FIFOSTAT, AX_FIFOCMD_CLEAR_FIFO_DATA_AND_FLAGS);

	radio_write_u8(AX_REG_PWRMODE, AX_PWRMODE_FULLTX | 0x60);

	/* Wait for oscillator to be stable */
	while (!(radio_read_u8(AX_REG_XTALSTATUS) & 1)) {
	};
}

void radio_rx_on(void)
{
	radio_write_u8(AX_REG_PWRMODE, AX_PWRMODE_FULLRX | 0x60);

	//AFSK
	uint16_t afskmark = ((uint64_t)1200 * (1UL << 16) * decimation * XTALDIV + F_CLK/2)/F_CLK;
	uint16_t afskspace = ((uint64_t)2200 * (1UL << 16) * decimation * XTALDIV + F_CLK/2)/F_CLK;
	radio_write_u16(AX_REG_AFSKMARK, afskmark);
	radio_write_u16(AX_REG_AFSKSPACE, afskspace);

	radio_write_u16(AX_REG_AFSKCTRL,afskshift);

	radio_write_u8(0xF00, 0x0F); /* const */
	radio_write_u8(0xF18, 0x02); /* ?? */

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

	radio_set_freq(144800000);
	radio_setup_modulation();
	radio_rx_on();
}
