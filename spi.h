/*
 * spi.h
 *
 *  Created on: Dec 31, 2021
 *      Author: kosa
 */

#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>

void spi_start(void);

void spi_transfer(uint8_t *buff, uint8_t len);
void spi_transfer_start(void);
uint8_t spi_transfer_cont(uint8_t data);
void spi_transfer_end(void);



#endif /* SPI_H_ */
