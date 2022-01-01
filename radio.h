/*
 * radio.h
 *
 *  Created on: Dec 31, 2021
 *      Author: kosa
 */

#ifndef RADIO_H_
#define RADIO_H_

#include <stdint.h>

void radio_start(void);
void radio_rx_on(void);
void radio_tx_on(void);
void radio_rx_data(void);
void radio_tx_data_packet(uint8_t *data, uint16_t length);


#endif /* RADIO_H_ */
