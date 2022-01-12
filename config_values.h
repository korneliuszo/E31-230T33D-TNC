/*
 * config_values.h
 *
 *  Created on: Jan 12, 2022
 *      Author: kosa
 */

#ifndef CONFIG_VALUES_H_
#define CONFIG_VALUES_H_

#include <stdint.h>

typedef struct
{
	uint16_t reg;
	uint8_t val;
} register_pair;

extern register_pair config_common[];
extern register_pair config_rx[];
extern register_pair config_tx[];



#endif /* CONFIG_VALUES_H_ */
