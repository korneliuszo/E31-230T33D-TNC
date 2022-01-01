/*
 * bits.h
 *
 *  Created on: Dec 27, 2021
 *      Author: kosa
 */

#ifndef BITS_H_
#define BITS_H_

#include <stdint.h>

inline void SBI(uint8_t * reg, uint8_t bitmask)
{
	*reg |= bitmask;
}

inline void CBI(uint8_t * reg, uint8_t bitmask)
{
	*reg &= ~bitmask;
}


#endif /* BITS_H_ */
