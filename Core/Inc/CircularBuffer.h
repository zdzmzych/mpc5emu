/*
 * CircularBuffer.h
 *
 *  Created on: 31 lip 2026
 *      Author: mzych
 */

#ifndef INC_CIRCULARBUFFER_H_
#define INC_CIRCULARBUFFER_H_

#include <stdint.h>
#include <stdbool.h>

bool cb_push(uint8_t byte);
bool cb_pop(uint8_t *byte);

#endif /* INC_CIRCULARBUFFER_H_ */
