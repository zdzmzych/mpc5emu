/*
 * CircularBuffer.h
 *
 *  Created on: 31 lip 2026
 *      Author: mzych
 */

#ifndef INC_CIRCULARBUFFER_H_
#define INC_CIRCULARBUFFER_H_

#include "main.h"
#include <string.h>
#include <stdbool.h>

#define BUFFER_SIZE 128 // Musi być potęgą dwójki (2, 4, 8, 16, 32, 64, 128, 256...)
#define BUFFER_MASK (BUFFER_SIZE - 1)

typedef struct {
    uint8_t data[BUFFER_SIZE];
    volatile uint16_t head; // Indeks dopisywania (zwiększany przez SPI)
    volatile uint16_t tail; // Indeks odczytu (zwiększany przez UART)
} CircularBuffer;

bool cb_push(CircularBuffer *cb, uint8_t byte);
bool cb_pusha(CircularBuffer *cb, uint8_t *bytes, uint8_t count);
bool cb_pop(CircularBuffer *cb, uint8_t *byte);

#endif /* INC_CIRCULARBUFFER_H_ */
