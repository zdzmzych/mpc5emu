/*
 * CircularBuffer.c
 *
 *  Created on: 31 lip 2026
 *      Author: mzych
 */


#include "CircularBuffer.h"



// Funkcja dodająca pojedynczy bajt (wywoływana w przerwaniu SPI)
bool cb_push(CircularBuffer *cb, uint8_t byte) {
    uint16_t next_head = (cb->head + 1) & BUFFER_MASK;

    // Sprawdzenie, czy bufor nie jest pełny
    if (next_head == cb->tail) {
        return false; // Przepełnienie bufora (błąd, brak miejsca)
    }

    cb->data[cb->head] = byte;
    cb->head = next_head;
    return true;
}

// Funkcja pobierająca pojedynczy bajt (wywoływana w pętli głównej)
bool cb_pop(CircularBuffer *cb, uint8_t *byte) {
    // Sprawdzenie, czy bufor jest pusty
    if (cb->head == cb->tail) {
        return false;
    }

    *byte = cb->data[cb->tail];
    cb->tail = (cb->tail + 1) & BUFFER_MASK;
    return true;
}

bool cb_pusha(CircularBuffer *cb, uint8_t *bytes, uint8_t count)
{
	for(int i=0; i<count;i++)
	{
		cb_push(cb, bytes[i]);
	}
}

