#include "CircularBuffer.h"

#define BUFFER_SIZE 128 // Musi być potęgą dwójki (2, 4, 8, 16, 32, 64, 128, 256...)
#define BUFFER_MASK (BUFFER_SIZE - 1)

uint8_t data[BUFFER_SIZE];
volatile uint16_t head; // Indeks dopisywania (zwiększany przez SPI)
volatile uint16_t tail; // Indeks odczytu (zwiększany przez UART)

bool cb_push(uint8_t byte) {
    uint16_t next_head = (head + 1) & BUFFER_MASK;

    // Sprawdzenie, czy bufor nie jest pełny
    if (next_head == tail) {
        return false; // Przepełnienie bufora (błąd, brak miejsca)
    }

    data[head] = byte;
    head = next_head;
    return true;
}

bool cb_pop(uint8_t *byte) {
    if (head == tail) {
        return false;
    }

    *byte = data[tail];
    tail = (tail + 1) & BUFFER_MASK;
    return true;
}

