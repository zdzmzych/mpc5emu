#ifndef AD7784_EMU_H
#define AD7784_EMU_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include "CircularBuffer.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Rejestry (zgodne z AD778x) ===== */
#define AD7784_REG_STATUS        0
#define AD7784_REG_MODE          1
#define AD7784_REG_CONFIG        2
#define AD7784_REG_DATA          3
#define AD7784_REG_ID            4
#define AD7784_REG_IO            5
#define AD7784_REG_OFFSET        6
#define AD7784_REG_FULLSCALE     7

/* Status bits */
#define AD7784_STATUS_RDY        (1 << 7)   /* 0 = data ready */
#define AD7784_STATUS_ERR        (1 << 6)
#define AD7784_STATUS_NOREX      (1 << 5)
#define AD7784_STATUS_PARITY     (1 << 4)
/* bit 3 zawsze 1 na AD7785, na AD7784 też ustawiamy */

typedef struct {
    /* Rejestry */
    uint8_t  status;
    uint16_t mode;
    uint16_t config;
    uint32_t data;          /* 24-bit */
    uint8_t  id;
    uint8_t  io;
    uint32_t offset;
    uint32_t fullscale;

    /* Stan maszyny SPI */
    uint8_t  next_reg;
    uint8_t  is_read;       /* 1 = read */
    uint8_t  cread;         /* continuous read */
    uint8_t  bytes_to_xfer;
    uint8_t  byte_idx;

    uint8_t  tx_buf[4];
    uint8_t  rx_buf[4];

    /* Konwersja */
    uint32_t conversion_period_ms;
    uint32_t last_conversion_tick;
    bool     data_ready;

    /* Hardware */
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef      *rdy_port;
    uint16_t           rdy_pin;
    CircularBuffer 	  *pcb;
} AD7784_Emu_t;

extern AD7784_Emu_t ad7784;

/* API */
void AD7784_Emu_Init(SPI_HandleTypeDef *hspi, CircularBuffer *cb,
                     GPIO_TypeDef *rdy_port, uint16_t rdy_pin);

void AD7784_Emu_Reset(void);
void AD7784_Emu_Process(void);                 /* wywoływać w main loop */
void AD7784_Emu_SetData(uint32_t value_24bit); /* stała wartość */

/* Callbacki SPI (wywoływane z interruptów) */
void AD7784_Emu_SPI_RxTxCplt(void);
void AD7784_Emu_SPI_Error(void);
void AD7784_Emu_CS_Falling(void);              /* opcjonalnie – na CS falling */

#ifdef __cplusplus
}
#endif

#endif /* AD7784_EMU_H */
