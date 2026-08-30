#ifndef AD7794_EMU_H
#define AD7794_EMU_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Rejestry (mapowanie RS[2:0]) */
#define AD7794_REG_STATUS        0
#define AD7794_REG_MODE          1
#define AD7794_REG_CONFIG        2
#define AD7794_REG_DATA          3
#define AD7794_REG_ID            4
#define AD7794_REG_IO            5
#define AD7794_REG_OFFSET        6
#define AD7794_REG_FULLSCALE     7

/* Status */
#define AD7794_STATUS_RDY        (1u << 7)  /* 0 = data ready */
#define AD7794_STATUS_ERR        (1u << 6)
#define AD7794_STATUS_NOREX      (1u << 5)
#define AD7794_STATUS_PARITY     (1u << 4)
#define AD7794_STATUS_CH_MASK    0x07u

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

    /* Stan SPI */
    uint8_t  next_reg;
    uint8_t  is_read;
    uint8_t  cread;
    uint8_t  bytes_to_xfer;
    uint8_t  bytes_to_read;
    uint8_t  byte_idx;
    uint8_t  ff_count;      /* soft-reset: licznik 0xFF */

    uint8_t  tx_buf[4];
    uint8_t  rx_buf[4];

    /* Konwersja */
    uint32_t conversion_period_ms;
    uint32_t last_conversion_tick;
    bool     data_ready;
    bool     cs_active;

    /* Hardware */
} AD7794_Emu_t;

extern AD7794_Emu_t ad7794;

/* API */
void AD7794_Emu_Init();
void AD7794_Emu_Reset(void);
void AD7794_Emu_Process(void);
void AD7794_Emu_SetData(uint32_t value_24bit);

/* CS z EXTI */
void AD7794_Emu_CS_Activate(void);
void AD7794_Emu_CS_Deactivate(void);

/* Callbacki SPI */
void AD7794_Emu_SPI_RxTxCplt(uint8_t data);
void AD7794_Emu_SPI_Error(void);

#ifdef __cplusplus
}
#endif

#endif /* AD7794_EMU_H */
