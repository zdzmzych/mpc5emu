#ifndef AD7794_EMU_H
#define AD7794_EMU_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AD7794 register addresses - RS[2:0]
 * ============================================================ */

#define AD7794_REG_STATUS        0u
#define AD7794_REG_MODE          1u
#define AD7794_REG_CONFIG        2u
#define AD7794_REG_DATA          3u
#define AD7794_REG_ID            4u
#define AD7794_REG_IO            5u
#define AD7794_REG_OFFSET        6u
#define AD7794_REG_FULLSCALE     7u

/* ============================================================
 * Communications Register
 *
 * Bit 7 = WEN
 * Bit 6 = R/W
 * Bit 5:3 = RS2:RS0
 * Bit 2 = CREAD
 * Bit 1:0 = 0
 * ============================================================ */

#define AD7794_COMM_WEN          0x80u
#define AD7794_COMM_READ         0x40u
#define AD7794_COMM_CREAD        0x04u

#define AD7794_COMM_READ_STATUS  0x40u
#define AD7794_COMM_READ_MODE    0x48u
#define AD7794_COMM_READ_CONFIG  0x50u
#define AD7794_COMM_READ_DATA    0x58u
#define AD7794_COMM_READ_ID      0x60u
#define AD7794_COMM_READ_IO      0x68u
#define AD7794_COMM_READ_OFFSET  0x70u
#define AD7794_COMM_READ_FS      0x78u

#define AD7794_COMM_WRITE_MODE   0x08u
#define AD7794_COMM_WRITE_CONFIG 0x10u
#define AD7794_COMM_WRITE_IO     0x28u
#define AD7794_COMM_WRITE_OFFSET 0x30u
#define AD7794_COMM_WRITE_FS     0x38u

#define AD7794_COMM_CREAD_DATA   0x5Cu
#define AD7794_COMM_EXIT_CREAD   0x58u

/* ============================================================
 * Status Register
 * ============================================================ */

#define AD7794_STATUS_RDY        (1u << 7)
#define AD7794_STATUS_ERR        (1u << 6)
#define AD7794_STATUS_NOXREF     (1u << 5)
#define AD7794_STATUS_CH_MASK    0x07u

/* ============================================================
 * Default values after reset
 *
 * According to AD7794 datasheet:
 * STATUS = 0x88
 * MODE   = 0x000A
 * CONFIG = 0x0710
 *
 * AD7794 ID on known silicon = 0x4F
 * ============================================================ */

#define AD7794_DEFAULT_STATUS    0x88u
#define AD7794_DEFAULT_MODE      0x000Au
#define AD7794_DEFAULT_CONFIG    0x0710u
#define AD7794_DEFAULT_ID        0x4Fu

#define AD7794_DEFAULT_IO        0x00u
#define AD7794_DEFAULT_OFFSET    0x800000u

/*
 * Wartość domyślna używana przez emulator.
 * Możesz ją później zmienić.
 */
#define AD7794_DEFAULT_FULLSCALE 0x5FFFFFu

#define AD7794_RESET_BITS        32u

/* ============================================================
 * SPI state machine
 * ============================================================ */

typedef enum
{
    AD7794_SPI_WAIT_COMM = 0,
    AD7794_SPI_READ,
    AD7794_SPI_WRITE,
    AD7794_SPI_CREAD_WAIT,
    AD7794_SPI_CREAD_DATA
} AD7794_SpiState_t;

/* ============================================================
 * Emulator structure
 * ============================================================ */

typedef struct
{
    /* ---------------- Registers ---------------- */

    uint8_t  status;

    uint16_t mode;
    uint16_t config;

    uint32_t data;          /* only 24 bits */

    uint8_t  id;
    uint8_t  io;

    uint32_t offset;        /* only 24 bits */
    uint32_t fullscale;     /* only 24 bits */

    /* ---------------- Communications ---------------- */

    uint8_t  next_reg;
    uint8_t  is_read;
    uint8_t  cread;

    AD7794_SpiState_t spi_state;

    uint8_t  bytes_to_xfer;
    uint8_t  byte_idx;

    uint8_t  tx_buf[4];
    uint8_t  rx_buf[4];

    /*
     * Liczba kolejnych bitów '1' na DIN.
     *
     * AD7794 resetuje interfejs po minimum 32 kolejnych
     * taktowanych bitach z DIN = 1.
     */
    uint8_t reset_one_bits;

    /* ---------------- Conversion ---------------- */

    uint32_t conversion_period_ms;
    uint32_t last_conversion_tick;

    bool data_ready;

    /* ---------------- CS ---------------- */

    bool cs_active;

} AD7794_Emu_t;

extern AD7794_Emu_t ad7794;

/* ============================================================
 * API
 * ============================================================ */

void AD7794_Emu_Init(void);

void AD7794_Emu_Reset(void);

void AD7794_Emu_Process(void);

void AD7794_Emu_SetData(uint32_t value_24bit);

/* ============================================================
 * Chip Select
 * ============================================================ */

void AD7794_Emu_CS_Activate(void);

void AD7794_Emu_CS_Deactivate(void);

/* ============================================================
 * SPI
 *
 * Wywoływane z SPI1_IRQHandler() po odebraniu bajtu.
 * ============================================================ */

void AD7794_Emu_SPI_RxTxCplt(uint8_t data);

void AD7794_Emu_SPI_Error(void);

#ifdef __cplusplus
}
#endif

#endif /* AD7794_EMU_H */
