#include "main.h"
#include "ee_emul.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#define EE_DATA_SIZE               16u

#define EE_CMD_WRITE               0x02u
#define EE_CMD_READ                0x03u

/*
 * 25LC320 = 4096 bytes
 */
#define EE_MEMORY_SIZE             0x1000u

typedef enum
{
    EE_SPI_IDLE = 0,
    EE_SPI_WAIT_CMD,
    EE_SPI_WAIT_ADDRESS_HIGH,
    EE_SPI_WAIT_ADDRESS_LOW,
    EE_SPI_WRITE_DATA,
    EE_SPI_READ_DATA,
    EE_SPI_WAIT_END

} EE_SpiState_t;

static uint8_t eeprom_memory[EE_MEMORY_SIZE];

static volatile EE_SpiState_t ee_state;
static volatile uint8_t ee_cmd;
static volatile uint16_t ee_address;
static volatile uint8_t ee_data_index;
static volatile bool ee_cs_active;

static const uint8_t eeprom_default_image[0x200] =
{
    /* TU WSTAW DOKŁADNIE ISTNIEJĄCY INITIALIZER
       z obecnego ee_emul.c */
};

typedef enum
{
    EE_STATE_WAIT_COMMAND,
    EE_STATE_WAIT_ADDR_H,
    EE_STATE_WAIT_ADDR_L,
    EE_STATE_WRITE,
    EE_STATE_READ,
} EE_StateInternal_t;

static volatile EE_StateInternal_t state;

static void EE_Tx(uint8_t value)
{
    if (LL_SPI_IsActiveFlag_TXE(SPI1))
    {
        LL_SPI_TransmitData8(SPI1, value);
    }
}

void EE_Emul_Init(void)
{
    memset(eeprom_memory,
           0xFF,
           sizeof(eeprom_memory));

    memcpy(eeprom_memory,
           eeprom_default_image,
           sizeof(eeprom_default_image));

    state = EE_STATE_WAIT_COMMAND;

    ee_cmd = 0;
    ee_address = 0;
    ee_data_index = 0;
    ee_cs_active = false;
}

void EE_Emul_CS_Activate(void)
{
    ee_cs_active = true;

    state = EE_STATE_WAIT_COMMAND;

    ee_cmd = 0;
    ee_address = 0;
    ee_data_index = 0;

    /*
     * Slave musi mieć coś w TX zanim master zacznie zegar.
     */
    EE_Tx(0x00);

    LL_SPI_EnableIT_RXNE(SPI1);
}

void EE_Emul_CS_Deactivate(void)
{
    ee_cs_active = false;

    state = EE_STATE_WAIT_COMMAND;

    ee_cmd = 0;
    ee_address = 0;
    ee_data_index = 0;

    LL_SPI_DisableIT_RXNE(SPI1);
}

uint8_t EE_Emul_Read(uint16_t address)
{
    return eeprom_memory[
        address & (EE_MEMORY_SIZE - 1u)
    ];
}

void EE_Emul_Write(uint16_t address, uint8_t value)
{
    eeprom_memory[
        address & (EE_MEMORY_SIZE - 1u)
    ] = value;
}

uint8_t EE_Emul_SPI_RxTx(uint8_t rx)
{
    uint8_t tx = 0x00;

    if (!ee_cs_active)
        return 0xFF;

    switch (state)
    {
        case EE_STATE_WAIT_COMMAND:

            if (rx == EE_CMD_WRITE || rx == EE_CMD_READ)
            {
                ee_cmd = rx;
                state = EE_STATE_WAIT_ADDR_H;
            }
            EE_Tx(0x00);
            return 0x00;


        case EE_STATE_WAIT_ADDR_H:

            ee_address =
                ((uint16_t)rx << 8);

            state = EE_STATE_WAIT_ADDR_L;

            EE_Tx(0x00);
            return 0x00;


        case EE_STATE_WAIT_ADDR_L:

            ee_address |= rx;

            ee_address &=
                (EE_MEMORY_SIZE - 1u);

            ee_data_index = 0;

            if (ee_cmd == EE_CMD_WRITE)
            {
                state = EE_STATE_WRITE;

                EE_Tx(0x00);

                return 0x00;
            }

            if (ee_cmd == EE_CMD_READ)
            {
                state = EE_STATE_READ;

                /*
                 * Najważniejszy moment:
                 *
                 * po adresie 0x0FF0 przygotowujemy pierwszy
                 * bajt EEPROM na MISO.
                 */
                tx = EE_Emul_Read(ee_address);

                EE_Tx(tx);

                return tx;
            }

            EE_Tx(0x00);

            return 0x00;


        case EE_STATE_WRITE:

            /*
             * Master:
             *
             * 00 01 02 ... 0F
             */
            EE_Emul_Write(
                ee_address,
                rx);

            ee_address =
                (uint16_t)(
                    (ee_address + 1u) &
                    (EE_MEMORY_SIZE - 1u));

            ee_data_index++;

            EE_Tx(0x00);

            return 0x00;


        case EE_STATE_READ:

            /*
             * rx = FF jest dummy byte.
             *
             * TX zawierał już aktualny bajt.
             */

            tx = EE_Emul_Read(ee_address);

            ee_address =
                (uint16_t)(
                    (ee_address + 1u) &
                    (EE_MEMORY_SIZE - 1u));

            ee_data_index++;

            if (ee_data_index >= EE_DATA_SIZE)
            {
                state = EE_STATE_WAIT_COMMAND;

                EE_Tx(0x00);
            }
            else
            {
                /*
                 * Przygotuj kolejny bajt.
                 */
                EE_Tx(
                    EE_Emul_Read(
                        ee_address));
            }

            return tx;


        default:

            state = EE_STATE_WAIT_COMMAND;

            EE_Tx(0x00);

            return 0x00;
    }
}

void EE_Emul_Process(void)
{
}
