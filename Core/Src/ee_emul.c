#include "main.h"
#include "ee_emul.h"

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/*
 * 25LC320
 *
 * 32 kbit = 4096 bytes
 */
#define EE_MEMORY_SIZE             0x1000u

#define EE_CMD_WRITE               0x02u
#define EE_CMD_READ                0x03u

/*
 * EEPROM SPI states.
 */
typedef enum
{
    EE_STATE_WAIT_COMMAND = 0,
    EE_STATE_WAIT_ADDR_H,
    EE_STATE_WAIT_ADDR_L,
    EE_STATE_WRITE,
    EE_STATE_READ

} EE_StateInternal_t;


/*
 * EEPROM contents.
 */
static uint8_t eeprom_memory[EE_MEMORY_SIZE];


static const uint8_t eeprom_default_image[0x200] =
{
		0x50, 0x97, 0x5C, 0xEF, 0xC1, 0x8F, 0x05, 0x8B, 0x0E, 0x8D, 0xD8, 0xF5, 0xAE, 0x89, 0xB8, 0x09, 0x0D, 0x8A, 0xC9, 0x0A, 0x48, 0x84, 0x00, 0x00, 0x1A, 0x97, 0xAD, 0x40, 0xED, 0x91, 0xC9, 0xBE,
		0xD2, 0x8B, 0xBA, 0x1A, 0x5E, 0x87, 0x4D, 0x03, 0x4F, 0x8A, 0xE1, 0x9C, 0x48, 0x84, 0x00, 0x00, 0x60, 0x9D, 0x00, 0xCC, 0xAA, 0x9A, 0xCD, 0x3A, 0x70, 0x93, 0xD5, 0x66, 0x31, 0x94, 0xA3, 0xA7,
		0x30, 0x97, 0x4A, 0x56, 0xE8, 0x98, 0xD0, 0x43, 0x2F, 0x97, 0x16, 0x6A, 0x50, 0x93, 0xFB, 0x12, 0x61, 0x93, 0x11, 0x30, 0x7F, 0x96, 0xB5, 0xED, 0x5D, 0x75, 0x4C, 0xF7, 0xB1, 0x76, 0x37, 0x3E,
		0xD1, 0x78, 0x48, 0x16, 0x00, 0x80, 0xA7, 0xE1, 0x64, 0x4B, 0x2E, 0xF9, 0xA5, 0x7E, 0xD3, 0x8B, 0xED, 0x7A, 0x43, 0xC5, 0x39, 0x79, 0x32, 0x04, 0x7F, 0x7F, 0x9F, 0xFE, 0x25, 0x7E, 0xD3, 0x8B,
		0x6D, 0x7A, 0x43, 0xC5, 0xB9, 0x79, 0x32, 0x04, 0x30, 0x70, 0x77, 0xB1, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x83, 0x75, 0xAD, 0x37,
		0x0A, 0x8D, 0x42, 0x25, 0xFF, 0xFF, 0xFF, 0xFF, 0x2F, 0x89, 0x00, 0x00, 0xAF, 0x89, 0x00, 0x00, 0x06, 0x00, 0x18, 0xA1, 0x60, 0x0C, 0x3A, 0x7F, 0x35, 0x5E, 0xBA, 0x7F, 0x35, 0x5E, 0x18, 0x79,
		0xB1, 0xC3, 0x02, 0x00, 0x06, 0xFF, 0x08, 0x8A, 0x00, 0x80, 0x88, 0x8A, 0x00, 0x80, 0x16, 0x86, 0x00, 0x00, 0xC8, 0x84, 0x00, 0x00, 0x50, 0x95, 0xE7, 0x86, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7E, 0xDB, 0x70, 0x81,
		0x0E, 0x4F, 0x84, 0x4C, 0x83, 0xD6, 0x7D, 0xA2, 0x4F, 0xDB, 0xC4, 0x1D, 0x63, 0x00, 0x19, 0x02, 0x1A, 0x80, 0x80, 0x02, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x04, 0xFF, 0xFF, 0x89, 0x7C,
};



/*
 * Current SPI state.
 */
static volatile EE_StateInternal_t state;


/*
 * Current EEPROM command/address.
 */
static volatile uint8_t  ee_cmd;
static volatile uint16_t ee_address;
static volatile bool     ee_cs_active;


/*
 * --------------------------------------------------------------------------
 * SPI helpers
 * --------------------------------------------------------------------------
 */


/*
 * PA6 = SPI1_MISO
 */
static void PA6_As_MISO(void)
{
    LL_GPIO_SetPinMode(GPIOA,
                       LL_GPIO_PIN_6,
                       LL_GPIO_MODE_ALTERNATE);
}


/*
 * Put byte into SPI TX register.
 *
 * IMPORTANT:
 *
 * This function is called AFTER a received byte has been processed,
 * therefore the value prepared here will be transmitted during the
 * NEXT SPI transfer.
 */
static void EE_PrepareTx(uint8_t value)
{
    while (!LL_SPI_IsActiveFlag_TXE(SPI1));
    LL_SPI_TransmitData8(SPI1, value);
}


/*
 * Clear stale RX / overrun state.
 *
 * This is especially important when CS changes between devices.
 */
static void EE_SPI_ClearPending(void)
{
    if (LL_SPI_IsActiveFlag_RXNE(SPI1))
    {
        (void)LL_SPI_ReceiveData8(SPI1);
    }

    if (LL_SPI_IsActiveFlag_OVR(SPI1))
    {
        (void)LL_SPI_ReceiveData8(SPI1);
        (void)SPI1->SR;
    }
}


/*
 * --------------------------------------------------------------------------
 * EEPROM initialization
 * --------------------------------------------------------------------------
 */

void EE_Emul_Init(void)
{
    /*
     * Real EEPROM after erase is normally FF.
     */
    memset(eeprom_memory,
           0xFF,
           sizeof(eeprom_memory));


    /*
     * Load the existing EEPROM image.
     *
     * KEEP YOUR EXISTING eeprom_default_image[].
     */
    memcpy(eeprom_memory,
           eeprom_default_image,
           sizeof(eeprom_default_image));


    state = EE_STATE_WAIT_COMMAND;

    ee_cmd = 0;
    ee_address = 0;
    ee_cs_active = false;
}


/*
 * --------------------------------------------------------------------------
 * CS LOW
 * --------------------------------------------------------------------------
 */

void EE_Emul_CS_Activate(void)
{
    ee_cs_active = true;
    state = EE_STATE_WAIT_COMMAND;
    ee_cmd = 0;
    ee_address = 0;

    EE_SPI_ClearPending();
    PA6_As_MISO();
    EE_PrepareTx(0x00);
    LL_SPI_EnableIT_RXNE(SPI1);
}

/*
 * --------------------------------------------------------------------------
 * CS HIGH
 * --------------------------------------------------------------------------
 */

void EE_Emul_CS_Deactivate(void)
{
    ee_cs_active = false;

    state = EE_STATE_WAIT_COMMAND;

    ee_cmd = 0;
    ee_address = 0;


    /*
     * No EEPROM RX processing while CS is inactive.
     */
    LL_SPI_DisableIT_RXNE(SPI1);


    /*
     * Clear possible RX/OVR left by the last transfer.
     */
    EE_SPI_ClearPending();
}


/*
 * --------------------------------------------------------------------------
 * EEPROM direct access
 * --------------------------------------------------------------------------
 */

void EE_Emul_Write(uint16_t address, uint8_t value)
{
    eeprom_memory[
        address & (EE_MEMORY_SIZE - 1u)
    ] = value;
}


/*
 * --------------------------------------------------------------------------
 * SPI RX/TX
 *
 * VERY IMPORTANT:
 *
 * This function is called AFTER one complete SPI byte has been received.
 *
 * Therefore the value returned from this function is NOT the byte that
 * was just received.
 *
 * It is the byte that must be put into TX for the NEXT SPI transfer.
 * --------------------------------------------------------------------------
 */

uint8_t EE_Emul_SPI_RxTx(uint8_t rx)
{
    uint8_t tx = 0x00;


    if (!ee_cs_active)
    {
        return 0xFF;
    }


    switch (state)
    {
        /*
         * ==============================================================
         * COMMAND
         * ==============================================================
         */

        case EE_STATE_WAIT_COMMAND:

            if (rx == EE_CMD_READ)
            {
                ee_cmd = EE_CMD_READ;

                state = EE_STATE_WAIT_ADDR_H;

                /*
                 * Response to command byte.
                 */
                tx = 0xa5;
            }
            else if (rx == EE_CMD_WRITE)
            {
                ee_cmd = EE_CMD_WRITE;

                state = EE_STATE_WAIT_ADDR_H;

                /*
                 * Response to command byte.
                 */
                tx = 0x00;
            }
            else
            {
                /*
                 * Invalid command.
                 */
                state = EE_STATE_WAIT_COMMAND;

                tx = 0x00;
            }

            break;


        /*
         * ==============================================================
         * ADDRESS HIGH
         * ==============================================================
         */

        case EE_STATE_WAIT_ADDR_H:

            ee_address =
                ((uint16_t)rx << 8);

            state = EE_STATE_WAIT_ADDR_L;

            tx = 0xa6;

            break;


        /*
         * ==============================================================
         * ADDRESS LOW
         * ==============================================================
         */

        case EE_STATE_WAIT_ADDR_L:

            ee_address |= rx;

            /*
             * 25LC320 has 12-bit address.
             */
            ee_address &=
                (EE_MEMORY_SIZE - 1u);


            if (ee_cmd == EE_CMD_READ)
            {
                /*
                 * ------------------------------------------------------
                 * THIS IS THE IMPORTANT FIX.
                 *
                 * The first EEPROM byte must be placed into TX NOW,
                 * before the master starts the next clock.
                 * ------------------------------------------------------
                 */

                tx = eeprom_memory[ee_address & (EE_MEMORY_SIZE - 1u)];


                /*
                 * Move address forward immediately.
                 *
                 * Therefore the next SPI transfer will return
                 * address + 1, not the same byte again.
                 */
                ee_address =
                    (ee_address + 1u) &
                    (EE_MEMORY_SIZE - 1u);


                state = EE_STATE_READ;
            }
            else if (ee_cmd == EE_CMD_WRITE)
            {
                state = EE_STATE_WRITE;

                tx = 0x00;
            }
            else
            {
                state = EE_STATE_WAIT_COMMAND;

                tx = 0x00;
            }

            break;


        /*
         * ==============================================================
         * READ
         * ==============================================================
         */

        case EE_STATE_READ:

            /*
             * The byte currently being received from MOSI is irrelevant
             * for a normal EEPROM READ.
             *
             * We prepare the NEXT EEPROM byte.
             */
            tx = eeprom_memory[ee_address & (EE_MEMORY_SIZE - 1u)];


            /*
             * Sequential read.
             *
             * 25LC320 wraps around at the end of the address space.
             */
            ee_address =
                (ee_address + 1u) &
                (EE_MEMORY_SIZE - 1u);

            break;


        /*
         * ==============================================================
         * WRITE
         * ==============================================================
         */

        case EE_STATE_WRITE:

            EE_Emul_Write(ee_address, rx);


            ee_address =
                (ee_address + 1u) &
                (EE_MEMORY_SIZE - 1u);


            /*
             * During WRITE MISO is not important.
             */
            tx = 0x00;

            break;


        default:

            state = EE_STATE_WAIT_COMMAND;

            tx = 0x00;

            break;
    }


    return tx;
}


/*
 * --------------------------------------------------------------------------
 * Background processing
 * --------------------------------------------------------------------------
 */

void EE_Emul_Process(void)
{
    /*
     * Nothing required.
     *
     * EEPROM SPI communication is handled entirely from SPI RX interrupt.
     */
}
