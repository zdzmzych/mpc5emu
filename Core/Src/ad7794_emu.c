#include "ad7794_emu.h"

#include <string.h>

/*
 * AD7794 emulator v3
 *
 * Main changes versus the previous version:
 *  - RDY is a conversion/status signal and is NOT forced high by CS.
 *  - 0xFF is treated as dummy data / serial-reset pattern, not as a
 *    Communications Register command.
 *  - Four consecutive 0xFF bytes (32 consecutive '1' bits) perform the
 *    AD7794 serial-interface reset.
 *  - 0x58 = normal DATA read.
 *  - 0x5C = DATA read with CREAD enabled.
 *  - CREAD survives CS deassertion, as it is a register/interface mode,
 *    and is cleared by a later Communications byte with CREAD=0.
 *  - STATUS is not destroyed when it is read.
 *  - DATA read clears RDY only after the complete 24-bit transfer.
 *  - Missing CS_Activate implementation is provided.
 */

AD7794_Emu_t ad7794;

/* -------------------------------------------------------------------------- */
/* GPIO helpers                                                              */
/* -------------------------------------------------------------------------- */

static void PA6_As_RDY(void)
{
	if (LL_GPIO_IsInputPinSet(CS_ADC_GPIO_Port, CS_ADC_Pin))
		return;
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6);
}

static void PA6_As_MISO(void)
{
	if (LL_GPIO_IsInputPinSet(CS_ADC_GPIO_Port, CS_ADC_Pin))
		return;
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_ALTERNATE);
}

static void PA6_RDY_Low(void)
{
	if (LL_GPIO_IsInputPinSet(CS_ADC_GPIO_Port, CS_ADC_Pin))
		return;

    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6);
}

static void PA6_RDY_High(void)
{
	if (LL_GPIO_IsInputPinSet(CS_ADC_GPIO_Port, CS_ADC_Pin))
		return;

    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6);
}

static void update_rdy_pin(void)
{
    /*
     * RDY is intentionally independent of CS.
     *
     * PA6 is temporarily switched to SPI alternate function while a READ
     * transfer is active. As soon as that transfer ends (or CS goes HIGH),
     * PA6 returns to the RDY output state.
     */
    if (ad7794.status & AD7794_STATUS_RDY)
        PA6_RDY_High();
    else
        PA6_RDY_Low();
}

/* -------------------------------------------------------------------------- */
/* SPI helpers                                                               */
/* -------------------------------------------------------------------------- */

static void SPI_ClearPending(void)
{
    if (LL_SPI_IsActiveFlag_RXNE(SPI1))
        (void)LL_SPI_ReceiveData8(SPI1);

    if (LL_SPI_IsActiveFlag_OVR(SPI1))
    {
        (void)LL_SPI_ReceiveData8(SPI1);
        (void)SPI1->SR;
    }
}

static void SPI_PrepareTx(uint8_t data)
{
    if (LL_SPI_IsActiveFlag_TXE(SPI1))
        LL_SPI_TransmitData8(SPI1, data);
}

/* -------------------------------------------------------------------------- */
/* Register -> TX buffer                                                     */
/* -------------------------------------------------------------------------- */

static void prepare_tx_buffer(void)
{
    ad7794.byte_idx = 0;

    switch (ad7794.next_reg)
    {
        case AD7794_REG_STATUS:
            ad7794.tx_buf[0] = ad7794.status;
            ad7794.bytes_to_xfer = 1;
            break;

        case AD7794_REG_MODE:
            ad7794.tx_buf[0] = (uint8_t)(ad7794.mode >> 8);
            ad7794.tx_buf[1] = (uint8_t)(ad7794.mode & 0xFFu);
            ad7794.bytes_to_xfer = 2;
            break;

        case AD7794_REG_CONFIG:
            ad7794.tx_buf[0] = (uint8_t)(ad7794.config >> 8);
            ad7794.tx_buf[1] = (uint8_t)(ad7794.config & 0xFFu);
            ad7794.bytes_to_xfer = 2;
            break;

        case AD7794_REG_DATA:
            ad7794.tx_buf[0] = (uint8_t)(ad7794.data >> 16);
            ad7794.tx_buf[1] = (uint8_t)(ad7794.data >> 8);
            ad7794.tx_buf[2] = (uint8_t)(ad7794.data);
            ad7794.bytes_to_xfer = 3;
            break;

        case AD7794_REG_ID:
            ad7794.tx_buf[0] = ad7794.id;
            ad7794.bytes_to_xfer = 1;
            break;

        case AD7794_REG_IO:
            ad7794.tx_buf[0] = ad7794.io;
            ad7794.bytes_to_xfer = 1;
            break;

        case AD7794_REG_OFFSET:
            ad7794.tx_buf[0] = (uint8_t)(ad7794.offset >> 16);
            ad7794.tx_buf[1] = (uint8_t)(ad7794.offset >> 8);
            ad7794.tx_buf[2] = (uint8_t)(ad7794.offset);
            ad7794.bytes_to_xfer = 3;
            break;

        case AD7794_REG_FULLSCALE:
            ad7794.tx_buf[0] = (uint8_t)(ad7794.fullscale >> 16);
            ad7794.tx_buf[1] = (uint8_t)(ad7794.fullscale >> 8);
            ad7794.tx_buf[2] = (uint8_t)(ad7794.fullscale);
            ad7794.bytes_to_xfer = 3;
            break;

        default:
            ad7794.bytes_to_xfer = 0;
            break;
    }
}

static uint8_t get_write_length(uint8_t reg)
{
    switch (reg)
    {
        case AD7794_REG_MODE:
        case AD7794_REG_CONFIG:
            return 2;

        case AD7794_REG_IO:
            return 1;

        case AD7794_REG_OFFSET:
        case AD7794_REG_FULLSCALE:
            return 3;

        default:
            return 0;
    }
}

/* -------------------------------------------------------------------------- */
/* Communications Register                                                   */
/* -------------------------------------------------------------------------- */

static void process_communications_register(uint8_t comm)
{
    /*
     * 0xFF is deliberately filtered before this function is called.
     * WEN=1 values other than the serial reset pattern are invalid and are
     * ignored, leaving the interface waiting for the next valid command.
     */
    if (comm & AD7794_COMM_WEN)
    {
        ad7794.spi_state = AD7794_SPI_WAIT_COMM;
        ad7794.bytes_to_xfer = 0;
        ad7794.byte_idx = 0;
        SPI_PrepareTx(0xFFu);
        return;
    }

    ad7794.is_read = (comm & AD7794_COMM_READ) ? 1u : 0u;
    ad7794.next_reg = (uint8_t)((comm >> 3) & 0x07u);
    ad7794.cread = (comm & AD7794_COMM_CREAD) ? 1u : 0u;

    /*
     * 0x5C: READ DATA + CREAD.
     * If a conversion is already ready, prepare it immediately. Otherwise
     * remain in CREAD_WAIT until AD7794_Emu_Process() sees a ready conversion.
     */
    if (ad7794.is_read &&
        ad7794.next_reg == AD7794_REG_DATA &&
        ad7794.cread)
    {
        ad7794.spi_state = AD7794_SPI_CREAD_WAIT;
        ad7794.bytes_to_xfer = 3;
        ad7794.byte_idx = 0;

        if (ad7794.data_ready)
        {
            PA6_As_MISO();
            ad7794.spi_state = AD7794_SPI_CREAD_DATA;
            prepare_tx_buffer();
            SPI_PrepareTx(ad7794.tx_buf[0]);
        }
        else
        {
            update_rdy_pin();
        }
        return;
    }

    /*
     * Any valid command with CREAD=0 exits continuous-read mode. In
     * particular 0x58 is the normal READ DATA command and therefore also
     * exits CREAD.
     */
    if (!ad7794.cread)
    {
        /* keep normal command processing below */
    }

    if (ad7794.is_read)
    {
        prepare_tx_buffer();

        if (ad7794.bytes_to_xfer == 0)
        {
            ad7794.spi_state = AD7794_SPI_WAIT_COMM;
            SPI_PrepareTx(0xFFu);
            return;
        }

        LL_GPIO_ResetOutputPin(HLP_GPIO_Port, HLP_Pin);
        PA6_As_MISO();
        ad7794.spi_state = AD7794_SPI_READ;
        SPI_PrepareTx(ad7794.tx_buf[0]);
        return;
    }

    /* WRITE */
    ad7794.bytes_to_xfer = get_write_length(ad7794.next_reg);
    ad7794.byte_idx = 0;

    if (ad7794.bytes_to_xfer != 0)
    {
        ad7794.spi_state = AD7794_SPI_WRITE;
        SPI_PrepareTx(0xFFu);
    }
    else
    {
        ad7794.spi_state = AD7794_SPI_WAIT_COMM;
        SPI_PrepareTx(0xFFu);
    }
}

/* -------------------------------------------------------------------------- */
/* Register write                                                            */
/* -------------------------------------------------------------------------- */

static void process_write_data(uint8_t data)
{
    if (ad7794.byte_idx < sizeof(ad7794.rx_buf))
        ad7794.rx_buf[ad7794.byte_idx] = data;

    ad7794.byte_idx++;

    /* Do not commit a multi-byte register until all bytes are received. */
    if (ad7794.byte_idx < ad7794.bytes_to_xfer)
        return;

    switch (ad7794.next_reg)
    {
        case AD7794_REG_MODE:
            ad7794.mode = ((uint16_t)ad7794.rx_buf[0] << 8) |
                          (uint16_t)ad7794.rx_buf[1];
            ad7794.status |= AD7794_STATUS_RDY;
            ad7794.data_ready = false;
            break;

        case AD7794_REG_CONFIG:
            ad7794.config = ((uint16_t)ad7794.rx_buf[0] << 8) |
                            (uint16_t)ad7794.rx_buf[1];
            ad7794.status |= AD7794_STATUS_RDY;
            ad7794.data_ready = false;
            break;

        case AD7794_REG_IO:
            ad7794.io = ad7794.rx_buf[0];
            break;

        case AD7794_REG_OFFSET:
            ad7794.offset = ((uint32_t)ad7794.rx_buf[0] << 16) |
                            ((uint32_t)ad7794.rx_buf[1] << 8) |
                            (uint32_t)ad7794.rx_buf[2];
            ad7794.offset &= 0xFFFFFFu;
            break;

        case AD7794_REG_FULLSCALE:
            ad7794.fullscale = ((uint32_t)ad7794.rx_buf[0] << 16) |
                               ((uint32_t)ad7794.rx_buf[1] << 8) |
                               (uint32_t)ad7794.rx_buf[2];
            ad7794.fullscale &= 0xFFFFFFu;
            break;

        default:
            break;
    }

    update_rdy_pin();
}

/* -------------------------------------------------------------------------- */
/* READ handling                                                             */
/* -------------------------------------------------------------------------- */

static void finish_read(void)
{
    /*
     * Reading DATA acknowledges the current conversion. STATUS, MODE, ID,
     * etc. are read-only accesses and do not clear RDY here.
     */
    if (ad7794.next_reg == AD7794_REG_DATA)
    {
        ad7794.status |= AD7794_STATUS_RDY;
        ad7794.data_ready = false;
    }

    ad7794.spi_state = AD7794_SPI_WAIT_COMM;
    ad7794.bytes_to_xfer = 0;
    ad7794.byte_idx = 0;

    SPI_PrepareTx(0xFFu);
    PA6_As_RDY();
    update_rdy_pin();
}

static void process_read(void)
{
    ad7794.byte_idx++;

    if (ad7794.byte_idx < ad7794.bytes_to_xfer)
    {
        SPI_PrepareTx(ad7794.tx_buf[ad7794.byte_idx]);
        return;
    }

    finish_read();
}

static void finish_write(void)
{
    ad7794.spi_state = AD7794_SPI_WAIT_COMM;
    ad7794.bytes_to_xfer = 0;
    ad7794.byte_idx = 0;
    SPI_PrepareTx(0xFFu);
    update_rdy_pin();
}

/* -------------------------------------------------------------------------- */
/* CREAD handling                                                            */
/* -------------------------------------------------------------------------- */

static void start_cread_data(void)
{
    if (!ad7794.cread || !ad7794.data_ready)
        return;

    prepare_tx_buffer();
    ad7794.spi_state = AD7794_SPI_CREAD_DATA;
    ad7794.byte_idx = 0;

    PA6_As_MISO();
    SPI_PrepareTx(ad7794.tx_buf[0]);
}

static void process_cread_data(void)
{
    ad7794.byte_idx++;

    if (ad7794.byte_idx < 3u)
    {
        SPI_PrepareTx(ad7794.tx_buf[ad7794.byte_idx]);
        return;
    }

    /* DATA was completely shifted out. */
    ad7794.status |= AD7794_STATUS_RDY;
    ad7794.data_ready = false;
    ad7794.byte_idx = 0;
    ad7794.bytes_to_xfer = 3;
    ad7794.spi_state = AD7794_SPI_CREAD_WAIT;

    PA6_As_RDY();
    update_rdy_pin();
    SPI_PrepareTx(0xFFu);
}

/* -------------------------------------------------------------------------- */
/* Reset                                                                     */
/* -------------------------------------------------------------------------- */

void AD7794_Emu_Reset(void)
{
    ad7794.status = AD7794_DEFAULT_STATUS;
    ad7794.mode = AD7794_DEFAULT_MODE;
    ad7794.config = AD7794_DEFAULT_CONFIG;
    ad7794.data = 0x123456u;
    ad7794.id = AD7794_DEFAULT_ID;
    ad7794.io = AD7794_DEFAULT_IO;
    ad7794.offset = AD7794_DEFAULT_OFFSET;
    ad7794.fullscale = AD7794_DEFAULT_FULLSCALE;

    ad7794.spicnt = 0;
    ad7794.next_reg = 0;
    ad7794.is_read = 0;
    ad7794.cread = 0;
    ad7794.spi_state = AD7794_SPI_WAIT_COMM;
    ad7794.bytes_to_xfer = 0;
    ad7794.byte_idx = 0;
    ad7794.reset_one_bits = 0;

    memset(ad7794.tx_buf, 0, sizeof(ad7794.tx_buf));
    memset(ad7794.rx_buf, 0, sizeof(ad7794.rx_buf));

    ad7794.data_ready = false;
    ad7794.last_conversion_tick = HAL_GetTick();

    PA6_As_RDY();
    update_rdy_pin();
}

void AD7794_Emu_SetData(uint32_t value_24bit)
{
    ad7794.data = value_24bit & 0xFFFFFFu;
}

void AD7794_Emu_Init(void)
{
    memset(&ad7794, 0, sizeof(ad7794));

    ad7794.cs_active = false;
    ad7794.conversion_period_ms = 1u;

    AD7794_Emu_Reset();

    /* Correct 24-bit test value: 0x800000, not 0x08000000. */
    AD7794_Emu_SetData(0x800000u);
}

/* Forward declaration: CREAD data can be armed by the background conversion
 * process before the master clocks the first data byte. */
static void start_cread_data(void);

/* -------------------------------------------------------------------------- */
/* Conversion process                                                        */
/* -------------------------------------------------------------------------- */

void AD7794_Emu_Process(void)
{
    uint32_t now = HAL_GetTick();

    if ((uint32_t)(now - ad7794.last_conversion_tick) <
        ad7794.conversion_period_ms)
        return;

    ad7794.last_conversion_tick = now;

    {
        uint8_t mode = (uint8_t)((ad7794.mode >> 13) & 0x07u);

        /* 010 = idle, 011 = power-down. */
        if (mode == 0x02u || mode == 0x03u)
        {
            ad7794.status |= AD7794_STATUS_RDY;
            ad7794.data_ready = false;
            PA6_As_RDY();
            update_rdy_pin();
            return;
        }
    }

    /* New conversion result. */
    ad7794.status &= (uint8_t)~AD7794_STATUS_RDY;
    ad7794.data_ready = true;

    if (ad7794.cread &&
        ad7794.spi_state == AD7794_SPI_CREAD_WAIT)
    {
        /*
         * In CREAD, RDY goes low and the data is ready for clocking.
         * The TX register must be loaded BEFORE the master starts SCLK.
         */
        start_cread_data();
        return;
    }

    PA6_As_RDY();
    update_rdy_pin();
}

/* -------------------------------------------------------------------------- */
/* CS                                                                        */
/* -------------------------------------------------------------------------- */

void AD7794_Emu_CS_Activate(void)
{
    ad7794.cs_active = true;

    /* Keep CREAD active across CS frames. */
    ad7794.spi_state = ad7794.cread ? AD7794_SPI_CREAD_WAIT
                                    : AD7794_SPI_WAIT_COMM;
    ad7794.bytes_to_xfer = 0;
    ad7794.byte_idx = 0;
    ad7794.reset_one_bits = 0;

    /* CREAD is deliberately NOT cleared here. */

    /* If CREAD is already active and data is ready, expose it immediately. */
    if (ad7794.cread && ad7794.data_ready)
    {
        start_cread_data();
    }
    else
    {
        PA6_As_RDY();
        update_rdy_pin();
        SPI_PrepareTx(0xFFu);
    }
}

void AD7794_Emu_CS_Deactivate(void)
{
    ad7794.cs_active = false;

    /* Abort only the currently active SPI byte transaction. */
    ad7794.spi_state = ad7794.cread ? AD7794_SPI_CREAD_WAIT
                                    : AD7794_SPI_WAIT_COMM;
    ad7794.bytes_to_xfer = 0;
    ad7794.byte_idx = 0;
    ad7794.reset_one_bits = 0;

    PA6_As_RDY();
    update_rdy_pin();
}

/* -------------------------------------------------------------------------- */
/* SPI RX callback                                                           */
/* -------------------------------------------------------------------------- */

void AD7794_Emu_SPI_RxTxCplt(uint8_t data)
{
    if (!ad7794.cs_active)
    {
        ad7794.cs_active = true;
        ad7794.spi_state = AD7794_SPI_WAIT_COMM;
    }

    /*
     * 0xFF is dummy data and, four consecutive times, the AD7794 serial
     * interface reset sequence (32 consecutive '1' bits).
     *
     * IMPORTANT: do not pass FF to process_communications_register().
     */
    if (data == 0xFFu)
    {
        if (ad7794.reset_one_bits < 4u)
            ad7794.reset_one_bits++;

        if (ad7794.reset_one_bits >= 4u)
        {
            bool cs = ad7794.cs_active;
            uint32_t period = ad7794.conversion_period_ms;

            AD7794_Emu_Reset();

            ad7794.cs_active = cs;
            ad7794.conversion_period_ms = period;
        }
        SPI_PrepareTx(0xFFu);
        return;
    }

    /* Any non-FF byte breaks the 32-one reset sequence. */
    ad7794.reset_one_bits = 0;

    switch (ad7794.spi_state)
    {
        case AD7794_SPI_WAIT_COMM:
            process_communications_register(data);
            break;

        case AD7794_SPI_READ:
            process_read();
            break;

        case AD7794_SPI_WRITE:
            process_write_data(data);
            if (ad7794.byte_idx >= ad7794.bytes_to_xfer)
                finish_write();
            break;

        case AD7794_SPI_CREAD_WAIT:
            /*
             * Normally the master waits for RDY before clocking data.
             * If it clocks anyway, leave the line at FF until data is ready.
             */
            SPI_PrepareTx(0xFFu);
            break;

        case AD7794_SPI_CREAD_DATA:
            process_cread_data();
            break;

        default:
            ad7794.spi_state = AD7794_SPI_WAIT_COMM;
            SPI_PrepareTx(0xFFu);
            break;
    }

    ad7794.spicnt++;
}

/* -------------------------------------------------------------------------- */
/* SPI error                                                                 */
/* -------------------------------------------------------------------------- */

void AD7794_Emu_SPI_Error(void)
{
    ad7794.spi_state = ad7794.cread ? AD7794_SPI_CREAD_WAIT
                                    : AD7794_SPI_WAIT_COMM;
    ad7794.bytes_to_xfer = 0;
    ad7794.byte_idx = 0;
    ad7794.reset_one_bits = 0;

    PA6_As_RDY();
    update_rdy_pin();

    SPI_ClearPending();
    SPI_PrepareTx(0xFFu);
}
