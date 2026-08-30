#include "ad7794_emu.h"

#include <string.h>

/* ============================================================
 * Global instance
 * ============================================================ */

AD7794_Emu_t ad7794;

/* ============================================================
 * Local helpers
 * ============================================================ */

static void PA6_As_RDY(void)
{
    /*
     * PA6 = DOUT/RDY
     *
     * HIGH = not ready
     * LOW  = data ready
     */
    LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);

    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6);
}

static void PA6_As_MISO(void)
{
    /*
     * PA6 is also SPI1_MISO.
     */
    LL_GPIO_SetPinMode(GPIOA,
                       LL_GPIO_PIN_6,
                       LL_GPIO_MODE_ALTERNATE);
}

static void PA6_RDY_Low(void)
{
    LL_GPIO_SetPinMode(GPIOA,
                       LL_GPIO_PIN_6,
                       LL_GPIO_MODE_OUTPUT);

    LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_6);
}

static void PA6_RDY_High(void)
{
    LL_GPIO_SetPinMode(GPIOA,
                       LL_GPIO_PIN_6,
                       LL_GPIO_MODE_OUTPUT);

    LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_6);
}

/* ============================================================
 * SPI helpers
 * ============================================================ */

/*
 * Clear possible stale RX data / overrun.
 *
 * On STM32F0 OVR is cleared by reading DR and then SR.
 */
static void SPI_ClearPending(void)
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
 * Prepare next byte for MISO.
 *
 * IMPORTANT:
 * The byte must be written to SPI data register BEFORE
 * the master starts generating the next SCLK edges.
 */
static void SPI_PrepareTx(uint8_t data)
{
    if (LL_SPI_IsActiveFlag_TXE(SPI1))
    {
        LL_SPI_TransmitData8(SPI1, data);
    }
}

/*
 * Start waiting for the next received byte.
 */
static void SPI_EnableReceive(void)
{
    LL_SPI_EnableIT_RXNE(SPI1);
}

/* ============================================================
 * Register -> TX buffer
 * ============================================================ */

static void prepare_tx_buffer(void)
{
    memset(ad7794.tx_buf, 0xFF, sizeof(ad7794.tx_buf));

    ad7794.byte_idx = 0;

    switch (ad7794.next_reg)
    {
        case AD7794_REG_STATUS:

            ad7794.tx_buf[0] = ad7794.status;
            ad7794.bytes_to_xfer = 1;

            break;


        case AD7794_REG_MODE:

            ad7794.tx_buf[0] =
                (uint8_t)((ad7794.mode >> 8) & 0xFF);

            ad7794.tx_buf[1] =
                (uint8_t)(ad7794.mode & 0xFF);

            ad7794.bytes_to_xfer = 2;

            break;


        case AD7794_REG_CONFIG:

            ad7794.tx_buf[0] =
                (uint8_t)((ad7794.config >> 8) & 0xFF);

            ad7794.tx_buf[1] =
                (uint8_t)(ad7794.config & 0xFF);

            ad7794.bytes_to_xfer = 2;

            break;


        case AD7794_REG_DATA:

            ad7794.tx_buf[0] =
                (uint8_t)((ad7794.data >> 16) & 0xFF);

            ad7794.tx_buf[1] =
                (uint8_t)((ad7794.data >> 8) & 0xFF);

            ad7794.tx_buf[2] =
                (uint8_t)(ad7794.data & 0xFF);

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

            ad7794.tx_buf[0] =
                (uint8_t)((ad7794.offset >> 16) & 0xFF);

            ad7794.tx_buf[1] =
                (uint8_t)((ad7794.offset >> 8) & 0xFF);

            ad7794.tx_buf[2] =
                (uint8_t)(ad7794.offset & 0xFF);

            ad7794.bytes_to_xfer = 3;

            break;


        case AD7794_REG_FULLSCALE:

            ad7794.tx_buf[0] =
                (uint8_t)((ad7794.fullscale >> 16) & 0xFF);

            ad7794.tx_buf[1] =
                (uint8_t)((ad7794.fullscale >> 8) & 0xFF);

            ad7794.tx_buf[2] =
                (uint8_t)(ad7794.fullscale & 0xFF);

            ad7794.bytes_to_xfer = 3;

            break;


        default:

            ad7794.bytes_to_xfer = 0;

            break;
    }
}

/* ============================================================
 * Register write length
 * ============================================================ */

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

        /*
         * DATA, STATUS and ID are not writable.
         */
        default:
            return 0;
    }
}

/* ============================================================
 * Update RDY according to status
 * ============================================================ */

static void update_rdy_pin(void)
{
    if (!ad7794.cs_active)
    {
        PA6_RDY_High();
        return;
    }

    if (ad7794.status & AD7794_STATUS_RDY)
    {
        PA6_RDY_High();
    }
    else
    {
        PA6_RDY_Low();
    }
}

/* ============================================================
 * Process Communications Register
 * ============================================================ */

static void process_communications_register(uint8_t comm)
{
    /*
     * AD7794:
     *
     * bit 7 = WEN
     * bit 6 = R/W
     * bit 5:3 = RS
     * bit 2 = CREAD
     * bit 1:0 = 0
     */

    /*
     * Any non-FF byte terminates the consecutive-1 reset sequence.
     */
    ad7794.reset_one_bits = 0;

    /*
     * WEN must be 0.
     */
    if (comm & AD7794_COMM_WEN)
    {
        ad7794.spi_state = AD7794_SPI_WAIT_COMM;
        ad7794.bytes_to_xfer = 0;
        return;
    }

    ad7794.is_read =
        (comm & AD7794_COMM_READ) ? 1u : 0u;

    ad7794.next_reg =
        (uint8_t)((comm >> 3) & 0x07u);

    ad7794.cread =
        (comm & AD7794_COMM_CREAD) ? 1u : 0u;


    /*
     * Continuous read mode:
     *
     * 0x5C = read DATA + CREAD
     */
    if (ad7794.is_read &&
        ad7794.next_reg == AD7794_REG_DATA &&
        ad7794.cread)
    {
        ad7794.spi_state = AD7794_SPI_CREAD_WAIT;

        ad7794.bytes_to_xfer = 3;
        ad7794.byte_idx = 0;

        /*
         * Wait until conversion is ready.
         */
        PA6_RDY_High();

        return;
    }


    /*
     * Normal READ
     */
    if (ad7794.is_read)
    {
        prepare_tx_buffer();

        if (ad7794.bytes_to_xfer > 0)
        {
            ad7794.spi_state = AD7794_SPI_READ;

            PA6_As_MISO();

            /*
             * First byte has to be loaded BEFORE the next SCLK.
             */
            SPI_PrepareTx(ad7794.tx_buf[0]);

            SPI_EnableReceive();
        }

        return;
    }


    /*
     * WRITE
     */
    ad7794.bytes_to_xfer =
        get_write_length(ad7794.next_reg);

    ad7794.byte_idx = 0;

    memset(ad7794.rx_buf,
           0,
           sizeof(ad7794.rx_buf));

    if (ad7794.bytes_to_xfer > 0)
    {
        ad7794.spi_state = AD7794_SPI_WRITE;

        /*
         * DOUT/RDY remains RDY during write.
         */
        update_rdy_pin();

        /*
         * MISO byte is irrelevant during write,
         * but we must provide a byte for SPI.
         */
        SPI_PrepareTx(0x00);

        SPI_EnableReceive();
    }
    else
    {
        ad7794.spi_state = AD7794_SPI_WAIT_COMM;

        PA6_As_RDY();

        SPI_PrepareTx(0x00);

        SPI_EnableReceive();
    }
}

/* ============================================================
 * Process completed register write
 * ============================================================ */

static void process_write_data(void)
{
    switch (ad7794.next_reg)
    {
        case AD7794_REG_MODE:

            ad7794.mode =
                ((uint16_t)ad7794.rx_buf[0] << 8) |
                ((uint16_t)ad7794.rx_buf[1]);

            /*
             * Any write to MODE resets the modulator/filter
             * and sets RDY.
             */
            ad7794.status |= AD7794_STATUS_RDY;

            ad7794.data_ready = false;

            update_rdy_pin();

            break;


        case AD7794_REG_CONFIG:

            ad7794.config =
                ((uint16_t)ad7794.rx_buf[0] << 8) |
                ((uint16_t)ad7794.rx_buf[1]);

            /*
             * Configuration change also invalidates current
             * conversion state.
             */
            ad7794.status |= AD7794_STATUS_RDY;

            ad7794.data_ready = false;

            update_rdy_pin();

            break;


        case AD7794_REG_IO:

            ad7794.io =
                ad7794.rx_buf[0];

            break;


        case AD7794_REG_OFFSET:

            ad7794.offset =
                ((uint32_t)ad7794.rx_buf[0] << 16) |
                ((uint32_t)ad7794.rx_buf[1] << 8) |
                ((uint32_t)ad7794.rx_buf[2]);

            ad7794.offset &= 0xFFFFFFu;

            break;


        case AD7794_REG_FULLSCALE:

            ad7794.fullscale =
                ((uint32_t)ad7794.rx_buf[0] << 16) |
                ((uint32_t)ad7794.rx_buf[1] << 8) |
                ((uint32_t)ad7794.rx_buf[2]);

            ad7794.fullscale &= 0xFFFFFFu;

            break;


        default:

            break;
    }
}

/* ============================================================
 * Finish normal READ
 * ============================================================ */

static void finish_read(void)
{
    /*
     * DATA read sets RDY.
     */
    if (ad7794.next_reg == AD7794_REG_DATA)
    {
        ad7794.status |= AD7794_STATUS_RDY;
        ad7794.data_ready = false;

        update_rdy_pin();
    }

    ad7794.spi_state = AD7794_SPI_WAIT_COMM;

    ad7794.bytes_to_xfer = 0;
    ad7794.byte_idx = 0;

    /*
     * MISO/RDY returns to RDY function.
     */
    PA6_As_RDY();

    /*
     * Next byte from master is a Communications Register.
     */
    SPI_PrepareTx(0x00);
}

/* ============================================================
 * Finish WRITE
 * ============================================================ */

static void finish_write(void)
{
    process_write_data();

    ad7794.spi_state = AD7794_SPI_WAIT_COMM;

    ad7794.bytes_to_xfer = 0;
    ad7794.byte_idx = 0;

    PA6_As_RDY();

    SPI_PrepareTx(0x00);
}

/* ============================================================
 * Finish Continuous Read
 * ============================================================ */

static void finish_cread_data(void)
{
    /*
     * After DATA has been read RDY goes high.
     */
    ad7794.status |= AD7794_STATUS_RDY;

    ad7794.data_ready = false;

    update_rdy_pin();

    /*
     * Stay in CREAD mode.
     *
     * Next conversion will bring RDY low and the next
     * 24-bit word will be made available automatically.
     */
    ad7794.spi_state = AD7794_SPI_CREAD_WAIT;

    ad7794.byte_idx = 0;
    ad7794.bytes_to_xfer = 3;
}

/* ============================================================
 * Start continuous read data transfer
 * ============================================================ */

static void start_cread_data(void)
{
    if (!ad7794.cs_active)
        return;

    if (!ad7794.cread)
        return;

    if (!ad7794.data_ready)
        return;

    prepare_tx_buffer();

    ad7794.spi_state = AD7794_SPI_CREAD_DATA;

    PA6_As_MISO();

    /*
     * RDY is already low.
     */
    SPI_PrepareTx(ad7794.tx_buf[0]);

    SPI_EnableReceive();
}

/* ============================================================
 * RESET
 * ============================================================ */

void AD7794_Emu_Reset(void)
{
    ad7794.status =
        AD7794_DEFAULT_STATUS;

    ad7794.mode =
        AD7794_DEFAULT_MODE;

    ad7794.config =
        AD7794_DEFAULT_CONFIG;

    ad7794.data =
        0x123456u;

    ad7794.id =
        AD7794_DEFAULT_ID;

    ad7794.io =
        AD7794_DEFAULT_IO;

    ad7794.offset =
        AD7794_DEFAULT_OFFSET;

    ad7794.fullscale =
        AD7794_DEFAULT_FULLSCALE;

    ad7794.next_reg = 0;
    ad7794.is_read = 0;
    ad7794.cread = 0;

    ad7794.spi_state =
        AD7794_SPI_WAIT_COMM;

    ad7794.bytes_to_xfer = 0;
    ad7794.byte_idx = 0;

    ad7794.reset_one_bits = 0;

    memset(ad7794.tx_buf,
           0,
           sizeof(ad7794.tx_buf));

    memset(ad7794.rx_buf,
           0,
           sizeof(ad7794.rx_buf));

    ad7794.data_ready = false;

    if (ad7794.cs_active)
    {
        PA6_As_RDY();
        update_rdy_pin();
    }
}

/* ============================================================
 * Set ADC conversion result
 * ============================================================ */

void AD7794_Emu_SetData(uint32_t value_24bit)
{
    ad7794.data =
        value_24bit & 0xFFFFFFu;
}

/* ============================================================
 * Init
 * ============================================================ */

void AD7794_Emu_Init(void)
{
    memset(&ad7794,
           0,
           sizeof(ad7794));

    ad7794.cs_active = false;

    AD7794_Emu_Reset();

    /*
     * Test value.
     */
    AD7794_Emu_SetData(0x123456u);

    /*
     * Emulator conversion period.
     *
     * This is only a model of the ADC.
     */
    ad7794.conversion_period_ms = 50u;

    ad7794.last_conversion_tick =
        HAL_GetTick();

    PA6_RDY_High();
}

/* ============================================================
 * ADC conversion process
 * ============================================================ */

void AD7794_Emu_Process(void)
{
    uint32_t now;

    if (!ad7794.cs_active)
        return;

    /*
     * Emulator conversion clock.
     */
    now = HAL_GetTick();

    if ((uint32_t)(now - ad7794.last_conversion_tick) <
        ad7794.conversion_period_ms)
    {
        return;
    }

    ad7794.last_conversion_tick = now;

    /*
     * Check MODE.
     *
     * MD2:MD0
     *
     * 000 = continuous
     * 001 = single
     * 010 = idle
     * 011 = power-down
     * ...
     */
    uint8_t mode =
        (uint8_t)((ad7794.mode >> 13) & 0x07u);

    if (mode == 0x02u ||   /* idle */
        mode == 0x03u)     /* power-down */
    {
        ad7794.status |= AD7794_STATUS_RDY;

        ad7794.data_ready = false;

        update_rdy_pin();

        return;
    }

    /*
     * New conversion available.
     */
    ad7794.status &=
        (uint8_t)~AD7794_STATUS_RDY;

    ad7794.data_ready = true;

    /*
     * In continuous read mode, automatically prepare
     * the DATA register for the master.
     */
    if (ad7794.spi_state ==
            AD7794_SPI_CREAD_WAIT &&
        ad7794.cread)
    {
        start_cread_data();

        /*
         * start_cread_data() changes MISO to SPI mode.
         */
        return;
    }

    update_rdy_pin();
}

/* ============================================================
 * CS LOW
 * ============================================================ */

void AD7794_Emu_CS_Activate(void)
{
    ad7794.cs_active = true;

    ad7794.spi_state =
        AD7794_SPI_WAIT_COMM;

    ad7794.bytes_to_xfer = 0;
    ad7794.byte_idx = 0;

    ad7794.reset_one_bits = 0;

    ad7794.cread = 0;

    /*
     * Clear anything left from previous transaction.
     */
    SPI_ClearPending();

    /*
     * DOUT/RDY is HIGH while waiting for conversion.
     */
    PA6_As_RDY();

    /*
     * We need a valid byte in TX register BEFORE
     * the master clocks the Communications byte.
     */
    SPI_PrepareTx(0x00);

    SPI_EnableReceive();
}

/* ============================================================
 * CS HIGH
 * ============================================================ */

void AD7794_Emu_CS_Deactivate(void)
{
    ad7794.cs_active = false;

    ad7794.spi_state =
        AD7794_SPI_WAIT_COMM;

    ad7794.bytes_to_xfer = 0;
    ad7794.byte_idx = 0;

    ad7794.cread = 0;

    ad7794.data_ready = false;

    /*
     * No RX interrupt while CS is inactive.
     */
    LL_SPI_DisableIT_RXNE(SPI1);

    /*
     * Clear possible byte left in RX register.
     */
    SPI_ClearPending();

    /*
     * DOUT/RDY must be inactive/high.
     */
    PA6_RDY_High();
}

/* ============================================================
 * SPI RX callback
 * ============================================================ */

void AD7794_Emu_SPI_RxTxCplt(uint8_t data)
{
    /*
     * If CS is no longer active, ignore data.
     */
    if (!ad7794.cs_active)
    {
        return;
    }


    /* ========================================================
     * WAITING FOR COMMUNICATIONS REGISTER
     * ======================================================== */

    if (ad7794.spi_state ==
            AD7794_SPI_WAIT_COMM)
    {
        /*
         * AD7794 reset:
         *
         * >=32 consecutive SCLK cycles with DIN=1.
         *
         * Four bytes 0xFF are enough.
         */
        if (data == 0xFFu)
        {
            ad7794.reset_one_bits += 8u;

            if (ad7794.reset_one_bits >=
                AD7794_RESET_BITS)
            {
                AD7794_Emu_Reset();

                /*
                 * Keep CS state.
                 */
                ad7794.cs_active = true;

                /*
                 * Return to Communications Register state.
                 */
                ad7794.spi_state =
                    AD7794_SPI_WAIT_COMM;

                PA6_As_RDY();

                SPI_PrepareTx(0x00);

                SPI_EnableReceive();

                return;
            }

            SPI_PrepareTx(0x00);

            SPI_EnableReceive();

            return;
        }


        /*
         * Any other byte breaks the reset sequence.
         */
        ad7794.reset_one_bits = 0;

        process_communications_register(data);

        /*
         * Continuous read does not immediately output data.
         */
        if (ad7794.spi_state ==
                AD7794_SPI_CREAD_WAIT)
        {
            /*
             * Wait until RDY goes low.
             */
            PA6_As_RDY();

            /*
             * Keep RX interrupt active.
             *
             * This is important because the AD7794
             * continuously monitors DIN while in CREAD.
             */
            SPI_PrepareTx(0x00);

            SPI_EnableReceive();

            return;
        }

        /*
         * Normal READ/WRITE have already prepared the
         * first TX byte.
         */
        if (ad7794.spi_state ==
                AD7794_SPI_READ ||
            ad7794.spi_state ==
                AD7794_SPI_WRITE)
        {
            return;
        }

        /*
         * Invalid command.
         */
        SPI_PrepareTx(0x00);

        SPI_EnableReceive();

        return;
    }


    /* ========================================================
     * NORMAL READ
     * ======================================================== */

    if (ad7794.spi_state ==
            AD7794_SPI_READ)
    {
        /*
         * We have just received one byte.
         */
        ad7794.byte_idx++;

        if (ad7794.byte_idx <
            ad7794.bytes_to_xfer)
        {
            /*
             * Prepare NEXT byte immediately.
             */
            SPI_PrepareTx(
                ad7794.tx_buf[
                    ad7794.byte_idx]);

            SPI_EnableReceive();

            return;
        }

        /*
         * Entire register read completed.
         */
        finish_read();

        SPI_EnableReceive();

        return;
    }


    /* ========================================================
     * NORMAL WRITE
     * ======================================================== */

    if (ad7794.spi_state ==
            AD7794_SPI_WRITE)
    {
        /*
         * Store received byte.
         */
        if (ad7794.byte_idx <
            sizeof(ad7794.rx_buf))
        {
            ad7794.rx_buf[
                ad7794.byte_idx] = data;
        }

        ad7794.byte_idx++;

        if (ad7794.byte_idx <
            ad7794.bytes_to_xfer)
        {
            /*
             * Dummy byte during write.
             */
            SPI_PrepareTx(0x00);

            SPI_EnableReceive();

            return;
        }

        /*
         * Write complete.
         */
        finish_write();

        SPI_EnableReceive();

        return;
    }


    /* ========================================================
     * CONTINUOUS READ WAIT
     * ======================================================== */

    if (ad7794.spi_state ==
            AD7794_SPI_CREAD_WAIT)
    {
        /*
         * AD7794 monitors DIN while in CREAD.
         *
         * 0x58 is the command used to exit CREAD.
         *
         * We accept it here when the master sends it
         * while RDY is low.
         */
        if (data == AD7794_COMM_EXIT_CREAD)
        {
            ad7794.cread = 0;

            ad7794.spi_state =
                AD7794_SPI_WAIT_COMM;

            ad7794.bytes_to_xfer = 0;
            ad7794.byte_idx = 0;

            PA6_As_RDY();

            SPI_PrepareTx(0x00);

            SPI_EnableReceive();

            return;
        }

        /*
         * If conversion is ready, this byte is part of
         * the 24-bit data transfer.
         */
        if (ad7794.data_ready)
        {
            /*
             * First data byte was already loaded by
             * start_cread_data().
             */
            ad7794.spi_state =
                AD7794_SPI_CREAD_DATA;

            ad7794.byte_idx = 1;

            /*
             * Prepare second byte.
             */
            if (ad7794.byte_idx <
                ad7794.bytes_to_xfer)
            {
                SPI_PrepareTx(
                    ad7794.tx_buf[
                        ad7794.byte_idx]);

                SPI_EnableReceive();

                return;
            }
        }

        /*
         * No conversion ready.
         */
        SPI_PrepareTx(0x00);

        SPI_EnableReceive();

        return;
    }


    /* ========================================================
     * CONTINUOUS READ DATA
     * ======================================================== */

    if (ad7794.spi_state ==
            AD7794_SPI_CREAD_DATA)
    {
        ad7794.byte_idx++;

        if (ad7794.byte_idx <
            ad7794.bytes_to_xfer)
        {
            SPI_PrepareTx(
                ad7794.tx_buf[
                    ad7794.byte_idx]);

            SPI_EnableReceive();

            return;
        }

        /*
         * 24-bit DATA completely read.
         */
        finish_cread_data();

        /*
         * Keep RX interrupt active so that the master can
         * later send 0x58 to exit CREAD.
         */
        PA6_As_RDY();

        SPI_PrepareTx(0x00);

        SPI_EnableReceive();

        return;
    }


    /*
     * Fallback.
     */
    ad7794.spi_state =
        AD7794_SPI_WAIT_COMM;

    ad7794.bytes_to_xfer = 0;
    ad7794.byte_idx = 0;

    PA6_As_RDY();

    SPI_PrepareTx(0x00);

    SPI_EnableReceive();
}

/* ============================================================
 * SPI error
 * ============================================================ */

void AD7794_Emu_SPI_Error(void)
{
    ad7794.spi_state =
        AD7794_SPI_WAIT_COMM;

    ad7794.bytes_to_xfer = 0;
    ad7794.byte_idx = 0;

    ad7794.reset_one_bits = 0;

    PA6_As_RDY();

    SPI_ClearPending();

    SPI_PrepareTx(0x00);

    SPI_EnableReceive();
}
