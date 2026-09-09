#include "ad7794_emu.h"
#include "CircularBuffer.h"

#include <string.h>

/* ============================================================
 * Global instance
 * ============================================================ */

AD7794_Emu_t ad7794;


/* ============================================================
 * PA6 = DOUT/RDY
 *
 * AD7794:
 *
 *   RDY = HIGH -> conversion not ready
 *   RDY = LOW  -> conversion ready
 *
 * During SPI read PA6 becomes SPI1_MISO.
 * ============================================================ */

static void PA6_As_RDY(void)
{
    LL_GPIO_SetPinMode(GPIOA,
                       LL_GPIO_PIN_6,
                       LL_GPIO_MODE_OUTPUT);

    LL_GPIO_SetOutputPin(GPIOA,
                         LL_GPIO_PIN_6);
}


static void PA6_RDY_Low(void)
{
    LL_GPIO_SetPinMode(GPIOA,
                       LL_GPIO_PIN_6,
                       LL_GPIO_MODE_OUTPUT);

    LL_GPIO_ResetOutputPin(GPIOA,
                           LL_GPIO_PIN_6);
}


static void PA6_RDY_High(void)
{
    LL_GPIO_SetPinMode(GPIOA,
                       LL_GPIO_PIN_6,
                       LL_GPIO_MODE_OUTPUT);

    LL_GPIO_SetOutputPin(GPIOA,
                         LL_GPIO_PIN_6);
}


static void PA6_As_MISO(void)
{
    LL_GPIO_SetPinMode(GPIOA,
                       LL_GPIO_PIN_6,
                       LL_GPIO_MODE_ALTERNATE);
}


/* ============================================================
 * SPI helpers
 * ============================================================ */

/*
 * Clear stale RX data and OVR.
 *
 * STM32F0:
 * OVR is cleared by reading DR and then SR.
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
 * Prepare the next MISO byte.
 *
 * IMPORTANT:
 *
 * This function deliberately does NOT wait for TXE.
 * Waiting inside SPI IRQ can create timing problems.
 *
 * The caller is responsible for preparing TX when TXE
 * is available.
 */
static void SPI_PrepareTx(uint8_t data)
{
    if (LL_SPI_IsActiveFlag_TXE(SPI1))
    {
        LL_SPI_TransmitData8(SPI1, data);
    }
}


/* ============================================================
 * Register -> TX buffer
 * ============================================================ */

static void prepare_tx_buffer(void)
{
    memset(ad7794.tx_buf,
           0xFF,
           sizeof(ad7794.tx_buf));

    ad7794.byte_idx = 0;

    switch (ad7794.next_reg)
    {
        case AD7794_REG_STATUS:

            ad7794.tx_buf[0] =
                ad7794.status;

            ad7794.bytes_to_xfer = 1;

            break;


        case AD7794_REG_MODE:

            ad7794.tx_buf[0] =
                (uint8_t)((ad7794.mode >> 8) & 0xFFu);

            ad7794.tx_buf[1] =
                (uint8_t)(ad7794.mode & 0xFFu);

            ad7794.bytes_to_xfer = 2;

            break;


        case AD7794_REG_CONFIG:

            ad7794.tx_buf[0] =
                (uint8_t)((ad7794.config >> 8) & 0xFFu);

            ad7794.tx_buf[1] =
                (uint8_t)(ad7794.config & 0xFFu);

            ad7794.bytes_to_xfer = 2;

            break;


        case AD7794_REG_DATA:

            ad7794.tx_buf[0] =
                (uint8_t)((ad7794.data >> 16) & 0xFFu);

            ad7794.tx_buf[1] =
                (uint8_t)((ad7794.data >> 8) & 0xFFu);

            ad7794.tx_buf[2] =
                (uint8_t)(ad7794.data & 0xFFu);

            ad7794.bytes_to_xfer = 3;

            break;


        case AD7794_REG_ID:

            ad7794.tx_buf[0] =
                ad7794.id;

            ad7794.bytes_to_xfer = 1;

            break;


        case AD7794_REG_IO:

            ad7794.tx_buf[0] =
                ad7794.io;

            ad7794.bytes_to_xfer = 1;

            break;


        case AD7794_REG_OFFSET:

            ad7794.tx_buf[0] =
                (uint8_t)((ad7794.offset >> 16) & 0xFFu);

            ad7794.tx_buf[1] =
                (uint8_t)((ad7794.offset >> 8) & 0xFFu);

            ad7794.tx_buf[2] =
                (uint8_t)(ad7794.offset & 0xFFu);

            ad7794.bytes_to_xfer = 3;

            break;


        case AD7794_REG_FULLSCALE:

            ad7794.tx_buf[0] =
                (uint8_t)((ad7794.fullscale >> 16) & 0xFFu);

            ad7794.tx_buf[1] =
                (uint8_t)((ad7794.fullscale >> 8) & 0xFFu);

            ad7794.tx_buf[2] =
                (uint8_t)(ad7794.fullscale & 0xFFu);

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
         * DATA, STATUS and ID are read-only.
         */
        default:
            return 0;
    }
}


/* ============================================================
 * RDY
 * ============================================================ */

static void update_rdy_pin(void)
{
    /*
     * When CS is inactive, DOUT/RDY is inactive HIGH.
     */
    if (!ad7794.cs_active)
    {
        PA6_RDY_High();
        return;
    }

    /*
     * During MISO operation do not force PA6 GPIO mode.
     */
    if (ad7794.spi_state == AD7794_SPI_READ ||
        ad7794.spi_state == AD7794_SPI_CREAD_DATA)
    {
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
 * Communications Register
 * ============================================================ */

static void process_communications_register(uint8_t comm)
{
    /*
     * Communications register:
     *
     * bit 7   WEN
     * bit 6   R/W
     * bit 5:3 RS2:RS0
     * bit 2   CREAD
     * bit 1:0 0
     */

    ad7794.reset_one_bits = 0;


    /*
     * WEN must be zero for a valid communication command.
     */
    if (comm & AD7794_COMM_WEN)
    {
        ad7794.spi_state = AD7794_SPI_WAIT_COMM;
        ad7794.bytes_to_xfer = 0;
        ad7794.byte_idx = 0;

        PA6_As_RDY();
        update_rdy_pin();

        SPI_PrepareTx(0xFF);

        return;
    }


    ad7794.is_read =
        (comm & AD7794_COMM_READ) ? 1u : 0u;


    ad7794.next_reg =
        (uint8_t)((comm >> 3) & 0x07u);


    ad7794.cread =
        (comm & AD7794_COMM_CREAD) ? 1u : 0u;


    /*
     * Continuous read DATA.
     *
     * 0x5C
     */
    if (ad7794.is_read &&
        ad7794.next_reg == AD7794_REG_DATA &&
        ad7794.cread)
    {
        ad7794.spi_state =
            AD7794_SPI_CREAD_WAIT;

        ad7794.bytes_to_xfer = 3;
        ad7794.byte_idx = 0;

        /*
         * Data is not yet ready.
         */
        PA6_RDY_High();

        /*
         * No data is put on MISO yet.
         *
         * The master must wait for RDY LOW.
         */
        SPI_PrepareTx(0xFF);
        cb_push('A');cb_push(comm);

        return;
    }


    /*
     * Normal READ.
     */
    if (ad7794.is_read)
    {
        prepare_tx_buffer();

        if (ad7794.bytes_to_xfer != 0)
        {
            ad7794.spi_state =
                AD7794_SPI_READ;

            /*
             * Switch PA6 from RDY to MISO
             * BEFORE the next SCLK.
             */
            PA6_As_MISO();

            /*
             * First response byte must already be
             * present before the master clocks it.
             */
            SPI_PrepareTx(ad7794.tx_buf[0]);
        }
        else
        {
            ad7794.spi_state =
                AD7794_SPI_WAIT_COMM;

            PA6_As_RDY();

            SPI_PrepareTx(0xFF);
        }

        return;
    }


    /*
     * Normal WRITE.
     */
    ad7794.bytes_to_xfer =
        get_write_length(ad7794.next_reg);

    ad7794.byte_idx = 0;

    memset(ad7794.rx_buf,
           0,
           sizeof(ad7794.rx_buf));


    if (ad7794.bytes_to_xfer != 0)
    {
        ad7794.spi_state =
            AD7794_SPI_WRITE;

        /*
         * During write DOUT/RDY remains RDY.
         */
        PA6_As_RDY();
        update_rdy_pin();

        /*
         * Dummy response.
         */
        SPI_PrepareTx(0xFF);
    }
    else
    {
        ad7794.spi_state =
            AD7794_SPI_WAIT_COMM;

        PA6_As_RDY();

        SPI_PrepareTx(0xFF);
    }
}


/* ============================================================
 * Process completed WRITE
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
             * Writing MODE restarts conversion.
             */
            ad7794.status |= AD7794_STATUS_RDY;
            ad7794.data_ready = false;

            ad7794.last_conversion_tick =
                HAL_GetTick();

            PA6_As_RDY();
            update_rdy_pin();

            break;


        case AD7794_REG_CONFIG:

            ad7794.config =
                ((uint16_t)ad7794.rx_buf[0] << 8) |
                ((uint16_t)ad7794.rx_buf[1]);

            /*
             * Configuration change invalidates the
             * current conversion.
             */
            ad7794.status |= AD7794_STATUS_RDY;
            ad7794.data_ready = false;

            ad7794.last_conversion_tick =
                HAL_GetTick();

            PA6_As_RDY();
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
     * Reading DATA clears RDY condition.
     */
    if (ad7794.next_reg == AD7794_REG_DATA)
    {
        ad7794.status |= AD7794_STATUS_RDY;
        ad7794.data_ready = false;
    }


    ad7794.spi_state =
        AD7794_SPI_WAIT_COMM;

    ad7794.bytes_to_xfer = 0;
    ad7794.byte_idx = 0;


    /*
     * Return PA6 to RDY.
     */
    PA6_As_RDY();
    update_rdy_pin();


    /*
     * Prepare dummy response for the next
     * communication byte.
     */
    SPI_PrepareTx(0xFF);
}


/* ============================================================
 * Finish WRITE
 * ============================================================ */

static void finish_write(void)
{
    process_write_data();


    ad7794.spi_state =
        AD7794_SPI_WAIT_COMM;

    ad7794.bytes_to_xfer = 0;
    ad7794.byte_idx = 0;


    PA6_As_RDY();
    update_rdy_pin();


    SPI_PrepareTx(0xFF);
}


/* ============================================================
 * Finish CREAD DATA
 * ============================================================ */

static void finish_cread_data(void)
{
    /*
     * After DATA read, RDY goes HIGH.
     */
    ad7794.status |= AD7794_STATUS_RDY;

    ad7794.data_ready = false;


    /*
     * Stay in CREAD mode.
     */
    ad7794.spi_state =
        AD7794_SPI_CREAD_WAIT;

    ad7794.byte_idx = 0;
    ad7794.bytes_to_xfer = 3;


    PA6_As_RDY();
    update_rdy_pin();


    /*
     * The next conversion will make RDY LOW again.
     */
    SPI_PrepareTx(0xFF);
}


/* ============================================================
 * Start CREAD DATA
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


    ad7794.spi_state =
        AD7794_SPI_CREAD_DATA;

    ad7794.byte_idx = 0;
    ad7794.bytes_to_xfer = 3;


    /*
     * RDY is LOW at this point.
     *
     * Switch to SPI MISO only after the master
     * has detected RDY.
     */
    PA6_As_MISO();


    /*
     * First DATA byte must already be loaded.
     */
    SPI_PrepareTx(ad7794.tx_buf[0]);
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
 * Set conversion result
 * ============================================================ */

void AD7794_Emu_SetData(uint32_t value_24bit)
{
    ad7794.data =
        value_24bit & 0xFFFFFFu;
}


/* ============================================================
 * INIT
 * ============================================================ */

void AD7794_Emu_Init(void)
{
    memset(&ad7794,
           0,
           sizeof(ad7794));


    ad7794.cs_active = false;


    AD7794_Emu_Reset();


    /*
     * Initial test conversion value.
     */
    AD7794_Emu_SetData(0x123456u);


    /*
     * Emulator conversion period.
     *
     * This is intentionally configurable.
     */
    ad7794.conversion_period_ms =
        50u;


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
    uint8_t mode;


    if (!ad7794.cs_active)
        return;


    now = HAL_GetTick();


    if ((uint32_t)(now -
                   ad7794.last_conversion_tick) <
        ad7794.conversion_period_ms)
    {
        return;
    }


    /*
     * Keep conversion timer running.
     */
    ad7794.last_conversion_tick =
        now;


    /*
     * MD2:MD0
     *
     * 000 = continuous conversion
     * 001 = single conversion
     * 010 = idle
     * 011 = power-down
     */
    mode =
        (uint8_t)((ad7794.mode >> 13) & 0x07u);


    /*
     * IDLE / POWER-DOWN.
     */
    if (mode == 0x02u ||
        mode == 0x03u)
    {
        ad7794.status |=
            AD7794_STATUS_RDY;

        ad7794.data_ready = false;

        PA6_As_RDY();
        update_rdy_pin();

        return;
    }


    /*
     * New conversion completed.
     */
    ad7794.status &=
        (uint8_t)~AD7794_STATUS_RDY;

    ad7794.data_ready = true;


    /*
     * In continuous-read mode the DATA transfer
     * can start automatically.
     */
    if (ad7794.spi_state ==
            AD7794_SPI_CREAD_WAIT &&
        ad7794.cread)
    {
        start_cread_data();

        return;
    }


    /*
     * Single conversion.
     *
     * The conversion is now complete.
     * AD7794 then returns to idle mode.
     */
    if (mode == 0x01u)
    {
        ad7794.mode &=
            (uint16_t)~(0x07u << 13);

        /*
         * MD = 010 (idle)
         */
        ad7794.mode |=
            (uint16_t)(0x02u << 13);
    }


    /*
     * Normal conversion:
     * RDY LOW means DATA is available.
     */
    PA6_As_RDY();
    update_rdy_pin();
}


/* ============================================================
 * CS LOW
 * ============================================================ */

void AD7794_Emu_CS_Activate(void)
{
    /*
     * Do NOT transmit a real data byte here.
     *
     * The first received byte from the master is the
     * Communications Register.
     *
     * We only prepare the SPI interface for the
     * transaction.
     */

    ad7794.cs_active = true;


    ad7794.spi_state =
        AD7794_SPI_WAIT_COMM;

    ad7794.bytes_to_xfer = 0;
    ad7794.byte_idx = 0;

    ad7794.next_reg = 0;
    ad7794.is_read = 0;
    ad7794.cread = 0;

    ad7794.reset_one_bits = 0;


    memset(ad7794.tx_buf,
           0,
           sizeof(ad7794.tx_buf));

    memset(ad7794.rx_buf,
           0,
           sizeof(ad7794.rx_buf));


    /*
     * DOUT/RDY starts inactive HIGH.
     *
     * If a conversion was already ready, update_rdy_pin()
     * will immediately drive it LOW.
     */
    PA6_As_RDY();
    update_rdy_pin();


    /*
     * The SPI IRQ is needed while ADC CS is active.
     */
    LL_SPI_EnableIT_RXNE(SPI1);


    /*
     * Dummy byte for the first SPI transfer.
     *
     * This is the response to the command byte and is
     * not part of the following register data.
     */
    SPI_PrepareTx(0xFF);
}


/* ============================================================
 * CS HIGH
 * ============================================================ */

void AD7794_Emu_CS_Deactivate(void)
{
    /*
     * CS HIGH terminates the complete ADC transaction.
     */
    ad7794.cs_active = false;


    ad7794.spi_state =
        AD7794_SPI_WAIT_COMM;

    ad7794.bytes_to_xfer = 0;
    ad7794.byte_idx = 0;

    ad7794.next_reg = 0;
    ad7794.is_read = 0;
    ad7794.cread = 0;


    ad7794.data_ready = false;
    ad7794.reset_one_bits = 0;


    /*
     * No ADC SPI RX interrupt while ADC CS is HIGH.
     *
     * EEPROM CS activation will enable RXNE again.
     */
    //LL_SPI_DisableIT_RXNE(SPI1);


    /*
     * Clear possible stale RX/OVR state.
     */
    SPI_ClearPending();


    /*
     * DOUT/RDY inactive.
     */
    PA6_RDY_High();
}


/* ============================================================
 * SPI RX/TX
 *
 * Called from SPI1_IRQHandler() after RXNE.
 * ============================================================ */

void AD7794_Emul_SPI_RxTx(uint8_t data)
{
    /*
     * Normally CS activation is handled by EXTI.
     *
     * This fallback is intentionally kept because the master
     * can start clocking almost immediately after CS LOW.
     *
     * It does NOT preload an extra byte beyond the normal
     * first response.
     */
    if (!ad7794.cs_active)
    {
        ad7794.cs_active = true;

        ad7794.spi_state =
            AD7794_SPI_WAIT_COMM;

        ad7794.bytes_to_xfer = 0;
        ad7794.byte_idx = 0;

        ad7794.reset_one_bits = 0;
        ad7794.cread = 0;

        PA6_As_RDY();

        //LL_SPI_EnableIT_RXNE(SPI1);
    }


    /* ========================================================
     * WAIT FOR COMMUNICATION REGISTER
     * ======================================================== */

    if (ad7794.spi_state ==
            AD7794_SPI_WAIT_COMM)
    {
        /*
         * 32 consecutive '1' bits reset the interface.
         *
         * Four 0xFF bytes = 32 bits.
         */
        if (data == 0xFFu)
        {
            ad7794.reset_one_bits += 8u;


            if (ad7794.reset_one_bits >=
                AD7794_RESET_BITS)
            {
                AD7794_Emu_Reset();

                /*
                 * Reset does not terminate CS.
                 */
                ad7794.cs_active = true;


                PA6_As_RDY();


                /*
                 * Response to the last reset byte.
                 */
                SPI_PrepareTx(0xFF);

                return;
            }


            SPI_PrepareTx(0xFF);

            return;
        }


        /*
         * Any other byte terminates the reset sequence.
         */
        ad7794.reset_one_bits = 0;


        process_communications_register(data);


        /*
         * CREAD:
         *
         * Wait for conversion / RDY LOW.
         */
        if (ad7794.spi_state ==
                AD7794_SPI_CREAD_WAIT)
        {
            PA6_As_RDY();
            update_rdy_pin();

            /*
             * Nothing is being read yet.
             */
            SPI_PrepareTx(0xFF);

            return;
        }


        /*
         * Normal READ already loaded first TX byte.
         */
        if (ad7794.spi_state ==
                AD7794_SPI_READ)
        {
            return;
        }


        /*
         * WRITE already prepared dummy TX.
         */
        if (ad7794.spi_state ==
                AD7794_SPI_WRITE)
        {
            return;
        }


        /*
         * Invalid command.
         */
        SPI_PrepareTx(0xFF);

        return;
    }


    /* ========================================================
     * NORMAL READ
     * ======================================================== */

    if (ad7794.spi_state ==
            AD7794_SPI_READ)
    {
        /*
         * The byte just received corresponds to the
         * TX byte which was already shifted out.
         */
        ad7794.byte_idx++;


        if (ad7794.byte_idx <
            ad7794.bytes_to_xfer)
        {
            /*
             * Prepare the NEXT byte immediately.
             */
            SPI_PrepareTx(
                ad7794.tx_buf[
                    ad7794.byte_idx
                ]);

            return;
        }


        /*
         * Entire register read completed.
         */
        finish_read();

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
                ad7794.byte_idx
            ] = data;
        }


        ad7794.byte_idx++;


        if (ad7794.byte_idx <
            ad7794.bytes_to_xfer)
        {
            /*
             * Dummy response while receiving write data.
             */
            SPI_PrepareTx(0xFF);

            return;
        }


        /*
         * Complete register write.
         */
        finish_write();

        return;
    }


    /* ========================================================
     * CREAD WAIT
     * ======================================================== */

    if (ad7794.spi_state ==
            AD7794_SPI_CREAD_WAIT)
    {
        /*
         * 0x58 exits continuous read.
         */
        if (data ==
            AD7794_COMM_EXIT_CREAD)
        {
            ad7794.cread = 0;

            ad7794.spi_state =
                AD7794_SPI_WAIT_COMM;

            ad7794.bytes_to_xfer = 0;
            ad7794.byte_idx = 0;

            PA6_As_RDY();
            update_rdy_pin();

            SPI_PrepareTx(0xFF);

            return;
        }


        /*
         * If a conversion is ready, start DATA transfer.
         *
         * Normally start_cread_data() has already put
         * the first byte into TX.
         */
        if (ad7794.data_ready)
        {
            /*
             * If Process() has not switched the state for
             * any reason, recover here.
             */
            if (ad7794.spi_state ==
                AD7794_SPI_CREAD_WAIT)
            {
                start_cread_data();

                /*
                 * First DATA byte is now prepared.
                 *
                 * It belongs to the transfer following
                 * this received byte.
                 */
                return;
            }
        }


        /*
         * Still waiting for conversion.
         */
        SPI_PrepareTx(0xFF);

        return;
    }


    /* ========================================================
     * CREAD DATA
     * ======================================================== */

    if (ad7794.spi_state ==
            AD7794_SPI_CREAD_DATA)
    {
        /*
         * First received byte corresponds to tx_buf[0].
         */
        ad7794.byte_idx++;


        if (ad7794.byte_idx <
            ad7794.bytes_to_xfer)
        {
            SPI_PrepareTx(
                ad7794.tx_buf[
                    ad7794.byte_idx
                ]);

            return;
        }


        /*
         * Complete 24-bit DATA transfer.
         */
        finish_cread_data();

        return;
    }


    /* ========================================================
     * FALLBACK
     * ======================================================== */

    ad7794.spi_state =
        AD7794_SPI_WAIT_COMM;

    ad7794.bytes_to_xfer = 0;
    ad7794.byte_idx = 0;

    ad7794.cread = 0;


    PA6_As_RDY();
    update_rdy_pin();


    SPI_PrepareTx(0xFF);
}


/* ============================================================
 * SPI ERROR
 * ============================================================ */

void AD7794_Emu_SPI_Error(void)
{
    /*
     * Do not destroy the register contents.
     *
     * Only reset the SPI protocol state.
     */
    ad7794.spi_state =
        AD7794_SPI_WAIT_COMM;

    ad7794.bytes_to_xfer = 0;
    ad7794.byte_idx = 0;

    ad7794.reset_one_bits = 0;


    PA6_As_RDY();
    update_rdy_pin();


    SPI_ClearPending();


    SPI_PrepareTx(0xFF);
}
