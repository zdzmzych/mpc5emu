#include <AD7794_emu.h>
#include <string.h>

AD7794_Emu_t ad7794;

/* Domyślne wartości (jak AD7785, ID można zmienić) */
#define AD7794_DEFAULT_STATUS     0x88
#define AD7794_DEFAULT_MODE       0x000A
#define AD7794_DEFAULT_CONFIG     0x0710
#define AD7794_DEFAULT_ID         0x4B      /* zmień jeśli znasz dokładny ID AD7794 */
#define AD7794_DEFAULT_IO         0x00
#define AD7794_DEFAULT_OFFSET     0x800000
#define AD7794_DEFAULT_FULLSCALE  0x5XXXXX  /* typowa wartość, możesz ustawić 0xFFFFFF */

/*
 * PA6:
 *
 *   GPIO_MODE_INPUT   -> Hi-Z, ADC nie steruje MISO
 *   GPIO_MODE_OUTPUT  -> RDY
 *   GPIO_MODE_AF_PP   -> SPI1_MISO
 *
 * PA6 jest fizycznie wspólnym:
 *
 *        SPI1_MISO / AD7794 DOUT/RDY
 */


/* ============================================================
 * PA6 handling
 * ============================================================ */

static void PA6_HiZ(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}


static void PA6_As_RDY(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /*
     * Najpierw konfigurujemy GPIO.
     * Wartość ustalamy przed przełączeniem pinu.
     */
    HAL_GPIO_WritePin(GPIOA,
                      GPIO_PIN_6,
                      ad7794.data_ready
                        ? GPIO_PIN_RESET
                        : GPIO_PIN_SET);

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*
     * Ustaw ponownie po konfiguracji.
     */
    HAL_GPIO_WritePin(GPIOA,
                      GPIO_PIN_6,
                      ad7794.data_ready
                        ? GPIO_PIN_RESET
                        : GPIO_PIN_SET);
}


static void PA6_As_MISO(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;

    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}


/*
 * RDY:
 *
 *   0 = data ready
 *   1 = data not ready
 */
static void update_rdy_pin(void)
{
    /*
     * Jeżeli ADC nie jest wybrany, PA6 musi być Hi-Z.
     */
    if (HAL_GPIO_ReadPin(CS_ADC_GPIO_Port, CS_ADC_Pin)
            != GPIO_PIN_RESET)
    {
        PA6_HiZ();
        return;
    }

    PA6_As_RDY();
}

static void prepare_tx_buffer(void)
{
    memset(ad7794.tx_buf, 0xFF, sizeof(ad7794.tx_buf));

    switch (ad7794.next_reg) {
        case AD7794_REG_STATUS:
            ad7794.tx_buf[0] = ad7794.status;
            ad7794.bytes_to_xfer = 1;
            break;

        case AD7794_REG_MODE:
            ad7794.tx_buf[0] = (ad7794.mode >> 8) & 0xFF;
            ad7794.tx_buf[1] = ad7794.mode & 0xFF;
            ad7794.bytes_to_xfer = 2;
            break;

        case AD7794_REG_CONFIG:
            ad7794.tx_buf[0] = (ad7794.config >> 8) & 0xFF;
            ad7794.tx_buf[1] = ad7794.config & 0xFF;
            ad7794.bytes_to_xfer = 2;
            break;

        case AD7794_REG_DATA:
            ad7794.tx_buf[0] = (ad7794.data >> 16) & 0xFF;
            ad7794.tx_buf[1] = (ad7794.data >> 8)  & 0xFF;
            ad7794.tx_buf[2] =  ad7794.data        & 0xFF;
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
            ad7794.tx_buf[0] = (ad7794.offset >> 16) & 0xFF;
            ad7794.tx_buf[1] = (ad7794.offset >> 8)  & 0xFF;
            ad7794.tx_buf[2] =  ad7794.offset        & 0xFF;
            ad7794.bytes_to_xfer = 3;
            break;

        case AD7794_REG_FULLSCALE:
            ad7794.tx_buf[0] = (ad7794.fullscale >> 16) & 0xFF;
            ad7794.tx_buf[1] = (ad7794.fullscale >> 8)  & 0xFF;
            ad7794.tx_buf[2] =  ad7794.fullscale        & 0xFF;
            ad7794.bytes_to_xfer = 3;
            break;

        default:
            ad7794.bytes_to_xfer = 1;
            break;
    }
    ad7794.byte_idx = 0;
}

static void process_communications_register(uint8_t comm)
{
    /* WEN musi być 0 */
    if (comm & 0x80) return;

    //cb_push(ad7794.pcb, comm);
    ad7794.is_read  = (comm >> 6) & 0x01;
    ad7794.next_reg = (comm >> 3) & 0x07;
    ad7794.cread    = (comm >> 2) & 0x01;

    if (ad7794.is_read) {
        prepare_tx_buffer();
    } else {
        /* Write – czekamy na kolejne bajty */
        switch (ad7794.next_reg) {
            case AD7794_REG_MODE:
            case AD7794_REG_CONFIG:
                ad7794.bytes_to_xfer = 2;
                break;
            case AD7794_REG_IO:
                ad7794.bytes_to_xfer = 1;
                break;
            case AD7794_REG_OFFSET:
            case AD7794_REG_FULLSCALE:
                ad7794.bytes_to_xfer = 3;
                break;
            default:
                ad7794.bytes_to_xfer = 0;
                break;
        }
        ad7794.byte_idx = 0;
    }
}

static void process_write_data(void)
{
    switch (ad7794.next_reg) {
        case AD7794_REG_MODE:
            ad7794.mode = ((uint16_t)ad7794.rx_buf[0] << 8) | ad7794.rx_buf[1];
            /* Zmiana Mode resetuje filtr → RDY */
            ad7794.status |= AD7794_STATUS_RDY;
            ad7794.data_ready = false;
            update_rdy_pin();
            break;

        case AD7794_REG_CONFIG:
            ad7794.config = ((uint16_t)ad7794.rx_buf[0] << 8) | ad7794.rx_buf[1];
            ad7794.status |= AD7794_STATUS_RDY;
            ad7794.data_ready = false;
            update_rdy_pin();
            break;

        case AD7794_REG_IO:
            ad7794.io = ad7794.rx_buf[0];
            break;

        case AD7794_REG_OFFSET:
            ad7794.offset = ((uint32_t)ad7794.rx_buf[0] << 16) |
                            ((uint32_t)ad7794.rx_buf[1] << 8)  |
                             (uint32_t)ad7794.rx_buf[2];
            break;

        case AD7794_REG_FULLSCALE:
            ad7794.fullscale = ((uint32_t)ad7794.rx_buf[0] << 16) |
                               ((uint32_t)ad7794.rx_buf[1] << 8)  |
                                (uint32_t)ad7794.rx_buf[2];
            break;
    }
}

void AD7794_Emu_Reset(void)
{
    ad7794.status    = AD7794_DEFAULT_STATUS;
    ad7794.mode      = AD7794_DEFAULT_MODE;
    ad7794.config    = AD7794_DEFAULT_CONFIG;
    ad7794.id        = AD7794_DEFAULT_ID;
    ad7794.io        = AD7794_DEFAULT_IO;
    ad7794.offset    = AD7794_DEFAULT_OFFSET;
    ad7794.fullscale = 0x5A5A5A;

    ad7794.data_ready = false;
    ad7794.cread      = 0;

    PA6_HiZ();
}

void AD7794_Emu_SetData(uint32_t value_24bit)
{
    ad7794.data = value_24bit & 0xFFFFFF;
}

void AD7794_Emu_Init(
    SPI_HandleTypeDef *hspi,
    CircularBuffer *cb)
{
    memset(&ad7794, 0, sizeof(ad7794));

    ad7794.hspi = hspi;
    ad7794.pcb  = cb;

    AD7794_Emu_Reset();

    AD7794_Emu_SetData(0x123456);

    ad7794.conversion_period_ms = 50;

    /*
     * ADC na początku nie jest wybrany.
     *
     * PA6 = Hi-Z.
     *
     * Bardzo ważne:
     * NIE uruchamiamy tutaj HAL_SPI_TransmitReceive_IT().
     */
    PA6_HiZ();

    ad7794.bytes_to_xfer = 0;
}

void AD7794_Emu_Process(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - ad7794.last_conversion_tick) >=
        ad7794.conversion_period_ms)
    {
        ad7794.last_conversion_tick = now;

        /*
         * Conversion completed.
         * RDY = 0.
         */
        ad7794.status &= ~AD7794_STATUS_RDY;
        ad7794.data_ready = true;

        update_rdy_pin();
    }
}

/* ===== Callbacki SPI ===== */
void AD7794_Emu_SPI_RxTxCplt(void)
{
    /*
     * CS może zostać podniesiony praktycznie w dowolnym momencie.
     */
    if (HAL_GPIO_ReadPin(CS_ADC_GPIO_Port, CS_ADC_Pin)
            != GPIO_PIN_RESET)
    {
        PA6_HiZ();
        return;
    }


    /*
     * --------------------------------------------------------
     * Communications Register
     * --------------------------------------------------------
     */
    if (ad7794.bytes_to_xfer == 0)
    {
        uint8_t comm = ad7794.rx_buf[0];

        process_communications_register(comm);

        /*
         * READ
         */
        if (ad7794.is_read)
        {
            /*
             * Do tej pory PA6 był RDY.
             *
             * Od następnego bajtu ma być MISO.
             */
            PA6_As_MISO();

            HAL_SPI_TransmitReceive_IT(
                ad7794.hspi,
                ad7794.tx_buf,
                ad7794.rx_buf,
                ad7794.bytes_to_xfer);

            return;
        }


        /*
         * WRITE
         */
        if (ad7794.bytes_to_xfer > 0)
        {
            /*
             * Przy zapisie PA6 nadal może zachowywać się
             * jako RDY.
             */
            PA6_As_RDY();

            HAL_SPI_TransmitReceive_IT(
                ad7794.hspi,
                ad7794.tx_buf,
                ad7794.rx_buf,
                ad7794.bytes_to_xfer);

            return;
        }


        /*
         * Communications Register bez dalszych danych.
         */
        PA6_As_RDY();

        HAL_SPI_TransmitReceive_IT(
            ad7794.hspi,
            ad7794.tx_buf,
            ad7794.rx_buf,
            1);

        return;
    }


    /*
     * --------------------------------------------------------
     * Koniec transferu rejestru
     * --------------------------------------------------------
     */

    if (!ad7794.is_read)
    {
        process_write_data();
    }


    /*
     * --------------------------------------------------------
     * DATA zostało odczytane
     * --------------------------------------------------------
     *
     * RDY wraca do stanu NOT READY.
     */
    if (ad7794.is_read &&
        ad7794.next_reg == AD7794_REG_DATA)
    {
        ad7794.data_ready = false;
        ad7794.status |= AD7794_STATUS_RDY;

        /*
         * Po zakończeniu odczytu wracamy z MISO do RDY.
         */
        PA6_As_RDY();
    }


    /*
     * Continuous Read
     */
    if (ad7794.cread &&
        ad7794.next_reg == AD7794_REG_DATA)
    {
        prepare_tx_buffer();

        PA6_As_MISO();

        HAL_SPI_TransmitReceive_IT(
            ad7794.hspi,
            ad7794.tx_buf,
            ad7794.rx_buf,
            ad7794.bytes_to_xfer);

        return;
    }


    /*
     * Następny transfer zaczyna się od Communications Register.
     *
     * Dlatego PA6 musi znowu być RDY.
     */
    ad7794.bytes_to_xfer = 0;
    ad7794.byte_idx = 0;

    PA6_As_RDY();

    HAL_SPI_TransmitReceive_IT(
        ad7794.hspi,
        ad7794.tx_buf,
        ad7794.rx_buf,
        1);
}

void AD7794_Emu_SPI_Error(void)
{
    /* Po błędzie restartujemy odbiór */
    ad7794.bytes_to_xfer = 0;
    HAL_SPI_TransmitReceive_IT(ad7794.hspi, ad7794.tx_buf, ad7794.rx_buf, 1);
}

void AD7794_Emu_CS_Activate(void)
{
    /*
     * ADC został wybrany.
     *
     * Na początku PA6 jest RDY, a nie MISO.
     */
    ad7794.bytes_to_xfer = 0;
    ad7794.byte_idx = 0;

    /*
     * Przywracamy stan RDY.
     */
    PA6_As_RDY();

    /*
     * Przygotuj odbiór Communications Register.
     *
     * Pierwszy bajt wysyłany przez ADC jest w tej chwili
     * nieistotny z punktu widzenia protokołu.
     */
    ad7794.tx_buf[0] = 0xFF;
    ad7794.rx_buf[0] = 0x00;

    HAL_SPI_TransmitReceive_IT(
        ad7794.hspi,
        ad7794.tx_buf,
        ad7794.rx_buf,
        1);
}


void AD7794_Emu_CS_Deactivate(void)
{
    /*
     * Przerwij ewentualny transfer.
     */
    HAL_SPI_Abort_IT(ad7794.hspi);

    ad7794.bytes_to_xfer = 0;
    ad7794.byte_idx = 0;

    /*
     * Po CS=HIGH AD7794 odłącza DOUT/RDY od magistrali.
     */
    PA6_HiZ();
}
