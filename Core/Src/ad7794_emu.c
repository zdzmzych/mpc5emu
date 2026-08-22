#include "ad7794_emu.h"
#include <string.h>

AD7794_Emu_t ad7794;

/* Domyślne wartości (seria AD77xx) */
#define AD7794_DEFAULT_STATUS     0x88u
#define AD7794_DEFAULT_MODE       0x000Au
#define AD7794_DEFAULT_CONFIG     0x0710u
#define AD7794_DEFAULT_ID         0x4Bu      /* dostosuj do prawdziwego ID AD7794 */
#define AD7794_DEFAULT_IO         0x00u
#define AD7794_DEFAULT_OFFSET     0x800000u
#define AD7794_DEFAULT_FULLSCALE  0x5A5A5Au

/* PA6 = SPI1_MISO / DOUT/RDY */
static void PA6_HiZ(void)
{
    GPIO_InitTypeDef g = {0};
    g.Pin  = GPIO_PIN_6;
    g.Mode = GPIO_MODE_INPUT;
    g.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &g);
}

static void PA6_As_RDY(void)
{
    GPIO_InitTypeDef g = {0};

    /* wartość przed przełączeniem trybu */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,
                      ad7794.data_ready ? GPIO_PIN_RESET : GPIO_PIN_SET);

    g.Pin   = GPIO_PIN_6;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6,
                      ad7794.data_ready ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void PA6_As_MISO(void)
{
    GPIO_InitTypeDef g = {0};
    g.Pin       = GPIO_PIN_6;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF0_SPI1;
    HAL_GPIO_Init(GPIOA, &g);
}

static void update_rdy_pin(void)
{
    if (!ad7794.cs_active) {
        PA6_HiZ();
        return;
    }
    PA6_As_RDY();
}

static bool spi_cs_is_active(void)
{
    return (HAL_GPIO_ReadPin(CS_ADC_GPIO_Port, CS_ADC_Pin) == GPIO_PIN_RESET);
}

static void spi_start_rx_tx(uint16_t n)
{
    if (!spi_cs_is_active() || ad7794.hspi == NULL) {
        return;
    }

    /* zabezpieczenie przed double-start */
    if (ad7794.hspi->State != HAL_SPI_STATE_READY) {
        HAL_SPI_Abort(ad7794.hspi);
    }

    HAL_SPI_TransmitReceive_IT(ad7794.hspi, ad7794.tx_buf, ad7794.rx_buf, n);
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
        ad7794.tx_buf[0] = (uint8_t)(ad7794.mode >> 8);
        ad7794.tx_buf[1] = (uint8_t)(ad7794.mode & 0xFF);
        ad7794.bytes_to_xfer = 2;
        break;

    case AD7794_REG_CONFIG:
        ad7794.tx_buf[0] = (uint8_t)(ad7794.config >> 8);
        ad7794.tx_buf[1] = (uint8_t)(ad7794.config & 0xFF);
        ad7794.bytes_to_xfer = 2;
        break;

    case AD7794_REG_DATA:
        ad7794.tx_buf[0] = (uint8_t)((ad7794.data >> 16) & 0xFF);
        ad7794.tx_buf[1] = (uint8_t)((ad7794.data >> 8)  & 0xFF);
        ad7794.tx_buf[2] = (uint8_t)( ad7794.data        & 0xFF);
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
        ad7794.tx_buf[0] = (uint8_t)((ad7794.offset >> 16) & 0xFF);
        ad7794.tx_buf[1] = (uint8_t)((ad7794.offset >> 8)  & 0xFF);
        ad7794.tx_buf[2] = (uint8_t)( ad7794.offset        & 0xFF);
        ad7794.bytes_to_xfer = 3;
        break;

    case AD7794_REG_FULLSCALE:
        ad7794.tx_buf[0] = (uint8_t)((ad7794.fullscale >> 16) & 0xFF);
        ad7794.tx_buf[1] = (uint8_t)((ad7794.fullscale >> 8)  & 0xFF);
        ad7794.tx_buf[2] = (uint8_t)( ad7794.fullscale        & 0xFF);
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
    /* Soft-reset: ciąg 0xFF */
    if (comm == 0xFFu) {
        ad7794.ff_count++;
        if (ad7794.ff_count >= 32u) {
            AD7794_Emu_Reset();
            ad7794.ff_count = 0;
        }
        ad7794.bytes_to_xfer = 0;
        return;
    }
    ad7794.ff_count = 0;

    /* WEN musi być 0 */
    if (comm & 0x80u) {
        ad7794.bytes_to_xfer = 0;
        return;
    }

    ad7794.is_read  = (comm >> 6) & 0x01u;
    ad7794.next_reg = (comm >> 3) & 0x07u;
    ad7794.cread    = (comm >> 2) & 0x01u;

    /* proste logowanie: R=read, W=write, numer rejestru */
    cb_push(ad7794.pcb, ad7794.is_read ? 'R' : 'W');
    cb_push(ad7794.pcb, (uint8_t)('0' + ad7794.next_reg));

    if (ad7794.is_read) {
        prepare_tx_buffer();
    } else {
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
            /* Status/Data/ID – zapis nieobsługiwany */
            ad7794.bytes_to_xfer = 0;
            break;
        }
        ad7794.byte_idx = 0;
        memset(ad7794.rx_buf, 0, sizeof(ad7794.rx_buf));
    }
}

static void process_write_data(void)
{
    switch (ad7794.next_reg) {
    case AD7794_REG_MODE:
        ad7794.mode = ((uint16_t)ad7794.rx_buf[0] << 8) | ad7794.rx_buf[1];
        /* zapis Mode → reset filtra */
        ad7794.status |= AD7794_STATUS_RDY;
        ad7794.data_ready = false;
        update_rdy_pin();
        cb_push(ad7794.pcb, 'M');
        break;

    case AD7794_REG_CONFIG:
        ad7794.config = ((uint16_t)ad7794.rx_buf[0] << 8) | ad7794.rx_buf[1];
        ad7794.status |= AD7794_STATUS_RDY;
        ad7794.data_ready = false;
        update_rdy_pin();
        cb_push(ad7794.pcb, 'C');
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

    default:
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
    ad7794.fullscale = AD7794_DEFAULT_FULLSCALE;

    ad7794.data_ready = false;
    ad7794.cread      = 0;
    ad7794.ff_count   = 0;
    ad7794.bytes_to_xfer = 0;
    ad7794.byte_idx   = 0;

    if (ad7794.cs_active) {
        PA6_As_RDY();
    } else {
        PA6_HiZ();
    }
}

void AD7794_Emu_SetData(uint32_t value_24bit)
{
    ad7794.data = value_24bit & 0xFFFFFFu;
}

void AD7794_Emu_Init(SPI_HandleTypeDef *hspi, CircularBuffer *cb)
{
    memset(&ad7794, 0, sizeof(ad7794));

    ad7794.hspi = hspi;
    ad7794.pcb  = cb;

    AD7794_Emu_Reset();
    AD7794_Emu_SetData(0x123456u);

    ad7794.conversion_period_ms = 50;   /* ~20 Hz – dostosuj */
    ad7794.cs_active = false;

    PA6_HiZ();
    /* NIE startujemy SPI IT dopóki CS nie spadnie */
}

void AD7794_Emu_Process(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - ad7794.last_conversion_tick) >= ad7794.conversion_period_ms) {
        ad7794.last_conversion_tick = now;

        /* konwersja zakończona → RDY = 0 */
        ad7794.status &= (uint8_t)~AD7794_STATUS_RDY;
        ad7794.data_ready = true;
        update_rdy_pin();
    }
}

void AD7794_Emu_CS_Activate(void)
{
    cb_push(ad7794.pcb, 'S');
    ad7794.cs_active = true;
    ad7794.bytes_to_xfer = 0;
    ad7794.byte_idx = 0;
    ad7794.ff_count = 0;

    /* Na starcie transakcji DOUT/RDY = RDY */
    PA6_As_RDY();

    ad7794.tx_buf[0] = 0xFF;
    ad7794.rx_buf[0] = 0x00;

    spi_start_rx_tx(1);
}

void AD7794_Emu_CS_Deactivate(void)
{
    cb_push(ad7794.pcb, 's');
    ad7794.cs_active = false;
    HAL_SPI_Abort_IT(ad7794.hspi);

    ad7794.bytes_to_xfer = 0;
    ad7794.byte_idx = 0;
    ad7794.cread = 0;

    /* Odłączenie od magistrali */
    PA6_HiZ();
}

void AD7794_Emu_SPI_RxTxCplt(void)
{
    if (!spi_cs_is_active()) {
        ad7794.cs_active = false;
        PA6_HiZ();
        return;
    }

    /* ----- Communications Register (oczekujemy 1 bajtu) ----- */
    if (ad7794.bytes_to_xfer == 0) {
        uint8_t comm = ad7794.rx_buf[0];

        process_communications_register(comm);

        if (ad7794.is_read && ad7794.bytes_to_xfer > 0) {
            /* odczyt rejestru → MISO */
            PA6_As_MISO();
            spi_start_rx_tx(ad7794.bytes_to_xfer);
            return;
        }

        if (!ad7794.is_read && ad7794.bytes_to_xfer > 0) {
            /* zapis rejestru → zostaw RDY, odbierz dane */
            PA6_As_RDY();
            spi_start_rx_tx(ad7794.bytes_to_xfer);
            return;
        }

        /* sam Communications bez danych (np. zły WEN / soft-reset w toku) */
        PA6_As_RDY();
        spi_start_rx_tx(1);
        return;
    }

    /* ----- Koniec transferu rejestru ----- */
    if (!ad7794.is_read) {
        process_write_data();
    }

    /* Po odczycie DATA → RDY wraca w stan „not ready” */
    if (ad7794.is_read && ad7794.next_reg == AD7794_REG_DATA) {
        ad7794.data_ready = false;
        ad7794.status |= AD7794_STATUS_RDY;
        PA6_As_RDY();
    }

    /* Continuous Read */
    if (ad7794.cread && ad7794.next_reg == AD7794_REG_DATA) {
        prepare_tx_buffer();
        PA6_As_MISO();
        spi_start_rx_tx(ad7794.bytes_to_xfer);
        return;
    }

    /* Kolejna transakcja zaczyna się od Communications */
    ad7794.bytes_to_xfer = 0;
    ad7794.byte_idx = 0;
    PA6_As_RDY();
    spi_start_rx_tx(1);
}

void AD7794_Emu_SPI_Error(void)
{
    ad7794.bytes_to_xfer = 0;

    if (spi_cs_is_active()) {
        PA6_As_RDY();
        spi_start_rx_tx(1);
    } else {
        PA6_HiZ();
    }
}
