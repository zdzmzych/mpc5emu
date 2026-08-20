#include <ad7794_emu.h>
#include <string.h>

AD7794_Emu_t AD7794;

/* Domyślne wartości (jak AD7785, ID można zmienić) */
#define AD7794_DEFAULT_STATUS     0x88
#define AD7794_DEFAULT_MODE       0x000A
#define AD7794_DEFAULT_CONFIG     0x0710
#define AD7794_DEFAULT_ID         0x4B      /* zmień jeśli znasz dokładny ID AD7794 */
#define AD7794_DEFAULT_IO         0x00
#define AD7794_DEFAULT_OFFSET     0x800000
#define AD7794_DEFAULT_FULLSCALE  0x5XXXXX  /* typowa wartość, możesz ustawić 0xFFFFFF */

static void update_rdy_pin(void)
{
    if (AD7794.data_ready) {
        HAL_GPIO_WritePin(AD7794.rdy_port, AD7794.rdy_pin, GPIO_PIN_RESET); /* aktywny niski */
    } else {
        HAL_GPIO_WritePin(AD7794.rdy_port, AD7794.rdy_pin, GPIO_PIN_SET);
    }
}

static void prepare_tx_buffer(void)
{
    memset(AD7794.tx_buf, 0xFF, sizeof(AD7794.tx_buf));

    switch (AD7794.next_reg) {
        case AD7794_REG_STATUS:
            AD7794.tx_buf[0] = AD7794.status;
            AD7794.bytes_to_xfer = 1;
            break;

        case AD7794_REG_MODE:
            AD7794.tx_buf[0] = (AD7794.mode >> 8) & 0xFF;
            AD7794.tx_buf[1] = AD7794.mode & 0xFF;
            AD7794.bytes_to_xfer = 2;
            break;

        case AD7794_REG_CONFIG:
            AD7794.tx_buf[0] = (AD7794.config >> 8) & 0xFF;
            AD7794.tx_buf[1] = AD7794.config & 0xFF;
            AD7794.bytes_to_xfer = 2;
            break;

        case AD7794_REG_DATA:
            AD7794.tx_buf[0] = (AD7794.data >> 16) & 0xFF;
            AD7794.tx_buf[1] = (AD7794.data >> 8)  & 0xFF;
            AD7794.tx_buf[2] =  AD7794.data        & 0xFF;
            AD7794.bytes_to_xfer = 3;
            /* Po odczycie Data RDY wraca w stan „niegotowy” */
            AD7794.status |= AD7794_STATUS_RDY;
            AD7794.data_ready = false;
            update_rdy_pin();
            break;

        case AD7794_REG_ID:
            AD7794.tx_buf[0] = AD7794.id;
            AD7794.bytes_to_xfer = 1;
            break;

        case AD7794_REG_IO:
            AD7794.tx_buf[0] = AD7794.io;
            AD7794.bytes_to_xfer = 1;
            break;

        case AD7794_REG_OFFSET:
            AD7794.tx_buf[0] = (AD7794.offset >> 16) & 0xFF;
            AD7794.tx_buf[1] = (AD7794.offset >> 8)  & 0xFF;
            AD7794.tx_buf[2] =  AD7794.offset        & 0xFF;
            AD7794.bytes_to_xfer = 3;
            break;

        case AD7794_REG_FULLSCALE:
            AD7794.tx_buf[0] = (AD7794.fullscale >> 16) & 0xFF;
            AD7794.tx_buf[1] = (AD7794.fullscale >> 8)  & 0xFF;
            AD7794.tx_buf[2] =  AD7794.fullscale        & 0xFF;
            AD7794.bytes_to_xfer = 3;
            break;

        default:
            AD7794.bytes_to_xfer = 1;
            break;
    }
    AD7794.byte_idx = 0;
}

static void process_communications_register(uint8_t comm)
{
    /* WEN musi być 0 */
    if (comm & 0x80) return;

    //cb_push(AD7794.pcb, comm);
    AD7794.is_read  = (comm >> 6) & 0x01;
    AD7794.next_reg = (comm >> 3) & 0x07;
    AD7794.cread    = (comm >> 2) & 0x01;

    if (AD7794.is_read) {
        prepare_tx_buffer();
    } else {
        /* Write – czekamy na kolejne bajty */
        switch (AD7794.next_reg) {
            case AD7794_REG_MODE:
            case AD7794_REG_CONFIG:
                AD7794.bytes_to_xfer = 2;
                break;
            case AD7794_REG_IO:
                AD7794.bytes_to_xfer = 1;
                break;
            case AD7794_REG_OFFSET:
            case AD7794_REG_FULLSCALE:
                AD7794.bytes_to_xfer = 3;
                break;
            default:
                AD7794.bytes_to_xfer = 0;
                break;
        }
        AD7794.byte_idx = 0;
    }
}

static void process_write_data(void)
{
    switch (AD7794.next_reg) {
        case AD7794_REG_MODE:
            AD7794.mode = ((uint16_t)AD7794.rx_buf[0] << 8) | AD7794.rx_buf[1];
            /* Zmiana Mode resetuje filtr → RDY */
            AD7794.status |= AD7794_STATUS_RDY;
            AD7794.data_ready = false;
            update_rdy_pin();
            break;

        case AD7794_REG_CONFIG:
            AD7794.config = ((uint16_t)AD7794.rx_buf[0] << 8) | AD7794.rx_buf[1];
            AD7794.status |= AD7794_STATUS_RDY;
            AD7794.data_ready = false;
            update_rdy_pin();
            break;

        case AD7794_REG_IO:
            AD7794.io = AD7794.rx_buf[0];
            break;

        case AD7794_REG_OFFSET:
            AD7794.offset = ((uint32_t)AD7794.rx_buf[0] << 16) |
                            ((uint32_t)AD7794.rx_buf[1] << 8)  |
                             (uint32_t)AD7794.rx_buf[2];
            break;

        case AD7794_REG_FULLSCALE:
            AD7794.fullscale = ((uint32_t)AD7794.rx_buf[0] << 16) |
                               ((uint32_t)AD7794.rx_buf[1] << 8)  |
                                (uint32_t)AD7794.rx_buf[2];
            break;
    }
}

void AD7794_Emu_Init(SPI_HandleTypeDef *hspi, CircularBuffer *cb,
                     GPIO_TypeDef *rdy_port, uint16_t rdy_pin)
{
    memset(&AD7794, 0, sizeof(AD7794));

    AD7794.hspi     = hspi;
    AD7794.rdy_port = rdy_port;
    AD7794.rdy_pin  = rdy_pin;
    AD7794.pcb = cb;

    AD7794_Emu_Reset();

    /* Stała wartość danych – możesz zmienić */
    AD7794_Emu_SetData(0x123456);

    AD7794.conversion_period_ms = 50;   /* 20 Hz – dostosuj do Mode */

    /* Start pierwszego odbioru (1 bajt Communications) */
    HAL_SPI_TransmitReceive_IT(AD7794.hspi, AD7794.tx_buf, AD7794.rx_buf, 1);
}

void AD7794_Emu_Reset(void)
{
    AD7794.status    = AD7794_DEFAULT_STATUS;
    AD7794.mode      = AD7794_DEFAULT_MODE;
    AD7794.config    = AD7794_DEFAULT_CONFIG;
    AD7794.id        = AD7794_DEFAULT_ID;
    AD7794.io        = AD7794_DEFAULT_IO;
    AD7794.offset    = AD7794_DEFAULT_OFFSET;
    AD7794.fullscale = 0x5A5A5A;   /* przykładowa wartość */

    AD7794.data_ready = false;
    AD7794.cread      = 0;
    update_rdy_pin();
}

void AD7794_Emu_SetData(uint32_t value_24bit)
{
    AD7794.data = value_24bit & 0xFFFFFF;
}

void AD7794_Emu_Process(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - AD7794.last_conversion_tick) >= AD7794.conversion_period_ms) {
        AD7794.last_conversion_tick = now;

        /* Nowa „konwersja” – stała wartość już ustawiona */
        AD7794.status &= ~AD7794_STATUS_RDY;   /* RDY = 0 → dane gotowe */
        AD7794.data_ready = true;
        update_rdy_pin();
    }
}

/* ===== Callbacki SPI ===== */
void AD7794_Emu_SPI_RxTxCplt(void)
{
    if (AD7794.bytes_to_xfer == 0) {
        /* To był bajt Communications Register */
        process_communications_register(AD7794.rx_buf[0]);

        if (AD7794.is_read) {
            /* Wysyłamy dane rejestru */
            HAL_SPI_TransmitReceive_IT(AD7794.hspi,
                                       AD7794.tx_buf,
                                       AD7794.rx_buf,
                                       AD7794.bytes_to_xfer);
        } else if (AD7794.bytes_to_xfer > 0) {
            /* Odbieramy dane do zapisu */
            HAL_SPI_TransmitReceive_IT(AD7794.hspi,
                                       AD7794.tx_buf,
                                       AD7794.rx_buf,
                                       AD7794.bytes_to_xfer);
        } else {
            /* Nic do zrobienia – czekamy na następny Communications */
            HAL_SPI_TransmitReceive_IT(AD7794.hspi, AD7794.tx_buf, AD7794.rx_buf, 1);
        }
    } else {
    	//cb_pusha(AD7794.pcb, AD7794.rx_buf, AD7794.bytes_to_xfer);
        /* Zakończyliśmy transfer rejestru */
        if (!AD7794.is_read) {
            process_write_data();
        }

        /* Continuous Read? */
        if (AD7794.cread && AD7794.next_reg == AD7794_REG_DATA) {
            prepare_tx_buffer();
            HAL_SPI_TransmitReceive_IT(AD7794.hspi,
                                       AD7794.tx_buf,
                                       AD7794.rx_buf,
                                       AD7794.bytes_to_xfer);
        } else {
            /* Wracamy do oczekiwania na Communications */
            AD7794.bytes_to_xfer = 0;
            HAL_SPI_TransmitReceive_IT(AD7794.hspi, AD7794.tx_buf, AD7794.rx_buf, 1);
        }
    }
}

void AD7794_Emu_SPI_Error(void)
{
    /* Po błędzie restartujemy odbiór */
    AD7794.bytes_to_xfer = 0;
    HAL_SPI_TransmitReceive_IT(AD7794.hspi, AD7794.tx_buf, AD7794.rx_buf, 1);
}

/* Opcjonalnie – podłącz do EXTI na CS (falling) */
void AD7794_Emu_CS_Falling(void)
{
    /* Można tu zresetować stan maszyny jeśli CS spadnie w trakcie */
}
