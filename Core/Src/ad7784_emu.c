#include "ad7784_emu.h"
#include <string.h>

AD7784_Emu_t ad7784;

/* Domyślne wartości (jak AD7785, ID można zmienić) */
#define AD7784_DEFAULT_STATUS     0x88
#define AD7784_DEFAULT_MODE       0x000A
#define AD7784_DEFAULT_CONFIG     0x0710
#define AD7784_DEFAULT_ID         0x4B      /* zmień jeśli znasz dokładny ID AD7784 */
#define AD7784_DEFAULT_IO         0x00
#define AD7784_DEFAULT_OFFSET     0x800000
#define AD7784_DEFAULT_FULLSCALE  0x5XXXXX  /* typowa wartość, możesz ustawić 0xFFFFFF */

static void update_rdy_pin(void)
{
    if (ad7784.data_ready) {
        HAL_GPIO_WritePin(ad7784.rdy_port, ad7784.rdy_pin, GPIO_PIN_RESET); /* aktywny niski */
    } else {
        HAL_GPIO_WritePin(ad7784.rdy_port, ad7784.rdy_pin, GPIO_PIN_SET);
    }
}

static void prepare_tx_buffer(void)
{
    memset(ad7784.tx_buf, 0xFF, sizeof(ad7784.tx_buf));

    switch (ad7784.next_reg) {
        case AD7784_REG_STATUS:
            ad7784.tx_buf[0] = ad7784.status;
            ad7784.bytes_to_xfer = 1;
            break;

        case AD7784_REG_MODE:
            ad7784.tx_buf[0] = (ad7784.mode >> 8) & 0xFF;
            ad7784.tx_buf[1] = ad7784.mode & 0xFF;
            ad7784.bytes_to_xfer = 2;
            break;

        case AD7784_REG_CONFIG:
            ad7784.tx_buf[0] = (ad7784.config >> 8) & 0xFF;
            ad7784.tx_buf[1] = ad7784.config & 0xFF;
            ad7784.bytes_to_xfer = 2;
            break;

        case AD7784_REG_DATA:
            ad7784.tx_buf[0] = (ad7784.data >> 16) & 0xFF;
            ad7784.tx_buf[1] = (ad7784.data >> 8)  & 0xFF;
            ad7784.tx_buf[2] =  ad7784.data        & 0xFF;
            ad7784.bytes_to_xfer = 3;
            /* Po odczycie Data RDY wraca w stan „niegotowy” */
            ad7784.status |= AD7784_STATUS_RDY;
            ad7784.data_ready = false;
            update_rdy_pin();
            break;

        case AD7784_REG_ID:
            ad7784.tx_buf[0] = ad7784.id;
            ad7784.bytes_to_xfer = 1;
            break;

        case AD7784_REG_IO:
            ad7784.tx_buf[0] = ad7784.io;
            ad7784.bytes_to_xfer = 1;
            break;

        case AD7784_REG_OFFSET:
            ad7784.tx_buf[0] = (ad7784.offset >> 16) & 0xFF;
            ad7784.tx_buf[1] = (ad7784.offset >> 8)  & 0xFF;
            ad7784.tx_buf[2] =  ad7784.offset        & 0xFF;
            ad7784.bytes_to_xfer = 3;
            break;

        case AD7784_REG_FULLSCALE:
            ad7784.tx_buf[0] = (ad7784.fullscale >> 16) & 0xFF;
            ad7784.tx_buf[1] = (ad7784.fullscale >> 8)  & 0xFF;
            ad7784.tx_buf[2] =  ad7784.fullscale        & 0xFF;
            ad7784.bytes_to_xfer = 3;
            break;

        default:
            ad7784.bytes_to_xfer = 1;
            break;
    }
    ad7784.byte_idx = 0;
}

static void process_communications_register(uint8_t comm)
{
    /* WEN musi być 0 */
    if (comm & 0x80) return;

    //cb_push(ad7784.pcb, comm);
    ad7784.is_read  = (comm >> 6) & 0x01;
    ad7784.next_reg = (comm >> 3) & 0x07;
    ad7784.cread    = (comm >> 2) & 0x01;

    if (ad7784.is_read) {
        prepare_tx_buffer();
    } else {
        /* Write – czekamy na kolejne bajty */
        switch (ad7784.next_reg) {
            case AD7784_REG_MODE:
            case AD7784_REG_CONFIG:
                ad7784.bytes_to_xfer = 2;
                break;
            case AD7784_REG_IO:
                ad7784.bytes_to_xfer = 1;
                break;
            case AD7784_REG_OFFSET:
            case AD7784_REG_FULLSCALE:
                ad7784.bytes_to_xfer = 3;
                break;
            default:
                ad7784.bytes_to_xfer = 0;
                break;
        }
        ad7784.byte_idx = 0;
    }
}

static void process_write_data(void)
{
    switch (ad7784.next_reg) {
        case AD7784_REG_MODE:
            ad7784.mode = ((uint16_t)ad7784.rx_buf[0] << 8) | ad7784.rx_buf[1];
            /* Zmiana Mode resetuje filtr → RDY */
            ad7784.status |= AD7784_STATUS_RDY;
            ad7784.data_ready = false;
            update_rdy_pin();
            break;

        case AD7784_REG_CONFIG:
            ad7784.config = ((uint16_t)ad7784.rx_buf[0] << 8) | ad7784.rx_buf[1];
            ad7784.status |= AD7784_STATUS_RDY;
            ad7784.data_ready = false;
            update_rdy_pin();
            break;

        case AD7784_REG_IO:
            ad7784.io = ad7784.rx_buf[0];
            break;

        case AD7784_REG_OFFSET:
            ad7784.offset = ((uint32_t)ad7784.rx_buf[0] << 16) |
                            ((uint32_t)ad7784.rx_buf[1] << 8)  |
                             (uint32_t)ad7784.rx_buf[2];
            break;

        case AD7784_REG_FULLSCALE:
            ad7784.fullscale = ((uint32_t)ad7784.rx_buf[0] << 16) |
                               ((uint32_t)ad7784.rx_buf[1] << 8)  |
                                (uint32_t)ad7784.rx_buf[2];
            break;
    }
}

void AD7784_Emu_Init(SPI_HandleTypeDef *hspi, CircularBuffer *cb,
                     GPIO_TypeDef *rdy_port, uint16_t rdy_pin)
{
    memset(&ad7784, 0, sizeof(ad7784));

    ad7784.hspi     = hspi;
    ad7784.rdy_port = rdy_port;
    ad7784.rdy_pin  = rdy_pin;
    ad7784.pcb = cb;

    AD7784_Emu_Reset();

    /* Stała wartość danych – możesz zmienić */
    AD7784_Emu_SetData(0x123456);

    ad7784.conversion_period_ms = 50;   /* 20 Hz – dostosuj do Mode */

    /* Start pierwszego odbioru (1 bajt Communications) */
    HAL_SPI_TransmitReceive_IT(ad7784.hspi, ad7784.tx_buf, ad7784.rx_buf, 1);
}

void AD7784_Emu_Reset(void)
{
    ad7784.status    = AD7784_DEFAULT_STATUS;
    ad7784.mode      = AD7784_DEFAULT_MODE;
    ad7784.config    = AD7784_DEFAULT_CONFIG;
    ad7784.id        = AD7784_DEFAULT_ID;
    ad7784.io        = AD7784_DEFAULT_IO;
    ad7784.offset    = AD7784_DEFAULT_OFFSET;
    ad7784.fullscale = 0x5A5A5A;   /* przykładowa wartość */

    ad7784.data_ready = false;
    ad7784.cread      = 0;
    update_rdy_pin();
}

void AD7784_Emu_SetData(uint32_t value_24bit)
{
    ad7784.data = value_24bit & 0xFFFFFF;
}

void AD7784_Emu_Process(void)
{
    uint32_t now = HAL_GetTick();

    if ((now - ad7784.last_conversion_tick) >= ad7784.conversion_period_ms) {
        ad7784.last_conversion_tick = now;

        /* Nowa „konwersja” – stała wartość już ustawiona */
        ad7784.status &= ~AD7784_STATUS_RDY;   /* RDY = 0 → dane gotowe */
        ad7784.data_ready = true;
        update_rdy_pin();
    }
}

/* ===== Callbacki SPI ===== */
void AD7784_Emu_SPI_RxTxCplt(void)
{
    if (ad7784.bytes_to_xfer == 0) {
        /* To był bajt Communications Register */
        process_communications_register(ad7784.rx_buf[0]);

        if (ad7784.is_read) {
            /* Wysyłamy dane rejestru */
            HAL_SPI_TransmitReceive_IT(ad7784.hspi,
                                       ad7784.tx_buf,
                                       ad7784.rx_buf,
                                       ad7784.bytes_to_xfer);
        } else if (ad7784.bytes_to_xfer > 0) {
            /* Odbieramy dane do zapisu */
            HAL_SPI_TransmitReceive_IT(ad7784.hspi,
                                       ad7784.tx_buf,
                                       ad7784.rx_buf,
                                       ad7784.bytes_to_xfer);
        } else {
            /* Nic do zrobienia – czekamy na następny Communications */
            HAL_SPI_TransmitReceive_IT(ad7784.hspi, ad7784.tx_buf, ad7784.rx_buf, 1);
        }
    } else {
    	//cb_pusha(ad7784.pcb, ad7784.rx_buf, ad7784.bytes_to_xfer);
        /* Zakończyliśmy transfer rejestru */
        if (!ad7784.is_read) {
            process_write_data();
        }

        /* Continuous Read? */
        if (ad7784.cread && ad7784.next_reg == AD7784_REG_DATA) {
            prepare_tx_buffer();
            HAL_SPI_TransmitReceive_IT(ad7784.hspi,
                                       ad7784.tx_buf,
                                       ad7784.rx_buf,
                                       ad7784.bytes_to_xfer);
        } else {
            /* Wracamy do oczekiwania na Communications */
            ad7784.bytes_to_xfer = 0;
            HAL_SPI_TransmitReceive_IT(ad7784.hspi, ad7784.tx_buf, ad7784.rx_buf, 1);
        }
    }
}

void AD7784_Emu_SPI_Error(void)
{
    /* Po błędzie restartujemy odbiór */
    ad7784.bytes_to_xfer = 0;
    HAL_SPI_TransmitReceive_IT(ad7784.hspi, ad7784.tx_buf, ad7784.rx_buf, 1);
}

/* Opcjonalnie – podłącz do EXTI na CS (falling) */
void AD7784_Emu_CS_Falling(void)
{
    /* Można tu zresetować stan maszyny jeśli CS spadnie w trakcie */
}
