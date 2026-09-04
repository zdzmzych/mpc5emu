/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"


/* USER CODE BEGIN Includes */

#include "CircularBuffer.h"
#include "ad7794_emu.h"
#include "ee_emul.h"

/* USER CODE END Includes */


/* USER CODE BEGIN PTD */

typedef enum
{
    DEV_IDLE = 0,
    DEV_EEPROM_ACTIVE,
    DEV_ADC_ACTIVE

} ActiveDevice_t;

/* USER CODE END PTD */


/* USER CODE BEGIN PD */

#define SEND_CHAR_USART2(c)             \
    do                                  \
    {                                   \
        while (!LL_USART_IsActiveFlag_TXE(USART2)) \
        {                               \
        }                               \
        LL_USART_TransmitData8(USART2, (c)); \
    } while (0)

/* USER CODE END PD */


/* USER CODE BEGIN PV */

volatile ActiveDevice_t active_device = DEV_IDLE;

/* USER CODE END PV */


/* Private function prototypes ----------------------------------------------- */

void SystemClock_Config(void);


/* USER CODE BEGIN 0 */


/*
 * ============================================================================
 * ADC CS change
 * ============================================================================
 */

void Adc_Pin_Changed(void)
{
    if (LL_GPIO_IsInputPinSet(CS_ADC_GPIO_Port, CS_ADC_Pin))
    {
        /*
         * ------------------------------------------------------------
         * CS ADC HIGH
         * ------------------------------------------------------------
         */

        if (active_device == DEV_ADC_ACTIVE)
        {
            AD7794_Emu_CS_Deactivate();

            active_device = DEV_IDLE;

            /*
             * Debug marker.
             */
            cb_push(':');
        }
    }
    else
    {
        /*
         * ------------------------------------------------------------
         * CS ADC LOW
         * ------------------------------------------------------------
         */

        /*
         * If EEPROM was active, terminate it.
         */
        if (active_device == DEV_EEPROM_ACTIVE)
        {
            EE_Emul_CS_Deactivate();

            cb_push(',');

            active_device = DEV_IDLE;
        }


        active_device = DEV_ADC_ACTIVE;

        /*
         * Debug marker.
         */
        cb_push('A');


        AD7794_Emu_CS_Activate();
    }
}


/*
 * ============================================================================
 * EEPROM CS change
 * ============================================================================
 */

void Ee_Pin_Changed(void)
{
    if (LL_GPIO_IsInputPinSet(CS_EE_GPIO_Port, CS_EE_Pin))
    {
        /*
         * ------------------------------------------------------------
         * CS EEPROM HIGH
         * ------------------------------------------------------------
         */

        if (active_device == DEV_EEPROM_ACTIVE)
        {
        	LL_GPIO_SetOutputPin(HLP_GPIO_Port, HLP_Pin);

            EE_Emul_CS_Deactivate();

            active_device = DEV_IDLE;

            /*
             * Debug marker.
             */
            cb_push(';');
        }
    }
    else
    {
        /*
         * ------------------------------------------------------------
         * CS EEPROM LOW
         * ------------------------------------------------------------
         */

        /*
         * If ADC was active, terminate it.
         */
        if (active_device == DEV_ADC_ACTIVE)
        {
            //AD7794_Emu_CS_Deactivate();

            cb_push('.');

            active_device = DEV_IDLE;
        }


        active_device = DEV_EEPROM_ACTIVE;


        /*
         * HLP indicates EEPROM active.
         */
        LL_GPIO_ResetOutputPin(HLP_GPIO_Port, HLP_Pin);


        /*
         * Debug marker.
         */
        cb_push('E');


        /*
         * Initialize EEPROM SPI transaction.
         *
         * This also preloads the first dummy byte into TX.
         */
        EE_Emul_CS_Activate();
    }
}


/*
 * ============================================================================
 * SPI byte processing
 * ============================================================================
 *
 * Called after one complete SPI byte has been received.
 *
 * The return value is the byte which must be transmitted during the
 * NEXT SPI transfer.
 *
 * ============================================================================
 */

uint8_t mpc5_update_spi(uint8_t data)
{
    uint8_t tx = 0x00;


    /*
     * Debug:
     *
     * Store received MOSI byte.
     */
    cb_push(data);


    /*
     * ------------------------------------------------------------
     * ADC
     * ------------------------------------------------------------
     */

    if (active_device == DEV_ADC_ACTIVE)
    {
        /*
         * AD7794 emulator prepares the next TX byte itself.
         */
        AD7794_Emu_SPI_RxTxCplt(data);

        return tx;
    }


    /*
     * ------------------------------------------------------------
     * EEPROM
     * ------------------------------------------------------------
     */

    if (active_device == DEV_EEPROM_ACTIVE)
    {
        tx = EE_Emul_SPI_RxTx(data);
        while(!LL_SPI_IsActiveFlag_TXE(SPI1));
        LL_SPI_TransmitData8(SPI1, tx);
        return tx;
    }


    /*
     * ------------------------------------------------------------
     * No active device
     * ------------------------------------------------------------
     */

    return 0x00;
}


/* USER CODE END 0 */


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */

    uint8_t cb;

    /* USER CODE END 1 */


    /*
     * MCU Configuration
     */

    HAL_Init();


    /* USER CODE BEGIN Init */

    /* USER CODE END Init */


    /*
     * Configure the system clock.
     */

    SystemClock_Config();


    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */


    /*
     * Initialize all configured peripherals.
     */

    MX_GPIO_Init();
    MX_SPI1_Init();
    MX_USART2_UART_Init();


    /* USER CODE BEGIN 2 */

    LL_GPIO_SetOutputPin(HLP_GPIO_Port, HLP_Pin);

    LL_SPI_Enable(SPI1);
    //AD7794_Emu_Init();
    EE_Emul_Init();
    /* USER CODE END 2 */


    /*
     * Infinite loop
     */

    while (1)
    {
        /*
         * ADC background processing.
         */
        //AD7794_Emu_Process();


        /*
         * EEPROM currently has no background processing,
         * but keeping the call makes the architecture consistent.
         */
        //EE_Emul_Process();


        /*
         * Send debug data through USART2.
         */
        if (cb_pop(&cb))
        {
            LL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            SEND_CHAR_USART2(cb);
        }
    }
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};


    /*
     * Initializes the RCC Oscillators.
     */

    RCC_OscInitStruct.OscillatorType =
        RCC_OSCILLATORTYPE_HSE;

    RCC_OscInitStruct.HSEState =
        RCC_HSE_BYPASS;

    RCC_OscInitStruct.PLL.PLLState =
        RCC_PLL_ON;

    RCC_OscInitStruct.PLL.PLLSource =
        RCC_PLLSOURCE_HSE;

    RCC_OscInitStruct.PLL.PLLMUL =
        RCC_PLL_MUL6;

    RCC_OscInitStruct.PLL.PREDIV =
        RCC_PREDIV_DIV1;


    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }


    /*
     * Initializes the CPU, AHB and APB buses clocks.
     */

    RCC_ClkInitStruct.ClockType =
        RCC_CLOCKTYPE_HCLK |
        RCC_CLOCKTYPE_SYSCLK |
        RCC_CLOCKTYPE_PCLK1;

    RCC_ClkInitStruct.SYSCLKSource =
        RCC_SYSCLKSOURCE_PLLCLK;

    RCC_ClkInitStruct.AHBCLKDivider =
        RCC_SYSCLK_DIV1;

    RCC_ClkInitStruct.APB1CLKDivider =
        RCC_HCLK_DIV1;


    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct,
                            FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }


    PeriphClkInit.PeriphClockSelection =
        RCC_PERIPHCLK_USART2;

    PeriphClkInit.Usart2ClockSelection =
        RCC_USART2CLKSOURCE_PCLK1;


    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
}


/* USER CODE BEGIN 4 */

/* USER CODE END 4 */


/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    __disable_irq();

    while (1)
    {
    }
}


#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line
  *         where the assert_param error has occurred.
  */
void assert_failed(uint8_t *file, uint32_t line)
{
    /*
     * User implementation.
     */
}

#endif
