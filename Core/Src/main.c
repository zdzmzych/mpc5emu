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
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CircularBuffer.h"
#include "ad7794_emu.h"
#include "ee_emul.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	DEV_IDLE = 0, DEV_EEPROM_ACTIVE, DEV_ADC_ACTIVE
} ActiveDevice_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SEND_CHAR_USART2(c) do { \
    while (!LL_USART_IsActiveFlag_TXE(USART2)); \
    LL_USART_TransmitData8(USART2, (c)); \
} while(0)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile ActiveDevice_t active_device = DEV_IDLE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Adc_Pin_Changed(void)
{
    if (LL_GPIO_IsInputPinSet(CS_ADC_GPIO_Port, CS_ADC_Pin))
    {
        /* CS HIGH */

        if (active_device == DEV_ADC_ACTIVE)
        {
            active_device = DEV_IDLE;
            AD7794_Emu_CS_Deactivate();
            cb_push(':');
        }
    }
    else
    {
    	 /* CS ADC LOW */
        if (active_device == DEV_EEPROM_ACTIVE)
        {
            active_device = DEV_IDLE;
            EE_Emul_CS_Deactivate();
            cb_push(',');
        }

        active_device = DEV_ADC_ACTIVE;
        cb_push('A');
        AD7794_Emu_CS_Activate();
    }
}

void Ee_Pin_Changed(void)
{
    if (LL_GPIO_IsInputPinSet(CS_EE_GPIO_Port, CS_EE_Pin))
    {
        /* CS EEPROM HIGH */

        if (active_device == DEV_EEPROM_ACTIVE)
        {
            EE_Emul_CS_Deactivate();
            active_device = DEV_IDLE;
            cb_push(';');
        }
    }
    else
    {
        /* CS EEPROM LOW */

        if (active_device == DEV_ADC_ACTIVE)
        {
            AD7794_Emu_CS_Deactivate();
            cb_push('.');
        }

        active_device = DEV_EEPROM_ACTIVE;
        cb_push('E');
        EE_Emul_CS_Activate();
    }
}

uint8_t mpc5_update_spi(uint8_t data)
{
	cb_push(data);
	if (active_device == DEV_ADC_ACTIVE)
	{
		AD7794_Emu_SPI_RxTxCplt(data);
	}
	else if (active_device == DEV_EEPROM_ACTIVE)
	{
		EE_Emul_SPI_RxTx(data);
	}
	return 0;
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

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  LL_SPI_Enable(SPI1);

  AD7794_Emu_Init();
  EE_Emul_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  AD7794_Emu_Process();
	  if(cb_pop(&cb))
	  {
		  LL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		  SEND_CHAR_USART2(cb);
	  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
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
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
