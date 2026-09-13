/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ee_emul.h"
#include "ad7794_emu.h"
#include "CircularBuffer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
#define SPI_RX_BUFFER_SIZE 16

volatile uint8_t spi_rx_buffer[SPI_RX_BUFFER_SIZE];
volatile uint16_t spi_rx_index = 0;
volatile uint8_t spi_rx_complete = 0;
extern volatile ActiveDevice_t active_device;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line 0 and 1 interrupts.
  */
void EXTI0_1_IRQHandler(void)
{
    /* CS_ADC - EXTI line 0 */
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_0))
    {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_0);

        if (LL_GPIO_IsInputPinSet(CS_ADC_GPIO_Port, CS_ADC_Pin))
        {
            /* CS_ADC = HIGH -> koniec transakcji AD7794 */
            AD7794_Emu_CS_Deactivate();
        }
        else
        {
            /* CS_ADC = LOW -> początek transakcji AD7794 */
            //AD7794_Emu_CS_Activate();
        }
    }

    /* CS_EE - EXTI line 1 */
    if (LL_EXTI_IsActiveFlag_0_31(LL_EXTI_LINE_1))
    {
        LL_EXTI_ClearFlag_0_31(LL_EXTI_LINE_1);

        if (LL_GPIO_IsInputPinSet(CS_EE_GPIO_Port, CS_EE_Pin))
        {
            /* CS_EE = HIGH -> koniec transakcji EEPROM */
            EE_Emul_CS_Deactivate();
        }
        else
        {
            /* CS_EE = LOW -> początek transakcji EEPROM */
            //EE_Emul_CS_Activate();
        }
    }
}

/**
  * @brief This function handles SPI1 global interrupt.
  */
void SPI1_IRQHandler(void)
{
    uint8_t rx;

    if (LL_SPI_IsActiveFlag_RXNE(SPI1))
    {
        rx = LL_SPI_ReceiveData8(SPI1);
        if (!LL_GPIO_IsInputPinSet(CS_EE_GPIO_Port, CS_EE_Pin))
        {
            EE_Emul_SPI_RxTx(rx);
            cb_push('e');cb_push(rx);
        }
        else if (!LL_GPIO_IsInputPinSet(CS_ADC_GPIO_Port, CS_ADC_Pin))
        {
        	AD7794_Emul_SPI_RxTx(rx);
        	cb_push('a');cb_push(rx);
        }
        else
        {
            if (LL_SPI_IsActiveFlag_TXE(SPI1))
            {
                LL_SPI_TransmitData8(SPI1, 0xFF);
            }
        }
    }

    /*
     * Clear OVR.
    if (LL_SPI_IsActiveFlag_OVR(SPI1))
    {
        (void)LL_SPI_ReceiveData8(SPI1);
        (void)SPI1->SR;
        cb_push('!');
    }
    */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
