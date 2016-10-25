/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include <string.h>

#define  WAIT_HOST    0
#define IDLE        1
#define PAGE_PROG    2

typedef void (*pFunction)(void);

// Flash configuration
#define MAIN_PROGRAM_START_ADDRESS              (uint32_t)0x08002000
#define MAIN_PROGRAM_PAGE_NUMBER                8
#define NUM_OF_PAGES                            (64 - MAIN_PROGRAM_PAGE_NUMBER)

// CAN identifiers
#define DEVICE_CAN_ID                            0x78E
#define CMD_HOST_INIT                            0x01
#define CMD_PAGE_PROG                            0x02
#define CMD_BOOT                                0x03

#define CAN_RESP_OK                              0x01
#define CAN_RESP_ERROR                          0x02

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

CRC_HandleTypeDef hcrc;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
static CanTxMsgTypeDef        canTxMessage;
static CanRxMsgTypeDef        canRxMessage;
static FLASH_EraseInitTypeDef eraseInitStruct;

pFunction                     JumpAddress;
uint8_t                       PageBuffer[FLASH_PAGE_SIZE];
volatile int                  PageBufferPtr;
uint8_t                       PageIndex;
int                           PageCRC;

volatile uint8_t              blState;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_CRC_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void JumpToApplication()
{
  JumpAddress = *(__IO pFunction*)(MAIN_PROGRAM_START_ADDRESS + 4);
  __set_MSP(*(__IO uint32_t*) MAIN_PROGRAM_START_ADDRESS);
  HAL_DeInit();
  JumpAddress();
}

void TransmitResponsePacket(uint8_t response)
{
  hcan.pTxMsg->StdId = DEVICE_CAN_ID;
  hcan.pTxMsg->DLC = 1;
  hcan.pTxMsg->Data[0] = response;
  HAL_CAN_Transmit_IT(&hcan);
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* CanHandle)
{
  // Skip messages not intended for our device
  if (CanHandle->pRxMsg->StdId != DEVICE_CAN_ID) {
    HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);
    return;
  }

  if (blState == PAGE_PROG)
  {
    memcpy(&PageBuffer[PageBufferPtr],
      CanHandle->pRxMsg->Data,
      CanHandle->pRxMsg->DLC);
    PageBufferPtr += CanHandle->pRxMsg->DLC;

    if (PageBufferPtr == FLASH_PAGE_SIZE) {
      HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);

      HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
      uint32_t crc = HAL_CRC_Calculate(&hcrc, (uint32_t*)PageBuffer, FLASH_PAGE_SIZE / 4);

      if (crc == PageCRC && PageIndex <= NUM_OF_PAGES)
      {
        HAL_FLASH_Unlock();

        uint32_t PageError = 0;

        eraseInitStruct.TypeErase = TYPEERASE_PAGES;
        eraseInitStruct.PageAddress = MAIN_PROGRAM_START_ADDRESS + PageIndex * FLASH_PAGE_SIZE;
        eraseInitStruct.NbPages = 1;

        HAL_FLASHEx_Erase(&eraseInitStruct, &PageError);

        for (int i = 0; i < FLASH_PAGE_SIZE; i += 4)
        {
          HAL_FLASH_Program(TYPEPROGRAM_WORD, MAIN_PROGRAM_START_ADDRESS + PageIndex * FLASH_PAGE_SIZE + i, *(uint32_t*)&PageBuffer[i]);
        }

        HAL_FLASH_Lock();

        TransmitResponsePacket(CAN_RESP_OK);
      }
      else
      {
        TransmitResponsePacket(CAN_RESP_ERROR);
      }

      blState = IDLE;

      HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
    }

    HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
    HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);
    return;
  }

  switch(CanHandle->pRxMsg->Data[0])
  {
    case CMD_HOST_INIT:
      blState = IDLE;
      TransmitResponsePacket(CAN_RESP_OK);
      break;
    case CMD_PAGE_PROG:
      if (blState == IDLE) {
        memset(PageBuffer, 0, FLASH_PAGE_SIZE);
        memcpy(&PageCRC, &CanHandle->pRxMsg->Data[2], sizeof(int));
        PageIndex = CanHandle->pRxMsg->Data[1];
        blState = PAGE_PROG;
        PageBufferPtr = 0;
      } else {
        // Should never get here
      }
      break;
    case CMD_BOOT:
      TransmitResponsePacket(CAN_RESP_OK);
      JumpToApplication();
      break;
    default:
      break;
  }

  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_6);
  HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);
}
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_CRC_Init();

  /* USER CODE BEGIN 2 */
  hcan.pTxMsg = &canTxMessage;
  hcan.pRxMsg = &canRxMessage;

  CAN_FilterConfTypeDef canFilterConfig;
  canFilterConfig.FilterNumber = 0;
  canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canFilterConfig.FilterIdHigh = 0x0000;
  canFilterConfig.FilterIdLow = 0x0000;
  canFilterConfig.FilterMaskIdHigh = 0x0000 << 5;
  canFilterConfig.FilterMaskIdLow = 0x0000;
  canFilterConfig.FilterFIFOAssignment = 0;
  canFilterConfig.FilterActivation = ENABLE;
  canFilterConfig.BankNumber = 1;
  HAL_CAN_ConfigFilter(&hcan, &canFilterConfig);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

  HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);
  HAL_Delay(1000);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);

  // Timed out waiting for host
  if (blState == WAIT_HOST) {
    JumpToApplication();
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* CAN init function */
static void MX_CAN_Init(void)
{

  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_5TQ;
  hcan.Init.BS2 = CAN_BS2_6TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = DISABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = ENABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }

}

/* CRC init function */
static void MX_CRC_Init(void)
{

  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
