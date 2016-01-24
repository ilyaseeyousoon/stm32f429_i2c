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
#include "stm32f4xx_hal.h"
#include "i2c.h"
#include "gpio.h"
#define MASTER_REQ_READ    0x12
#define MASTER_REQ_WRITE   0x34
/* Buffer used for transmission */
/* USER CODE BEGIN Includes */
uint8_t aTxBuffer[10];
/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))

/* Size of Transmission buffer */
#define TXBUFFERSIZE                      (COUNTOF(aTxBuffer) - 1)
/* Size of Reception buffer */
#define RXBUFFERSIZE                      TXBUFFERSIZE
uint8_t aRxBuffer[RXBUFFERSIZE];
uint16_t hTxNumData = 0, hRxNumData = 0;
uint8_t bTransferRequest = 0;

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

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
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	 HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t*)&bTransferRequest, 1);
  while (1)
  {
			 HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t*)&bTransferRequest, 1);
			    /* Initialize number of data variables */
//    hTxNumData = 0;
//    hRxNumData = 0;

//		
//		    /*##-2- Slave receive request from master ################################*/
//    HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t*)&bTransferRequest, 1);

  /* Initialize number of data variables */
    hTxNumData = 0;
    hRxNumData = 0;

    /*##-2- Slave receive request from master ################################*/
   
    
//    /*  Before starting a new communication transfer, you need to check the current
//    state of the peripheral; if itӳ busy you need to wait for the end of current
//    transfer before starting a new one.
//    For simplicity reasons, this example is just waiting till the end of the
//    transfer, but application may perform other tasks while transfer operation
//    is ongoing. */
//    while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
//    {
//    }

//    /* If master request write operation #####################################*/
//  
//      /*##-3- Slave receive number of data to be read ########################*/
//      while(HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t*)&hRxNumData, 2)!= HAL_OK);

//      /*  Before starting a new communication transfer, you need to check the current
//      state of the peripheral; if itӳ busy you need to wait for the end of current
//      transfer before starting a new one.
//      For simplicity reasons, this example is just waiting till the end of the
//      transfer, but application may perform other tasks while transfer operation
//      is ongoing. */
//      while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
//      {
//      }

//      /*##-4- Slave receives aRxBuffer from master ###########################*/
//      while(HAL_I2C_Slave_Receive_IT(&hi2c1, (uint8_t*)aRxBuffer, hRxNumData)!= HAL_OK);

//      /*  Before starting a new communication transfer, you need to check the current
//      state of the peripheral; if itӳ busy you need to wait for the end of current
//      transfer before starting a new one.
//      For simplicity reasons, this example is just waiting till the end of the
//      transfer, but application may perform other tasks while transfer operation
//      is ongoing. */
//      while (HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY)
//      {
//      }

  
    
    /* If master request write operation #####################################*/
  
		
		 
		
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

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 190;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV4;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
