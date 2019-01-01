/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "adc.h"
#include "can.h"
#include "dac.h"
#include "i2c.h"
#include "spi.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

//#include "mycan.h"

#define BOARD_ID 2
#define NUM_REGISTERS 12

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


void init_max(int number) {

	uint8_t reg[] = {0x00,0x03,0xff,0x7f,0xc0,0x7f,0xff,0x80,0,0,0,0};

	for (int i = 0; i < NUM_REGISTERS; i++) {

		uint8_t write_reg[] = { i+0x80, reg[i] };
		uint8_t received_data[2];

		HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 0);
	    HAL_SPI_TransmitReceive(&hspi1, (uint8_t *) write_reg, (uint8_t *) received_data, 2, HAL_MAX_DELAY);
	    HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 1);
	    HAL_Delay(150);
	}

}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_CAN1_Init();
  MX_DAC_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();

  /* USER CODE BEGIN 2 */
 // Init_MyCAN();

  uint8_t test[2] = {0x01, 0};
  uint8_t start_conversion[2] = {0x80, 0b1000000};
  uint8_t received_data[2];

  uint8_t temp_address[5] = {0x0C, 0, 0, 0, 0};
  uint8_t received_temp[5] = {0};

  uint8_t reg_check[12] = {0, 0,0,0,0,0,0,0,0,0,0,0};
  uint8_t reg_vals[12] = {0};



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 1);
  HAL_GPIO_WritePin(CS2_GPIO_Port, CS2_Pin, 1);
  HAL_GPIO_WritePin(CS3_GPIO_Port, CS3_Pin, 1);
  HAL_GPIO_WritePin(CS4_GPIO_Port, CS4_Pin, 1);
  HAL_GPIO_WritePin(CS5_GPIO_Port, CS5_Pin, 1);
  HAL_GPIO_WritePin(CS6_GPIO_Port, CS6_Pin, 1);
  HAL_GPIO_WritePin(CS7_GPIO_Port, CS7_Pin, 1);
  HAL_GPIO_WritePin(CS8_GPIO_Port, CS8_Pin, 1);
  HAL_GPIO_WritePin(CS9_GPIO_Port, CS9_Pin, 1);
  HAL_GPIO_WritePin(CS10_GPIO_Port, CS10_Pin, 1);


  //init_max(1);

  while (1)
  {

	  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 0);
	  HAL_SPI_TransmitReceive(&hspi1, (uint8_t *) test, (uint8_t *) received_data, 2, HAL_MAX_DELAY);
	  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 1);
	  HAL_Delay(150);



	  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 0);
	  HAL_SPI_TransmitReceive(&hspi1, (uint8_t *) start_conversion, (uint8_t *) received_data, 2, HAL_MAX_DELAY);
	  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 1);
	  HAL_Delay(250);

	  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 0);
	  HAL_SPI_TransmitReceive(&hspi1, (uint8_t *) temp_address, (uint8_t *) received_temp, 5, HAL_MAX_DELAY);
	  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 1);
	  HAL_Delay(150);

	  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 0);
	  HAL_SPI_TransmitReceive(&hspi1, (uint8_t *) reg_check, (uint8_t *) reg_vals, 12, HAL_MAX_DELAY);
	  HAL_GPIO_WritePin(CS1_GPIO_Port, CS1_Pin, 1);
	  HAL_Delay(150);

	  int32_t temp24 = received_temp[1] << 16 | received_temp[2] << 8 | received_temp[3];
	  if (temp24 & 0x800000)
	  {
		  temp24 |= 0xFF000000; //fix sign
	  }

	  temp24 >>= 5;

	  float tempfloat = temp24;
	  tempfloat *= 0.0078125;
	  tempfloat += 273.15;

	  int sensorNum = 1;
/*	  uint16_t msgID = create_ID(BOARD_ID, sensorNum);
	  can_msg_t msg;

*/
	  uint16_t value = tempfloat*32;
/*	  CAN_short_msg(&msg, msgID, value);

	  while(!CAN_can_transmit()){};

	  CAN_queue_transmit(&msg);

*/
	  volatile int var = 0;

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

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
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
