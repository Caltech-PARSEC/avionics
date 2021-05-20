
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2021 STMicroelectronics
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
#include "can.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

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

// Structs for containing CAN message information
CAN_RxHeaderTypeDef baseCanRxHead;
CAN_TxHeaderTypeDef baseCanTxHead;

// The port number portion of the CAN ID corresponding to the message
// send for the data of each of the sensors, shifted so that the
// field is properly positioned within the CAN ID.
const int portNum[NUM_SENSORS] = {
    0 << PORT_NUM_SHIFT,
    1 << PORT_NUM_SHIFT,
    2 << PORT_NUM_SHIFT,
    3 << PORT_NUM_SHIFT,
    4 << PORT_NUM_SHIFT,
    5 << PORT_NUM_SHIFT,
    6 << PORT_NUM_SHIFT,
    7 << PORT_NUM_SHIFT,
    8 << PORT_NUM_SHIFT
};

// Expected to be used by the timer 3 ISR to determine which of the
// channels to average and send data for over the CAN bus.
//static int nextIdxToSend = 0;

// Chip select pins for each thermocouple
const uint16_t csPins[NUM_SENSORS] = {
  CS1_Pin, 
  CS2_Pin, 
  CS3_Pin, 
  CS4_Pin, 
  CS5_Pin, 
  CS6_Pin, 
  CS7_Pin, 
  CS8_Pin, 
  CS9_Pin, 
  CS10_Pin
};

// GPIO Port of each chip select pin
static GPIO_TypeDef* csPinPorts[NUM_SENSORS] = {
  CS1_GPIO_Port,
  CS2_GPIO_Port,
  CS3_GPIO_Port,
  CS4_GPIO_Port,
  CS5_GPIO_Port,
  CS6_GPIO_Port,
  CS7_GPIO_Port,
  CS8_GPIO_Port,
  CS9_GPIO_Port,
  CS10_GPIO_Port
};

// Buffer for each thermocouple's 3 byte reading
static uint8_t tempDataBuffer[NUM_SENSORS][3];
static uint8_t cjDataBuffer[2];
//static int tempDataReady[NUM_SENSORS] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


static uint8_t tempDataAddr[] = {0x0C};
static uint8_t cjDataAddr[] = {0x0A};
static uint8_t requestConv[2] = {0x80, 0b01000000};

// The number of thermocouple conversions triggered in
// the time it takes for a single conversion to complete.
// For a 660ms max conversion time, if we trigger a conversion
// every 100 ms, this value is ceiling(6.6) = 7
static int trigToReadGap = 7;

int trigInd = 0;
int readInd;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
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
  MX_CAN1_Init();
  MX_SPI1_Init();
  MX_TIM3_Init();

  /* USER CODE BEGIN 2 */
  // Start the IRQ timer on which data is sent over CAN
  HAL_TIM_Base_Start_IT(&htim3);


  // Initialize the CAN peripheral
  HAL_CAN_Init(&hcan1);

  // Configure CAN hardware receive filters
  //CAN_filterConfig();

  // Start the CAN module
  HAL_CAN_Start(&hcan1);


  // Start listening for CAN traffic on interrupt
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  // Set transmission parameters that are common to all Tx messages
  baseCanTxHead.IDE = CAN_ID_STD; // Standard 11-bit ID
  baseCanTxHead.StdId = OWN_CAN_ID; // CAN ID with port number = 0
  baseCanTxHead.RTR = CAN_RTR_DATA; // data frame (not a request frame)
  baseCanTxHead.DLC = 3;      // Number of data bytes


  // Just a buffer where AddTxMessage can return a code for which Tx
  // mailbox it added the message to. We don't care what it is.
  uint32_t mailbox;

  // Write all chip select pins low (inactive)
  for (int i = 0; i < NUM_SENSORS; i++)
  {
	  HAL_GPIO_WritePin(csPinPorts[i], csPins[i], 1);
  }

  // Initialize thermocouple signal processing ICs
  for (int i = 0; i < NUM_SENSORS; i++)
  {
	  // Write to configuration register 1 to average 16 samples per conversion,
	  // and ensure type K thermocouples are used.
	  uint8_t config1[2] = {0x81, 0b01110011};
	  HAL_GPIO_WritePin(csPinPorts[i], csPins[i], 0);
	  HAL_SPI_Transmit(&hspi1, (uint8_t *)config1, 2, HAL_MAX_DELAY);
	  HAL_GPIO_WritePin(csPinPorts[i], csPins[i], 1);
  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  // Trigger a one-shot conversion to thermocouple number `trigInd`
  HAL_GPIO_WritePin(csPinPorts[trigInd], csPins[trigInd], 0);
  HAL_SPI_Transmit(&hspi1, (uint8_t *) requestConv, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(csPinPorts[trigInd], csPins[trigInd], 1);

  // Read the value from a previously triggered conversion on a
  // different thermocouple.
  readInd = (trigInd - trigToReadGap) % NUM_SENSORS;
  if (readInd < 0) {readInd += NUM_SENSORS;}

  HAL_GPIO_WritePin(csPinPorts[readInd], csPins[readInd], 0);
  HAL_SPI_Transmit(&hspi1, (uint8_t *) tempDataAddr, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi1, &tempDataBuffer[readInd][0], 3, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(csPinPorts[readInd], csPins[readInd], 1);

  HAL_Delay(10);

  // For debugging, check what it's reading for cold junction temperature
  HAL_GPIO_WritePin(csPinPorts[readInd], csPins[readInd], 0);
  HAL_SPI_Transmit(&hspi1, (uint8_t *) cjDataAddr, 1, HAL_MAX_DELAY);
  HAL_SPI_Receive(&hspi1, (uint8_t *) cjDataBuffer, 2, HAL_MAX_DELAY);
  HAL_GPIO_WritePin(csPinPorts[readInd], csPins[readInd], 1);

  trigInd = (trigInd + 1) % NUM_SENSORS;


  int temp24 = tempDataBuffer[readInd][0] << 16 | \
                   tempDataBuffer[readInd][1] << 8 |  \
                   tempDataBuffer[readInd][2];

  int cjTemp = (cjDataBuffer[0] << 8) | cjDataBuffer[1];
  cjTemp = cjTemp & 0xFFFFFFFF;

//  if (temp24 & 0x800000)
//  {
//    temp24 |= 0xFF000000; //fix sign
//  }
//
//  temp24 >>= 5;
//
//  float tempfloat = temp24;
//  tempfloat *= 0.0078125;
//  tempfloat += 273.15;

  // Specify the contents of the message we're going to send.

  // Copy the base Tx message header into the header for this
  // message specifically.
  CAN_TxHeaderTypeDef msgCanTxHead =  baseCanTxHead;

  // Write the port ID for this sensor to the appropriate field
  // of the CAN ID.
  msgCanTxHead.StdId = OWN_CAN_ID | portNum[readInd];

  // Send the value for this sensor measurement over the CAN bus
  uint8_t canData[3];
  canData[2] = (temp24 >> 16) & 0xFF;
  canData[1] = (temp24 >> 8)  & 0xFF;
  canData[0] = temp24 & 0xFF;

  HAL_CAN_AddTxMessage(&hcan1, &msgCanTxHead, canData, &mailbox);

  HAL_Delay(100);


//  int sensorNum = 1;
///*    uint16_t msgID = create_ID(BOARD_ID, sensorNum);
//    can_msg_t msg;
//
//*/
//    uint16_t value = tempfloat*32;
///*    CAN_short_msg(&msg, msgID, value);
//
//    while(!CAN_can_transmit()){};
//
//    CAN_queue_transmit(&msg);
//
//*/
//    volatile int var = 0;
//
//  }
//  /* USER CODE END 3 */

  }
}

/**
  * @brief System Clock Configuration
  * @retval None
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
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
