
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
#include "stm32f1xx_hal.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#define HT9200_CE_LOW		HAL_GPIO_WritePin(tel_ce_GPIO_Port,tel_ce_Pin,GPIO_PIN_RESET)
#define HT9200_CE_HIGH		HAL_GPIO_WritePin(tel_ce_GPIO_Port,tel_ce_Pin,GPIO_PIN_SET)

#define HT9200_DATA_LOW		HAL_GPIO_WritePin(tel_data_GPIO_Port,tel_data_Pin,GPIO_PIN_RESET)
#define HT9200_DATA_HIGH	HAL_GPIO_WritePin(tel_data_GPIO_Port,tel_data_Pin,GPIO_PIN_SET)

#define HT9200_CLK_LOW		HAL_GPIO_WritePin(tel_clk_GPIO_Port,tel_clk_Pin,GPIO_PIN_RESET)
#define HT9200_CLK_HIGH		HAL_GPIO_WritePin(tel_clk_GPIO_Port,tel_clk_Pin,GPIO_PIN_SET)

#define HT9200_MUTE_HIGH	HAL_GPIO_WritePin(mute_GPIO_Port,mute_Pin,GPIO_PIN_SET)
#define HT9200_MUTE_LOW		HAL_GPIO_WritePin(mute_GPIO_Port,mute_Pin,GPIO_PIN_RESET)
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

uint8_t bjyyFlag = 0;
int8_t bjyyValue = 4;
uint16_t count;
uint8_t muteFlag;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void keyScan(void);
void sendOneBit(uint8_t dtmfData);

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
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  keyScan();
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

	if (htim->Instance == TIM4)//tim4 1ms
	{

		if (muteFlag == 1) {
			count++;
		}


		if (count >4000) {
			muteFlag = 0;
			count = 0;
			//	HAL_GPIO_TogglePin(led3_GPIO_Port,led3_Pin);
			HT9200_MUTE_LOW;

		}
	}
}

void sendOneBit(uint8_t dtmfData) {

	muteFlag = 1;
	HT9200_MUTE_HIGH;
	HT9200_CE_LOW;
	HAL_Delay(20);
	for (int i = 0; i<5; i++) {
		HT9200_CLK_HIGH;
		HAL_Delay(4);
		if (dtmfData & 0x01) {
			HT9200_DATA_HIGH;
			HAL_Delay(4);
		}
		else {
			HT9200_DATA_LOW;
			HAL_Delay(4);
		}
		HT9200_CLK_LOW;
		HAL_Delay(4);
		HT9200_CLK_HIGH;
		HAL_Delay(4);
		dtmfData >>= 1;
	}
	HAL_Delay(100);
	HT9200_CE_HIGH;
	HT9200_DATA_HIGH;
	HAL_Delay(10);
}


void keyScan() {

	/*¶Ô½²*/
	if (HAL_GPIO_ReadPin(dj_GPIO_Port,dj_Pin) == 0)
	{
		HAL_GPIO_WritePin(dj_ext_GPIO_Port, dj_ext_Pin, GPIO_PIN_SET);
	}
	else {
		HAL_GPIO_WritePin(dj_ext_GPIO_Port, dj_ext_Pin, GPIO_PIN_RESET);
	}



	/*ÃâÌá*/
	if (HAL_GPIO_ReadPin(mt_GPIO_Port, mt_Pin) == 0) {
		HAL_Delay(20);
		if (HAL_GPIO_ReadPin(mt_GPIO_Port, mt_Pin) == 0) {
			HAL_Delay(20);
			while (HAL_GPIO_ReadPin(mt_GPIO_Port, mt_Pin) == 0) {}
			HAL_GPIO_TogglePin(hfi_GPIO_Port, hfi_Pin);
		}
	}

	/*±³¾°ÒôÀÖ*/

	if (HAL_GPIO_ReadPin(bjyy_GPIO_Port, bjyy_Pin) == 0) {
		HAL_Delay(20);
		if (HAL_GPIO_ReadPin(bjyy_GPIO_Port, bjyy_Pin) == 0) {
			HAL_Delay(20);
			while (HAL_GPIO_ReadPin(bjyy_GPIO_Port, bjyy_Pin) == 0) {}
			
			if (bjyyFlag == 0)
			{
				bjyyFlag = 1;
			}
			else
			{
				bjyyFlag = 0;
			}

		}
	}


	if (1 == bjyyFlag)
	{
		if (HAL_GPIO_ReadPin(vol_up_GPIO_Port, vol_up_Pin) == 0) {
			HAL_Delay(20);
			if (HAL_GPIO_ReadPin(vol_up_GPIO_Port, vol_up_Pin) == 0) {
				HAL_Delay(20);
				while (HAL_GPIO_ReadPin(vol_up_GPIO_Port, vol_up_Pin) == 0) {}

				bjyyValue++;
				if (bjyyValue > 4)
				{
					bjyyValue = 4;
				}
			}
		}

		if (HAL_GPIO_ReadPin(vol_down_GPIO_Port, vol_down_Pin) == 0) {
			HAL_Delay(20);
			if (HAL_GPIO_ReadPin(vol_down_GPIO_Port, vol_down_Pin) == 0) {
				HAL_Delay(20);
				while (HAL_GPIO_ReadPin(vol_down_GPIO_Port, vol_down_Pin) == 0) {}

				bjyyValue--;
				if (bjyyValue < 1)
				{
					bjyyValue = 1;
				}
			}
		}

	}

	/*ÏÔÊ¾ÒôÁ¿*/

	if (1 == bjyyFlag)
	{
		switch (bjyyValue)
		{
		case 0: HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(led3_GPIO_Port, led3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_SET);

			HAL_GPIO_WritePin(relay1_GPIO_Port, relay1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(relay2_GPIO_Port, relay2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(relay3_GPIO_Port, relay3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(relay4_GPIO_Port, relay4_Pin, GPIO_PIN_RESET);
			break;

		case 1: HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(led3_GPIO_Port, led3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_SET);

			HAL_GPIO_WritePin(relay1_GPIO_Port, relay1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(relay2_GPIO_Port, relay2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(relay3_GPIO_Port, relay3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(relay4_GPIO_Port, relay4_Pin, GPIO_PIN_SET);
			break;

		case 2: HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(led3_GPIO_Port, led3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_SET);

			HAL_GPIO_WritePin(relay1_GPIO_Port, relay1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(relay2_GPIO_Port, relay2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(relay3_GPIO_Port, relay3_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(relay4_GPIO_Port, relay4_Pin, GPIO_PIN_RESET);
			break;

		case 3: HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(led3_GPIO_Port, led3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_SET);

			HAL_GPIO_WritePin(relay1_GPIO_Port, relay1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(relay2_GPIO_Port, relay2_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(relay3_GPIO_Port, relay3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(relay4_GPIO_Port, relay4_Pin, GPIO_PIN_RESET);
			break;

		case 4: HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(led3_GPIO_Port, led3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_RESET);

			HAL_GPIO_WritePin(relay1_GPIO_Port, relay1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(relay2_GPIO_Port, relay2_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(relay3_GPIO_Port, relay3_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(relay4_GPIO_Port, relay4_Pin, GPIO_PIN_RESET);
			break;

		}
	}
	else
	{
		HAL_GPIO_WritePin(led1_GPIO_Port, led1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(led2_GPIO_Port, led2_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(led3_GPIO_Port, led3_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(led4_GPIO_Port, led4_Pin, GPIO_PIN_SET);

		HAL_GPIO_WritePin(relay1_GPIO_Port, relay1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(relay2_GPIO_Port, relay2_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(relay3_GPIO_Port, relay3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(relay4_GPIO_Port, relay4_Pin, GPIO_PIN_RESET);
	}

	


	/*²¦ºÅ*/
	if (HAL_GPIO_ReadPin(touch0_GPIO_Port, touch0_Pin) == 0) {
		HAL_Delay(20);
		if (HAL_GPIO_ReadPin(touch0_GPIO_Port, touch0_Pin) == 0) {
			HAL_Delay(20);
			while (HAL_GPIO_ReadPin(touch0_GPIO_Port, touch0_Pin) == 0) {}
			//HAL_GPIO_TogglePin(led3_GPIO_Port,led3_Pin);
			sendOneBit(0X0A);

		}
	}

	if (HAL_GPIO_ReadPin(touch1_GPIO_Port, touch1_Pin) == 0) {
		HAL_Delay(20);
		if (HAL_GPIO_ReadPin(touch1_GPIO_Port, touch1_Pin) == 0) {
			HAL_Delay(20);
			while (HAL_GPIO_ReadPin(touch1_GPIO_Port, touch1_Pin) == 0) {}
			//HAL_GPIO_TogglePin(led3_GPIO_Port,led3_Pin);
			sendOneBit(1);

		}
	}

	if (HAL_GPIO_ReadPin(touch2_GPIO_Port, touch2_Pin) == 0) {
		HAL_Delay(20);
		if (HAL_GPIO_ReadPin(touch2_GPIO_Port, touch2_Pin) == 0) {
			HAL_Delay(20);
			while (HAL_GPIO_ReadPin(touch2_GPIO_Port, touch2_Pin) == 0) {}
			//HAL_GPIO_TogglePin(led3_GPIO_Port,led3_Pin);
			sendOneBit(2);

		}
	}

	if (HAL_GPIO_ReadPin(touch3_GPIO_Port, touch3_Pin) == 0) {
		HAL_Delay(20);
		if (HAL_GPIO_ReadPin(touch3_GPIO_Port, touch3_Pin) == 0) {
			HAL_Delay(20);
			while (HAL_GPIO_ReadPin(touch3_GPIO_Port, touch3_Pin) == 0) {}
			//	HAL_GPIO_TogglePin(led3_GPIO_Port,led3_Pin);
			sendOneBit(3);

		}
	}

	if (HAL_GPIO_ReadPin(touch4_GPIO_Port, touch4_Pin) == 0) {
		HAL_Delay(20);
		if (HAL_GPIO_ReadPin(touch4_GPIO_Port, touch4_Pin) == 0) {
			HAL_Delay(20);
			while (HAL_GPIO_ReadPin(touch4_GPIO_Port, touch4_Pin) == 0) {}
			//	HAL_GPIO_TogglePin(led3_GPIO_Port,led3_Pin);
			sendOneBit(4);

		}
	}

	if (HAL_GPIO_ReadPin(touch5_GPIO_Port, touch5_Pin) == 0) {
		HAL_Delay(20);
		if (HAL_GPIO_ReadPin(touch5_GPIO_Port, touch5_Pin) == 0) {
			HAL_Delay(20);
			while (HAL_GPIO_ReadPin(touch5_GPIO_Port, touch5_Pin) == 0) {}
			//	HAL_GPIO_TogglePin(led3_GPIO_Port,led3_Pin);
			sendOneBit(5);

		}
	}

	if (HAL_GPIO_ReadPin(touch6_GPIO_Port, touch6_Pin) == 0) {
		HAL_Delay(20);
		if (HAL_GPIO_ReadPin(touch6_GPIO_Port, touch6_Pin) == 0) {
			HAL_Delay(20);
			while (HAL_GPIO_ReadPin(touch6_GPIO_Port, touch6_Pin) == 0) {}
			//	HAL_GPIO_TogglePin(led3_GPIO_Port,led3_Pin);
			sendOneBit(6);

		}
	}

	if (HAL_GPIO_ReadPin(touch7_GPIO_Port, touch7_Pin) == 0) {
		HAL_Delay(20);
		if (HAL_GPIO_ReadPin(touch7_GPIO_Port, touch7_Pin) == 0) {
			HAL_Delay(20);
			while (HAL_GPIO_ReadPin(touch7_GPIO_Port, touch7_Pin) == 0) {}
			//	HAL_GPIO_TogglePin(led3_GPIO_Port,led3_Pin);
			sendOneBit(7);

		}
	}

	if (HAL_GPIO_ReadPin(touch8_GPIO_Port, touch8_Pin) == 0) {
		HAL_Delay(20);
		if (HAL_GPIO_ReadPin(touch8_GPIO_Port, touch8_Pin) == 0) {
			HAL_Delay(20);
			while (HAL_GPIO_ReadPin(touch8_GPIO_Port, touch8_Pin) == 0) {}
			//		HAL_GPIO_TogglePin(led3_GPIO_Port,led3_Pin);
			sendOneBit(8);

		}
	}

	if (HAL_GPIO_ReadPin(touch9_GPIO_Port, touch9_Pin) == 0) {
		HAL_Delay(20);
		if (HAL_GPIO_ReadPin(touch9_GPIO_Port, touch9_Pin) == 0) {
			HAL_Delay(20);
			while (HAL_GPIO_ReadPin(touch9_GPIO_Port, touch9_Pin) == 0) {}
			//		HAL_GPIO_TogglePin(led3_GPIO_Port,led3_Pin);
			sendOneBit(9);

		}
	}

	if (HAL_GPIO_ReadPin(touch_xing_GPIO_Port, touch_xing_Pin) == 0) {
		HAL_Delay(20);
		if (HAL_GPIO_ReadPin(touch_xing_GPIO_Port, touch_xing_Pin) == 0) {
			HAL_Delay(20);
			while (HAL_GPIO_ReadPin(touch_xing_GPIO_Port, touch_xing_Pin) == 0) {}
			//	HAL_GPIO_TogglePin(led3_GPIO_Port,led3_Pin);
			sendOneBit(0x0b);
		}
	}

	if (HAL_GPIO_ReadPin(touch_jing_GPIO_Port, touch_jing_Pin) == 0) {
		HAL_Delay(20);
		if (HAL_GPIO_ReadPin(touch_jing_GPIO_Port, touch_jing_Pin) == 0) {
			HAL_Delay(20);
			while (HAL_GPIO_ReadPin(touch_jing_GPIO_Port, touch_jing_Pin) == 0) {}
			//	HAL_GPIO_TogglePin(led3_GPIO_Port,led3_Pin);
			sendOneBit(0x0c);
		}
	}

}



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
