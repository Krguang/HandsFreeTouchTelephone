/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define touch0_Pin GPIO_PIN_13
#define touch0_GPIO_Port GPIOC
#define touch_jing_Pin GPIO_PIN_14
#define touch_jing_GPIO_Port GPIOC
#define relay1_Pin GPIO_PIN_15
#define relay1_GPIO_Port GPIOC
#define relay2_Pin GPIO_PIN_0
#define relay2_GPIO_Port GPIOD
#define relay3_Pin GPIO_PIN_1
#define relay3_GPIO_Port GPIOD
#define relay4_Pin GPIO_PIN_0
#define relay4_GPIO_Port GPIOA
#define relay5_Pin GPIO_PIN_1
#define relay5_GPIO_Port GPIOA
#define relay6_Pin GPIO_PIN_2
#define relay6_GPIO_Port GPIOA
#define dj_Pin GPIO_PIN_3
#define dj_GPIO_Port GPIOA
#define mt_Pin GPIO_PIN_4
#define mt_GPIO_Port GPIOA
#define vol_down_Pin GPIO_PIN_5
#define vol_down_GPIO_Port GPIOA
#define vol_up_Pin GPIO_PIN_6
#define vol_up_GPIO_Port GPIOA
#define bjyy_Pin GPIO_PIN_7
#define bjyy_GPIO_Port GPIOA
#define mute_Pin GPIO_PIN_0
#define mute_GPIO_Port GPIOB
#define led1_Pin GPIO_PIN_1
#define led1_GPIO_Port GPIOB
#define led2_Pin GPIO_PIN_2
#define led2_GPIO_Port GPIOB
#define led3_Pin GPIO_PIN_10
#define led3_GPIO_Port GPIOB
#define led4_Pin GPIO_PIN_11
#define led4_GPIO_Port GPIOB
#define led5_Pin GPIO_PIN_12
#define led5_GPIO_Port GPIOB
#define led6_Pin GPIO_PIN_13
#define led6_GPIO_Port GPIOB
#define hfi_Pin GPIO_PIN_14
#define hfi_GPIO_Port GPIOB
#define tel_data_Pin GPIO_PIN_15
#define tel_data_GPIO_Port GPIOB
#define tel_clk_Pin GPIO_PIN_8
#define tel_clk_GPIO_Port GPIOA
#define tel_ce_Pin GPIO_PIN_9
#define tel_ce_GPIO_Port GPIOA
#define dj_ext_Pin GPIO_PIN_10
#define dj_ext_GPIO_Port GPIOA
#define touch1_Pin GPIO_PIN_11
#define touch1_GPIO_Port GPIOA
#define touch2_Pin GPIO_PIN_12
#define touch2_GPIO_Port GPIOA
#define touch3_Pin GPIO_PIN_15
#define touch3_GPIO_Port GPIOA
#define touch4_Pin GPIO_PIN_3
#define touch4_GPIO_Port GPIOB
#define touch5_Pin GPIO_PIN_4
#define touch5_GPIO_Port GPIOB
#define touch6_Pin GPIO_PIN_5
#define touch6_GPIO_Port GPIOB
#define touch7_Pin GPIO_PIN_6
#define touch7_GPIO_Port GPIOB
#define touch8_Pin GPIO_PIN_7
#define touch8_GPIO_Port GPIOB
#define touch9_Pin GPIO_PIN_8
#define touch9_GPIO_Port GPIOB
#define touch_xing_Pin GPIO_PIN_9
#define touch_xing_GPIO_Port GPIOB

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
