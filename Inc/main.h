/**
  ******************************************************************************
  * File Name          : main.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * Copyright (c) 2020 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define configUSE_STATS_FORMATTING_FUNCTIONS 1

#define HEAT_NOZZLE2_Pin GPIO_PIN_2
#define HEAT_NOZZLE2_GPIO_Port GPIOE
#define INT_POWER_Pin GPIO_PIN_3
#define INT_POWER_GPIO_Port GPIOE
#define INT_POWER_EXTI_IRQn EXTI3_IRQn
#define DOOR_CHECK_Pin GPIO_PIN_4
#define DOOR_CHECK_GPIO_Port GPIOE
#define LIGHT_BAR_Pin GPIO_PIN_13
#define LIGHT_BAR_GPIO_Port GPIOC
#define HEAT_BED_Pin GPIO_PIN_14
#define HEAT_BED_GPIO_Port GPIOC
#define LEVEL_Pin GPIO_PIN_6
#define LEVEL_GPIO_Port GPIOF
#define ADC_B_Pin GPIO_PIN_0
#define ADC_B_GPIO_Port GPIOA
#define C_MIN_Pin GPIO_PIN_2
#define C_MIN_GPIO_Port GPIOH
#define C_STEP_Pin GPIO_PIN_3
#define C_STEP_GPIO_Port GPIOH
#define C_DIR_Pin GPIO_PIN_4
#define C_DIR_GPIO_Port GPIOH
#define C_EN_Pin GPIO_PIN_5
#define C_EN_GPIO_Port GPIOH
#define ADC_E_Pin GPIO_PIN_3
#define ADC_E_GPIO_Port GPIOA
#define CUT_CHECK_Pin GPIO_PIN_5
#define CUT_CHECK_GPIO_Port GPIOA
#define ADC_BED_Pin GPIO_PIN_6
#define ADC_BED_GPIO_Port GPIOA
#define ADC_CAVITY_Pin GPIO_PIN_1
#define ADC_CAVITY_GPIO_Port GPIOB
#define Y_DIR_Pin GPIO_PIN_10
#define Y_DIR_GPIO_Port GPIOB
#define E_DIR_Pin GPIO_PIN_8
#define E_DIR_GPIO_Port GPIOH
#define Y_STEP_Pin GPIO_PIN_12
#define Y_STEP_GPIO_Port GPIOB
#define Y_MIN_Pin GPIO_PIN_13
#define Y_MIN_GPIO_Port GPIOB
#define HEAT_NOZZLE1_Pin GPIO_PIN_6
#define HEAT_NOZZLE1_GPIO_Port GPIOC
#define Z_EN_Pin GPIO_PIN_7
#define Z_EN_GPIO_Port GPIOC
#define Z_DIR_Pin GPIO_PIN_8
#define Z_DIR_GPIO_Port GPIOC
#define Z_STEP_Pin GPIO_PIN_9
#define Z_STEP_GPIO_Port GPIOC
#define FAN_MOTOR_EB_Pin GPIO_PIN_8
#define FAN_MOTOR_EB_GPIO_Port GPIOA
#define Z_MIN_Pin GPIO_PIN_10
#define Z_MIN_GPIO_Port GPIOC
#define Z_MAX_Pin GPIO_PIN_11
#define Z_MAX_GPIO_Port GPIOC
#define E_STEP_Pin GPIO_PIN_12
#define E_STEP_GPIO_Port GPIOC
#define X_EN_Pin GPIO_PIN_2
#define X_EN_GPIO_Port GPIOD
#define B_STEP_Pin GPIO_PIN_3
#define B_STEP_GPIO_Port GPIOD
#define Y_EN_Pin GPIO_PIN_7
#define Y_EN_GPIO_Port GPIOD
#define B_EN_Pin GPIO_PIN_10
#define B_EN_GPIO_Port GPIOG
#define E_EN_Pin GPIO_PIN_12
#define E_EN_GPIO_Port GPIOG
#define B_DIR_Pin GPIO_PIN_4
#define B_DIR_GPIO_Port GPIOB
#define LCD_BL_Pin GPIO_PIN_5
#define LCD_BL_GPIO_Port GPIOB
#define X_DIR_Pin GPIO_PIN_6
#define X_DIR_GPIO_Port GPIOB
#define X_STEP_Pin GPIO_PIN_7
#define X_STEP_GPIO_Port GPIOB
#define X_MIN_Pin GPIO_PIN_8
#define X_MIN_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */

/**
  * @}
*/

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
