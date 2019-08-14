/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define Board2_Thermistor_ADC_Pin GPIO_PIN_5
#define Board2_Thermistor_ADC_GPIO_Port GPIOF
#define Board1_Thermistor_ADC_Pin GPIO_PIN_7
#define Board1_Thermistor_ADC_GPIO_Port GPIOF
#define Lid_Thermistor_ADC_Pin GPIO_PIN_9
#define Lid_Thermistor_ADC_GPIO_Port GPIOF
#define Gearbox_Thermistor_adc_Pin GPIO_PIN_0
#define Gearbox_Thermistor_adc_GPIO_Port GPIOC
#define POT_MISO_Pin GPIO_PIN_2
#define POT_MISO_GPIO_Port GPIOC
#define POT_MOSI_Pin GPIO_PIN_3
#define POT_MOSI_GPIO_Port GPIOC
#define Arrestment_Thermistor_ADC_Pin GPIO_PIN_3
#define Arrestment_Thermistor_ADC_GPIO_Port GPIOA
#define Meter_Thermistor_1_ADC_Pin GPIO_PIN_4
#define Meter_Thermistor_1_ADC_GPIO_Port GPIOA
#define Meter_Thermistor_2_ADC_Pin GPIO_PIN_5
#define Meter_Thermistor_2_ADC_GPIO_Port GPIOA
#define Long_Level_ADC_Pin GPIO_PIN_6
#define Long_Level_ADC_GPIO_Port GPIOA
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB
#define Conning_Tower_Thermistor_ADC_Pin GPIO_PIN_1
#define Conning_Tower_Thermistor_ADC_GPIO_Port GPIOB
#define Encoder_MOSI_Pin GPIO_PIN_2
#define Encoder_MOSI_GPIO_Port GPIOB
#define Beam_ADC_Pin GPIO_PIN_11
#define Beam_ADC_GPIO_Port GPIOF
#define Cross_Level_ADC_Pin GPIO_PIN_13
#define Cross_Level_ADC_GPIO_Port GPIOF
#define POT_CS_Pin GPIO_PIN_14
#define POT_CS_GPIO_Port GPIOF
#define Encoder_CS_Pin GPIO_PIN_15
#define Encoder_CS_GPIO_Port GPIOF
#define PWM65K_Pin GPIO_PIN_9
#define PWM65K_GPIO_Port GPIOE
#define PWM65K_180_Pin GPIO_PIN_11
#define PWM65K_180_GPIO_Port GPIOE
#define PWM125Hz_Pin GPIO_PIN_13
#define PWM125Hz_GPIO_Port GPIOE
#define POT_SCLK_Pin GPIO_PIN_10
#define POT_SCLK_GPIO_Port GPIOB
#define LD3_Pin GPIO_PIN_14
#define LD3_GPIO_Port GPIOB
#define Bluetooth_RX_Pin GPIO_PIN_15
#define Bluetooth_RX_GPIO_Port GPIOB
#define STLINK_RX_Pin GPIO_PIN_8
#define STLINK_RX_GPIO_Port GPIOD
#define STLINK_TX_Pin GPIO_PIN_9
#define STLINK_TX_GPIO_Port GPIOD
#define USB_OTG_FS_PWR_EN_Pin GPIO_PIN_10
#define USB_OTG_FS_PWR_EN_GPIO_Port GPIOD
#define Board1_Heater_Pin GPIO_PIN_15
#define Board1_Heater_GPIO_Port GPIOD
#define Board2_Heater_Pin GPIO_PIN_2
#define Board2_Heater_GPIO_Port GPIOG
#define Gearbox_Heater_Pin GPIO_PIN_3
#define Gearbox_Heater_GPIO_Port GPIOG
#define Conning_Tower_Heater_Pin GPIO_PIN_4
#define Conning_Tower_Heater_GPIO_Port GPIOG
#define Arrestment_Heater_Pin GPIO_PIN_5
#define Arrestment_Heater_GPIO_Port GPIOG
#define Meter_Heater_Pin GPIO_PIN_6
#define Meter_Heater_GPIO_Port GPIOG
#define USB_OTG_FS_OVCR_Pin GPIO_PIN_7
#define USB_OTG_FS_OVCR_GPIO_Port GPIOG
#define PWM_Levels_Pin GPIO_PIN_6
#define PWM_Levels_GPIO_Port GPIOC
#define Encoder_INT_Pin GPIO_PIN_15
#define Encoder_INT_GPIO_Port GPIOA
#define Encoder_SCLK_Pin GPIO_PIN_10
#define Encoder_SCLK_GPIO_Port GPIOC
#define Shaft_STBY_Pin GPIO_PIN_12
#define Shaft_STBY_GPIO_Port GPIOC
#define Shaft_Fault_Pin GPIO_PIN_0
#define Shaft_Fault_GPIO_Port GPIOD
#define Shaft_EN_Pin GPIO_PIN_1
#define Shaft_EN_GPIO_Port GPIOD
#define Shaft_Mode2_Pin GPIO_PIN_2
#define Shaft_Mode2_GPIO_Port GPIOD
#define Shaft_Mode1_Pin GPIO_PIN_3
#define Shaft_Mode1_GPIO_Port GPIOD
#define Shaft_Step_Pin GPIO_PIN_4
#define Shaft_Step_GPIO_Port GPIOD
#define Shaft_Dir_Pin GPIO_PIN_5
#define Shaft_Dir_GPIO_Port GPIOD
#define DAC_SPI1_CS_Pin GPIO_PIN_6
#define DAC_SPI1_CS_GPIO_Port GPIOD
#define DAC_SPI1_MOSI_Pin GPIO_PIN_7
#define DAC_SPI1_MOSI_GPIO_Port GPIOD
#define DAC_SPI1_MISO_Pin GPIO_PIN_9
#define DAC_SPI1_MISO_GPIO_Port GPIOG
#define DAC_SPI1_SCLK_Pin GPIO_PIN_3
#define DAC_SPI1_SCLK_GPIO_Port GPIOB
#define Encoder_MISO_Pin GPIO_PIN_4
#define Encoder_MISO_GPIO_Port GPIOB
#define Bluetooth_TX_Pin GPIO_PIN_6
#define Bluetooth_TX_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_1
#define LD2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
