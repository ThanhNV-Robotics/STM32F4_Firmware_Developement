/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32f4xx_hal.h"

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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CN1_22_RDY_Pin GPIO_PIN_2
#define CN1_22_RDY_GPIO_Port GPIOE
#define CN1_21_SPDOUT_TRQOUT_Pin GPIO_PIN_3
#define CN1_21_SPDOUT_TRQOUT_GPIO_Port GPIOE
#define PE4_ZIGBEE_PA_EN_Pin GPIO_PIN_4
#define PE4_ZIGBEE_PA_EN_GPIO_Port GPIOE
#define PE5_BLE_GPIO_Pin GPIO_PIN_5
#define PE5_BLE_GPIO_GPIO_Port GPIOE
#define CN1_48_BRAKE_Pin GPIO_PIN_6
#define CN1_48_BRAKE_GPIO_Port GPIOE
#define PC13_USER_BT_MID_Pin GPIO_PIN_13
#define PC13_USER_BT_MID_GPIO_Port GPIOC
#define CN1_46_ALARM_Pin GPIO_PIN_0
#define CN1_46_ALARM_GPIO_Port GPIOC
#define CN1_47_INSPD_INPOS_Pin GPIO_PIN_1
#define CN1_47_INSPD_INPOS_GPIO_Port GPIOC
#define CN1_45_NCWOUT_NTQOUT_Pin GPIO_PIN_2
#define CN1_45_NCWOUT_NTQOUT_GPIO_Port GPIOC
#define PC3_ZIGBEE_nRST_Pin GPIO_PIN_3
#define PC3_ZIGBEE_nRST_GPIO_Port GPIOC
#define PA0_UART4_TX_ESP32_RX_Pin GPIO_PIN_0
#define PA0_UART4_TX_ESP32_RX_GPIO_Port GPIOA
#define PA1_TIM2_CH2_ENC_PBO_Pin GPIO_PIN_1
#define PA1_TIM2_CH2_ENC_PBO_GPIO_Port GPIOA
#define PA2_USER_BT_UP_Pin GPIO_PIN_2
#define PA2_USER_BT_UP_GPIO_Port GPIOA
#define PA3_LCD_RST_Pin GPIO_PIN_3
#define PA3_LCD_RST_GPIO_Port GPIOA
#define CN1_20_PCWOUT_PTQOUT_Pin GPIO_PIN_4
#define CN1_20_PCWOUT_PTQOUT_GPIO_Port GPIOA
#define PA5_TIM2_CH1_ENC_PAO_Pin GPIO_PIN_5
#define PA5_TIM2_CH1_ENC_PAO_GPIO_Port GPIOA
#define PA6_TIM3_CH1_ENC_PZO_Pin GPIO_PIN_6
#define PA6_TIM3_CH1_ENC_PZO_GPIO_Port GPIOA
#define CN1_23_TYPEOUT_Pin GPIO_PIN_7
#define CN1_23_TYPEOUT_GPIO_Port GPIOA
#define SerVoReset_PC4_18_Pin GPIO_PIN_4
#define SerVoReset_PC4_18_GPIO_Port GPIOC
#define Stop_PC5_43_Pin GPIO_PIN_5
#define Stop_PC5_43_GPIO_Port GPIOC
#define EStop_Not_PB0_17_Pin GPIO_PIN_0
#define EStop_Not_PB0_17_GPIO_Port GPIOB
#define ArlarmRST_PB1_42_Pin GPIO_PIN_1
#define ArlarmRST_PB1_42_GPIO_Port GPIOB
#define Tor1_PB2_16_Pin GPIO_PIN_2
#define Tor1_PB2_16_GPIO_Port GPIOB
#define Speed2_Not_PE7_15_Pin GPIO_PIN_7
#define Speed2_Not_PE7_15_GPIO_Port GPIOE
#define Type_Not_PE8_40_Pin GPIO_PIN_8
#define Type_Not_PE8_40_GPIO_Port GPIOE
#define PE9_TIM1_CH1_PFIN_Pin GPIO_PIN_9
#define PE9_TIM1_CH1_PFIN_GPIO_Port GPIOE
#define Dir_Not_PE10_14_Pin GPIO_PIN_10
#define Dir_Not_PE10_14_GPIO_Port GPIOE
#define SPDLIM_Not_PE11_38_Pin GPIO_PIN_11
#define SPDLIM_Not_PE11_38_GPIO_Port GPIOE
#define CCWLIM_Not_PE12_39_Pin GPIO_PIN_12
#define CCWLIM_Not_PE12_39_GPIO_Port GPIOE
#define Speed1_not_PE13_41_Pin GPIO_PIN_13
#define Speed1_not_PE13_41_GPIO_Port GPIOE
#define CWLIM_Not_PE14_13_Pin GPIO_PIN_14
#define CWLIM_Not_PE14_13_GPIO_Port GPIOE
#define PE15_RELAY1_Pin GPIO_PIN_15
#define PE15_RELAY1_GPIO_Port GPIOE
#define PB10_I2C2_SCL_LCD_IOEXP_Pin GPIO_PIN_10
#define PB10_I2C2_SCL_LCD_IOEXP_GPIO_Port GPIOB
#define PB11_I2C2_SDA_LCD_IOEXP_Pin GPIO_PIN_11
#define PB11_I2C2_SDA_LCD_IOEXP_GPIO_Port GPIOB
#define PB12_RELAY3_Pin GPIO_PIN_12
#define PB12_RELAY3_GPIO_Port GPIOB
#define PB13_Output_JP7_Pin GPIO_PIN_13
#define PB13_Output_JP7_GPIO_Port GPIOB
#define PB14_POS_CMD_OPC_EN_Pin GPIO_PIN_14
#define PB14_POS_CMD_OPC_EN_GPIO_Port GPIOB
#define PB15_485_MCU_PC_DIR_Pin GPIO_PIN_15
#define PB15_485_MCU_PC_DIR_GPIO_Port GPIOB
#define PD8_USART3_TX_485_MCU_PC_Pin GPIO_PIN_8
#define PD8_USART3_TX_485_MCU_PC_GPIO_Port GPIOD
#define PD9_USART3_RX_485_MCU_PC_Pin GPIO_PIN_9
#define PD9_USART3_RX_485_MCU_PC_GPIO_Port GPIOD
#define PD10_ESP32_EN_Pin GPIO_PIN_10
#define PD10_ESP32_EN_GPIO_Port GPIOD
#define PD11_ESP32_BOOT_SEL_Pin GPIO_PIN_11
#define PD11_ESP32_BOOT_SEL_GPIO_Port GPIOD
#define PD12_Input_J6_Pin GPIO_PIN_12
#define PD12_Input_J6_GPIO_Port GPIOD
#define PD13_MON1_2_EN_Pin GPIO_PIN_13
#define PD13_MON1_2_EN_GPIO_Port GPIOD
#define PD14_Input_J6_Pin GPIO_PIN_14
#define PD14_Input_J6_GPIO_Port GPIOD
#define PD15_SPDIN_TRQIN_EN_Pin GPIO_PIN_15
#define PD15_SPDIN_TRQIN_EN_GPIO_Port GPIOD
#define PC8_PR_Pin GPIO_PIN_8
#define PC8_PR_GPIO_Port GPIOC
#define PC9_ZIGBEE_HGM_EN_Pin GPIO_PIN_9
#define PC9_ZIGBEE_HGM_EN_GPIO_Port GPIOC
#define PA8_LINE_DRV_SELFTEST2_Pin GPIO_PIN_8
#define PA8_LINE_DRV_SELFTEST2_GPIO_Port GPIOA
#define PA9_LINE_RECV_SELFTEST_Pin GPIO_PIN_9
#define PA9_LINE_RECV_SELFTEST_GPIO_Port GPIOA
#define PA10_LINE_DRV_SELFTEST1_Pin GPIO_PIN_10
#define PA10_LINE_DRV_SELFTEST1_GPIO_Port GPIOA
#define PA11_ENC_RECEIV_EN_Pin GPIO_PIN_11
#define PA11_ENC_RECEIV_EN_GPIO_Port GPIOA
#define PA12_LINE_DRV_EN_Pin GPIO_PIN_12
#define PA12_LINE_DRV_EN_GPIO_Port GPIOA
#define PA15_SPI3_NSS_SPARE_Pin GPIO_PIN_15
#define PA15_SPI3_NSS_SPARE_GPIO_Port GPIOA
#define PC10_SPI3_SCK_SPARE_Pin GPIO_PIN_10
#define PC10_SPI3_SCK_SPARE_GPIO_Port GPIOC
#define PC12_UART5_TX_485_MCU_DRV_Pin GPIO_PIN_12
#define PC12_UART5_TX_485_MCU_DRV_GPIO_Port GPIOC
#define Input0_J6_DAC_ADC_Pin GPIO_PIN_0
#define Input0_J6_DAC_ADC_GPIO_Port GPIOD
#define Input1_J6_DAC_ADC_Pin GPIO_PIN_1
#define Input1_J6_DAC_ADC_GPIO_Port GPIOD
#define PD2_UART5_RX_485_MCU_DRV_Pin GPIO_PIN_2
#define PD2_UART5_RX_485_MCU_DRV_GPIO_Port GPIOD
#define PD7_A_CODE2_Pin GPIO_PIN_7
#define PD7_A_CODE2_GPIO_Port GPIOD
#define CN1_19_ZSPD_Pin GPIO_PIN_3
#define CN1_19_ZSPD_GPIO_Port GPIOB
#define PB4_SPI3_MISO_SPARE_Pin GPIO_PIN_4
#define PB4_SPI3_MISO_SPARE_GPIO_Port GPIOB
#define PB5_SPI3_MOSI_SPARE_Pin GPIO_PIN_5
#define PB5_SPI3_MOSI_SPARE_GPIO_Port GPIOB
#define PB6_RELAY2_Pin GPIO_PIN_6
#define PB6_RELAY2_GPIO_Port GPIOB
#define PB7_I2C1_SDA_DAC_ADC_Pin GPIO_PIN_7
#define PB7_I2C1_SDA_DAC_ADC_GPIO_Port GPIOB
#define PB8_I2C1_SCL_DAC_ADC_Pin GPIO_PIN_8
#define PB8_I2C1_SCL_DAC_ADC_GPIO_Port GPIOB
#define Input_JP7_Pin GPIO_PIN_9
#define Input_JP7_GPIO_Port GPIOB
#define PE0_485_MCU_DRV_DIR_Pin GPIO_PIN_0
#define PE0_485_MCU_DRV_DIR_GPIO_Port GPIOE
#define PE1_ZIGBEE_EN_Pin GPIO_PIN_1
#define PE1_ZIGBEE_EN_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
