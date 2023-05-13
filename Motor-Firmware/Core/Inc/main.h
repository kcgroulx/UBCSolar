/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

typedef union FloatBytes {
	float float_value;			/**< Float value member of the union. */
	uint8_t bytes[4];			/**< Array of 4 bytes member of union. */
} FloatBytes;

typedef struct InputFlags {
  volatile uint8_t regen_enable;			/**< Records the position of the regen switch. A value of 0x01 indicates regenerative braking is
  	  	  	  	  	  	  	  	  	  	  	 	 enabled and a value of 0x00 indicates that regen is disabled. */

  volatile uint8_t reverse_enable;			/**< Records the position of the reverse switch. A value of 0x01 indicates "reverse" is enabled
   	   	   	   	   	   	   	   	   	   	   	 	 and a value of 0x00 indicates that "reverse" is disabled. */

  volatile uint8_t cruise_status;			/**< Records the cruise control status. A value of 0x01 indicates that the CRUISE_EN button has
   	   	   	   	   	   	   	   	   	   	   	 	 pressed and a value of 0x00 means the CRUISE_DIS button has been pressed. */

  volatile uint8_t brake_in;				/**< Records if the brake has been pressed. A value of 0x01 indicates the brake pedal has been pressed
  	  	  	  	  	  	  	  	  	  	  	  	 and a value of 0x00 means the brake pedal has not been pressed. This value is used
  	  	  	  	  	  	  	  	  	  	  	  	 to ensure cruise control mode is exited when the brake pedal is pressed. */

  volatile uint8_t regen_value_is_zero;		/**< Flag that indicates if the regen value read from the ADC is zero or not. A value
  	  	  	  	  	  	  	  	  	  	  	  	 of 0x01 means the regen value is zero while a value of 0x00 means the regen value
  	  	  	  	  	  	  	  	  	  	  	  	 is not zero. */

  volatile uint8_t encoder_value_is_zero;	/**< Flag that indicates if the encoder value read from the hardware timer is zero
  	  	  	  	  	  	  	  	  	  	  	  	 or not. A value of 0x01 means the regen value is zero while a value of 0x00
  	  	  	  	  	  	  	  	  	  	  	  	 means the regen value is not zero. */

  volatile uint8_t motor_overheat;			/**< Flag that indicates if the motor is above its maximum temperature. A value of 0x01 means that
   	   	   	   	   	   	   	   	   	   	   	   	 the motor is over heating while 0x00 means that the motor condition is acceptable. */

} InputFlags;

extern union FloatBytes current;			/**< Stores the current value to send to motor controller over CAN */
extern union FloatBytes velocity;			/**< Stores the velocity value to send to motor controller over CAN */

extern struct InputFlags event_flags;

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define ADC_Throttle_Pin GPIO_PIN_0
#define ADC_Throttle_GPIO_Port GPIOA
#define ADC_Regen_Pin GPIO_PIN_1
#define ADC_Regen_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
