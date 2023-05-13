/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart.h"
#include "adc.h"
#include "can.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
union FloatBytes current;
union FloatBytes velocity;

struct InputFlags event_flags;

enum DriveState {
	INVALID = (uint32_t) 0x0000,		    /**< Indicates an error state. The MCB should not ever be in this state. */

    IDLE = (uint32_t) 0x0001,			    /**< Indicates an idle state. The MCB will enter this state when the car is stationary */

    NORMAL_READY = (uint32_t) 0x0002,	    /**< Indicates a normal state. The MCB will enter this state when neither cruise control is
                                              on nor regenerative braking. In this state, the pedal encoder directly determines the
                                              motor current and therefore the throttle provided by the motor. */

    REGEN_READY = (uint32_t) 0x0004,	    /**< Indicates that regenerative braking is active. The MCB will enter this state when
                                              REGEN_EN is enabled and the regen value is more than zero. This state takes priority
                                              over NORMAL_READY, CRUISE_READY, and IDLE.*/

	CRUISE_READY = (uint32_t) 0x0008,		/**< Indicates that cruise control is active. The MCB will enter this state when CRUISE_EN
											 is enabled and the cruise value is more than zero. Pressing the pedal down or pressing
											 the brake will exit the cruise control state. */

	MOTOR_OVERHEAT = (uint32_t) 0x0010  	/**< Indicates that when the motor is over the maximum temperature. The value is set in the
	 	 	 	 	 	 	 	 	 	 	 sendMotorOverheatTask which checks the incoming motor temperature received over CAN. */
} state;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_POLL_DELAY 10
#define SEND_MOTOR_COMMAND_DELAY 10
#define ADC_DEADZONE 500
#define ADC_MAX 4000
#define TRUE 1
#define FALSE 0
#define CAN_DATA_LENGTH 8 			// incoming data is 64 bytes total
#define EVENT_FLAG_UPDATE_DELAY 5

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for GetADCValues */
osThreadId_t GetADCValuesHandle;
const osThreadAttr_t GetADCValues_attributes = {
  .name = "GetADCValues",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for sendRegen */
osThreadId_t sendRegenHandle;
const osThreadAttr_t sendRegen_attributes = {
  .name = "sendRegen",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for sendThrottle */
osThreadId_t sendThrottleHandle;
const osThreadAttr_t sendThrottle_attributes = {
  .name = "sendThrottle",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for updateFlags */
osThreadId_t updateFlagsHandle;
const osThreadAttr_t updateFlags_attributes = {
  .name = "updateFlags",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for sendIdle */
osThreadId_t sendIdleHandle;
const osThreadAttr_t sendIdle_attributes = {
  .name = "sendIdle",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void getADCValues(void *argument);
void StartSendRegen(void *argument);
void startSendThrottle(void *argument);
void startUpdateFlags(void *argument);
void StartSendIdle(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of GetADCValues */
  GetADCValuesHandle = osThreadNew(getADCValues, NULL, &GetADCValues_attributes);

  /* creation of sendRegen */
  sendRegenHandle = osThreadNew(StartSendRegen, NULL, &sendRegen_attributes);

  /* creation of sendThrottle */
  sendThrottleHandle = osThreadNew(startSendThrottle, NULL, &sendThrottle_attributes);

  /* creation of updateFlags */
  updateFlagsHandle = osThreadNew(startUpdateFlags, NULL, &updateFlags_attributes);

  /* creation of sendIdle */
  sendIdleHandle = osThreadNew(StartSendIdle, NULL, &sendIdle_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
	 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
     osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_getADCValues */
/**
* @brief Function implementing the GetADCValues thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_getADCValues */
void getADCValues(void *argument)
{
  /* USER CODE BEGIN getADCValues */
  uint16_t ADC_throttle_val;
  uint16_t ADC_regen_val;
  char msg[20];
  /* Infinite loop */
  for(;;)
  {
	// Get ADC values for throttle and regen
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	ADC_throttle_val = HAL_ADC_GetValue(&hadc1);

	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
	ADC_regen_val = HAL_ADC_GetValue(&hadc2);

	// Checks if regen paddle is pressed
	if(ADC_regen_val > ADC_DEADZONE)
	{
		velocity.float_value = 0.0;
		current.float_value = (ADC_regen_val - ADC_DEADZONE >= 0 ? ((float)(ADC_regen_val - ADC_DEADZONE))/ADC_MAX : 0.0);
	}
	else if(ADC_throttle_val > ADC_DEADZONE)
	{
		//Checks for reverse
		if (event_flags.reverse_enable)
			velocity.float_value = -100.0;
		else
			velocity.float_value = 100.0;

		current.float_value = (ADC_throttle_val - ADC_DEADZONE >= 0 ? ((float)(ADC_throttle_val - ADC_DEADZONE))/ADC_MAX : 0.0);

		state = NORMAL_READY;
	}
	else
	{
		state = IDLE;
	}
 &
	//HAL_UART_Transmit_IT(&huart2, msg, sizeof(msg));

    osDelay(ADC_POLL_DELAY);
  }
  /* USER CODE END getADCValues */
}

/* USER CODE BEGIN Header_StartSendRegen */
/**
* @brief Function implementing the sendRegen thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSendRegen */
void StartSendRegen(void *argument)
{
  /* USER CODE BEGIN StartSendRegen */
  char msg[20];
  /* Infinite loop */
  for(;;)
  {
	  //TODO Replace with osEventFlagsWait()
      osEventFlagsWait(commandEventFlagsHandle, REGEN_READY, osFlagsWaitAll, osWaitForever);


	  if(state == REGEN_READY)
	  {
		  // TODO Impliment CAN instead of UART and test with PiCAN
		  sprintf(msg, "  Regen State   \r");
		  HAL_UART_Transmit(&huart2, msg, sizeof(msg),100);
	  }
	  osDelay(10);
  }
  /* USER CODE END StartSendRegen */
}

/* USER CODE BEGIN Header_startSendThrottle */
/**
* @brief Function implementing the sendThrottle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startSendThrottle */
void startSendThrottle(void *argument)
{
  /* USER CODE BEGIN startSendThrottle */
  uint8_t CAN_msg[CAN_DATA_LENGTH];
  char msg[20];
  /* Infinite loop */
  for(;;)
  {
	if(state == NORMAL_READY)
	{
	  sprintf(msg, "  Throttle State   \r");
	  HAL_UART_Transmit(&huart2, msg, sizeof(msg), 100);

	  // writing data into data_send array which will be sent as a CAN message
	  for (int i = 0; i < (uint8_t) CAN_DATA_LENGTH / 2; i++) {
		  CAN_msg[i] = velocity.bytes[i];
	      CAN_msg[4 + i] = current.bytes[i];
	  }
	  HAL_CAN_AddTxMessage(&hcan, &drive_command_header, CAN_msg, &can_mailbox);
	}
    osDelay(10);
  }
  /* USER CODE END startSendThrottle */
}

/* USER CODE BEGIN Header_startUpdateFlags */
/**
* @brief Function implementing the updateFlags thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startUpdateFlags */
void startUpdateFlags(void *argument)
{
  /* USER CODE BEGIN startUpdateFlags */

  /* Infinite loop */
  for(;;)
  {
	  // order of priorities beginning with most important: motor over heating, regen braking, encoder motor command, cruise control
	  if (event_flags.regen_enable && regen_value > 0 && battery_soc < BATTERY_REGEN_THRESHOLD) {
		  state = REGEN_READY;
	  }
	  else if (!event_flags.encoder_value_is_zero && !event_flags.cruise_status) {
	  	 state = NORMAL_READY;
	  }
	  else if (event_flags.cruise_status && cruise_value > 0 && !event_flags.brake_in) {
	  	 state = CRUISE_READY;
	  }
	  else {
	  	 state = IDLE;
	  }

	  // signals the MCB state to other threads
	  osEventFlagsSet(commandEventFlagsHandle, state);

	  osDelay(EVENT_FLAG_UPDATE_DELAY);
  }
  /* USER CODE END startUpdateFlags */
}

/* USER CODE BEGIN Header_StartSendIdle */
/**
* @brief Function implementing the sendIdle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSendIdle */
void StartSendIdle(void *argument)
{
  /* USER CODE BEGIN StartSendIdle */
  char msg[20];
  /* Infinite loop */
  for(;;)
  {
	if(state == IDLE)
	{
		sprintf(msg, "  Idle State   \r");
	  	HAL_UART_Transmit(&huart2, msg, sizeof(msg), 100);
	}
    osDelay(10);
  }
  /* USER CODE END StartSendIdle */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

