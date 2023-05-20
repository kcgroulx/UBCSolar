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
enum DriveState {
	INVALID = (uint32_t) 0x0000,
    IDLE = (uint32_t) 0x0001,
    NORMAL_READY = (uint32_t) 0x0002,
    REGEN_READY = (uint32_t) 0x0004,
	CRUISE_READY = (uint32_t) 0x0008,
	MOTOR_OVERHEAT = (uint32_t) 0x0010
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

#define CRUISE_INCREMENT_VAL 1 // How much pressing the cruise up/down buttons increase or decrease the cruise velocity
#define CRUISE_MAX 30 // Max cruise speed
#define CRUISE_MIN 5 // Min cruise speed


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint16_t ADC_throttle_val;
uint16_t ADC_regen_val;

struct InputFlags event_flags;

int velocity_cruise;

// TODO Impliment task to get the current velocity
int velocity_current = 20;

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
/* Definitions for updateFlags */
osThreadId_t updateFlagsHandle;
const osThreadAttr_t updateFlags_attributes = {
  .name = "updateFlags",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for motorStateMachi */
osThreadId_t motorStateMachiHandle;
const osThreadAttr_t motorStateMachi_attributes = {
  .name = "motorStateMachi",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void getADCValues(void *argument);
void startUpdateFlags(void *argument);
void startMotorStateMachine(void *argument);

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

  /* creation of updateFlags */
  updateFlagsHandle = osThreadNew(startUpdateFlags, NULL, &updateFlags_attributes);

  /* creation of motorStateMachi */
  motorStateMachiHandle = osThreadNew(startMotorStateMachine, NULL, &motorStateMachi_attributes);

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
  /* Infinite loop */
  for(;;)
  {
	// Get ADC values for throttle and regen
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	ADC_throttle_val = HAL_ADC_GetValue(&hadc1);

	event_flags.throttle_pressed = ADC_throttle_val > ADC_DEADZONE;

	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
	ADC_regen_val = HAL_ADC_GetValue(&hadc2);

	event_flags.regen_pressed = ADC_regen_val > ADC_DEADZONE;

    osDelay(ADC_POLL_DELAY);
  }
  /* USER CODE END getADCValues */
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
	  // TODO impliment Mechanical brake signal to state machine
	  // TODO impliment battery charge check for regen
	  // order of priorities beginning with most important: regen braking, encoder motor command, cruise control
	  if (event_flags.regen_pressed) {
		  state = REGEN_READY;
	  }
	  else if (event_flags.cruise_status) {
	  	 state = CRUISE_READY;
	  }
	  else if (event_flags.throttle_pressed) {
	  	  	 state = NORMAL_READY;
	  }
	  else {
	  	 state = IDLE;
	  }
	  osDelay(EVENT_FLAG_UPDATE_DELAY);
  }
  /* USER CODE END startUpdateFlags */
}

/* USER CODE BEGIN Header_startMotorStateMachine */
/**
* @brief Function implementing the motorStateMachi thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startMotorStateMachine */
void startMotorStateMachine(void *argument)
{
  /* USER CODE BEGIN startMotorStateMachine */
  char msg[20];
  char mode;
  union FloatBytes current;
  union FloatBytes velocity;
  /* Infinite loop */

  //TODO Add Wrapper Functions
  for(;;)
  {
    if(state == REGEN_READY)
    {
    	velocity.float_value = 0;
    	current.float_value = (ADC_regen_val - ADC_DEADZONE >= 0 ? ((float)(ADC_regen_val - ADC_DEADZONE))/ADC_MAX : 0.0);

    	mode = 'R';
    }
    else if(state == NORMAL_READY)
    {
    	if (event_flags.reverse_enable)
    		velocity.float_value = -100.0;
    	else
    		velocity.float_value = 100.0;

    	current.float_value = (ADC_throttle_val - ADC_DEADZONE >= 0 ? ((float)(ADC_throttle_val - ADC_DEADZONE))/ADC_MAX : 0.0);

    	mode = 'D';
    }
    else if (state == CRUISE_READY)
    {
    	velocity.float_value = velocity_cruise;
    	current.float_value = 1;

    	mode = 'C';
    }
    else
    {
    	velocity.float_value = 0;
    	current.float_value = 0;

    	mode = 'I';
    }

    //TODO Replace with CAN
    sprintf(msg, "S:%c V:%d C:%d   \r", mode,(int)velocity.float_value, (int)(current.float_value*100));
    HAL_UART_Transmit(&huart2, msg, sizeof(msg),100);
    osDelay(10);
  }
  /* USER CODE END startMotorStateMachine */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	char msg[20];
	if(GPIO_Pin == BTN_CRUISE_TOGGLE_Pin)
	{
		if(state == NORMAL_READY || state == CRUISE_READY)
		{
			event_flags.cruise_status = !event_flags.cruise_status;
			velocity_cruise = velocity_current;
		}

	}
	else if(GPIO_Pin == BTN_CRUISE_UP_Pin)
	{
		if(event_flags.cruise_status)
			if(velocity_cruise + CRUISE_INCREMENT_VAL < CRUISE_MAX)
				velocity_cruise += CRUISE_INCREMENT_VAL;
			else
				velocity_cruise = CRUISE_MAX;
	}
	else if(GPIO_Pin == BTN_CRUISE_DOWN_Pin)
	{
		if(event_flags.cruise_status)
			if(velocity_cruise - CRUISE_INCREMENT_VAL > CRUISE_MIN)
				velocity_cruise -= CRUISE_INCREMENT_VAL;
			else
				velocity_cruise = CRUISE_MIN;
	}
	else if(GPIO_Pin == BTN_REVERSE_Pin)
	{
		if(state == IDLE)
			event_flags.reverse_enable = !event_flags.reverse_enable;
	}
	//osDelay(5);
}
/* USER CODE END Application */

