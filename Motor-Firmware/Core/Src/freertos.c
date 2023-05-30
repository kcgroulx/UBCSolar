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

// TODO Add gpio inputs (park, reverse, mech_brake) to event_flags?
// - Adding park reverse and mech_brake to event_flags will in theory lower performance but greatly improve code readablity
// TODO Confirm velocity readings will be signed.
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
	MOTOR_OVERHEAT = (uint32_t) 0x0010,
	PARK = (uint32_t) 0x0012
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
#define CAN_DATA_LENGTH 8 			// incoming data is 64 bits total
#define EVENT_FLAG_UPDATE_DELAY 5

#define MIN_REVERSE_VELOCITY 3

#define GET_CAN_VELOCITY_DELAY 500
#define READ_BATTERY_SOC_DELAY 5000
#define DELAY_MOTOR_STATE_MACHINE 10


#define CRUISE_INCREMENT_VAL 1 // How much pressing the cruise up/down buttons increase or decrease the cruise velocity
#define CRUISE_MAX 30 // Max cruise speed
#define CRUISE_MIN 5 // Min cruise speed

#define BATTERY_SOC_THRESHHOLD 90


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint16_t ADC_throttle_val;
uint16_t ADC_regen_val;

union FloatBytes velocity_current; // Current velocity of the car will be stored here.

struct InputFlags event_flags; // Event flags for deciding what state to be in.

int velocity_cruise;

uint8_t battery_soc;


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
/* Definitions for getCANVelocity */
osThreadId_t getCANVelocityHandle;
const osThreadAttr_t getCANVelocity_attributes = {
  .name = "getCANVelocity",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for getCANBatterySO */
osThreadId_t getCANBatterySOHandle;
const osThreadAttr_t getCANBatterySO_attributes = {
  .name = "getCANBatterySO",
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
void startGetCANVelocity(void *argument);
void StartSetCANBatterySO(void *argument);

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

  /* creation of getCANVelocity */
  getCANVelocityHandle = osThreadNew(startGetCANVelocity, NULL, &getCANVelocity_attributes);

  /* creation of getCANBatterySO */
  getCANBatterySOHandle = osThreadNew(StartSetCANBatterySO, NULL, &getCANBatterySO_attributes);

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

	if(ADC_throttle_val > ADC_DEADZONE)
	{
		event_flags.throttle_pressed = TRUE;
		//event_flags.cruise_status = FALSE;
	}
	else
	{
		event_flags.throttle_pressed = FALSE;
	}

	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
	ADC_regen_val = HAL_ADC_GetValue(&hadc2);

	if(ADC_regen_val > ADC_DEADZONE)
	{
		event_flags.regen_pressed = TRUE;
		event_flags.cruise_status = FALSE;
	}
	else
	{
		event_flags.regen_pressed = FALSE;
	}
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
	  if (HAL_GPIO_ReadPin(SWITCH_PARK_GPIO_Port, SWITCH_PARK_Pin))
		  state = PARK;
	  else if (HAL_GPIO_ReadPin(MECH_BRAKE_GPIO_Port, MECH_BRAKE_Pin))
		  state = IDLE;
	  else if (event_flags.regen_pressed && battery_soc < BATTERY_SOC_THRESHHOLD)
		  state = REGEN_READY;
	  else if (event_flags.cruise_status)
	  	 state = CRUISE_READY;
	  else if (event_flags.throttle_pressed)
	  	 state = NORMAL_READY;
	  else
	  	 state = IDLE;
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
  uint8_t data_send[CAN_DATA_LENGTH];
  /* Infinite loop */
  for(;;)
  {
	if(state == PARK)
	{
		velocity.float_value = 0;
		current.float_value = 1;

		mode = 'P';
	}
	else if(state == REGEN_READY)
    {
    	velocity.float_value = 0;
    	current.float_value = (ADC_regen_val - ADC_DEADZONE >= 0 ? ((float)(ADC_regen_val - ADC_DEADZONE))/ADC_MAX : 0.0);

    	mode = 'R';
    }
    else if(state == NORMAL_READY)
    {
    	if (HAL_GPIO_ReadPin(SWITCH_REVERSE_GPIO_Port, SWITCH_REVERSE_Pin) == 1 && velocity_current.float_value < MIN_REVERSE_VELOCITY)
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

    sprintf(msg, "S:%c V:%d C:%d   \r", mode,(int)velocity.float_value, (int)(current.float_value*100));
    HAL_UART_Transmit(&huart2, msg, sizeof(msg),100);

    //TODO test CAN
    /*
    // writing data into data_send array which will be sent as a CAN message
    for (int i = 0; i < (uint8_t) CAN_DATA_LENGTH / 2; i++) {
    	data_send[i] = velocity.bytes[i];
        data_send[4 + i] = current.bytes[i];
    }

    HAL_CAN_AddTxMessage(&hcan, &drive_command_header, data_send, &can_mailbox);
	*/

    osDelay(DELAY_MOTOR_STATE_MACHINE);
  }
  /* USER CODE END startMotorStateMachine */
}

/* USER CODE BEGIN Header_startGetCANVelocity */
/**
* @brief Function implementing the getCANVelocity thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_startGetCANVelocity */
void startGetCANVelocity(void *argument)
{
  /* USER CODE BEGIN startGetCANVelocity */
	uint8_t CAN_message[8];
	/* Infinite loop */
	for(;;)
	{
		if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0))
		{
			// there are multiple CAN IDs being passed through the filter, check if the message is the SOC
			HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &can_rx_header, CAN_message);
			if (can_rx_header.StdId == 0x503)
			{
				for(int i= 0; i < 4; i++)
				{
					velocity_current.bytes[i] = CAN_message[i+4]; // Vechicle Velocity is stored in bits 32-63.
				}
			}
		}
		osDelay(GET_CAN_VELOCITY_DELAY);
	}
  /* USER CODE END startGetCANVelocity */
}

/* USER CODE BEGIN Header_StartSetCANBatterySO */
/**
* @brief Function implementing the getCANBatterySO thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSetCANBatterySO */
void StartSetCANBatterySO(void *argument)
{
  /* USER CODE BEGIN StartSetCANBatterySO */
	uint8_t battery_msg_data[8];
	/* Infinite loop */
	for(;;)
	{
		if (HAL_CAN_GetRxFifoFillLevel(&hcan, CAN_RX_FIFO0))
		{
			// there are multiple CAN IDs being passed through the filter, check if the message is the SOC
			HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &can_rx_header, battery_msg_data);
			if (can_rx_header.StdId == 0x626)
			{
				// if the battery SOC is out of range, assume it is at 100% as a safety measure
				if (battery_msg_data[0] < 0 || battery_msg_data[0] > 100)
					battery_soc = 100;
				else
					battery_soc = battery_msg_data[0];
			}

	  		osDelay(READ_BATTERY_SOC_DELAY);
		}
	}
  /* USER CODE END StartSetCANBatterySO */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{


	if(GPIO_Pin == BTN_CRUISE_TOGGLE_Pin)
	{

		if(state == NORMAL_READY || state == CRUISE_READY)
		{
			event_flags.cruise_status = !event_flags.cruise_status;
			velocity_cruise = velocity_current.float_value;
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
	else if (GPIO_Pin == MECH_BRAKE_Pin)
	{
		event_flags.cruise_status = FALSE;
	}
}
/* USER CODE END Application */

