/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/string.h>

#include <time.h>


#include <sensor_msgs/msg/temperature.h>             // for the ftemperature msg
#include <sensor_msgs/msg/battery_state.h>			 // for the battery state;
/*sensor's libraries*/
#include "LM75.h"
#include "INA219.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TEMP_MAX 27.5
/* Thread Flags*/

#define READ_ftemperature 	  0x0001
#define BATTERY_DATA_READY  0x0002
#define SOC_READY 			  0x0003

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

extern UART_HandleTypeDef hlpuart1;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern I2C_HandleTypeDef hi2c4;

uint8_t address=0;

/*global variables - ftemperature, battery state, battery SOC*/
typedef struct battery{
	float fvoltage;
	float fadcConstant;
} battery;
typedef struct batteryCellSoc{
	battery batteryCell[2];
	float fcurrent;
	float ftemperature;
	float fpercentage;
	float fcapacity;
} batterySoc;

batterySoc soc;
uint32_t now;
uint32_t previous_time;

/*Micro-ros variables*/
rcl_allocator_t freeRTOS_allocator;;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rclc_executor_t executor;
rcl_init_options_t init_options;
rcl_node_options_t node_ops;

/* Publisher declaration */
rcl_publisher_t battery_state_pub;
/* ROS timer declaration */
rcl_timer_t timer;

/* Messages declaration */
sensor_msgs__msg__BatteryState battery_state_msg;

/* USER CODE END Variables */
/* Definitions for microROSTask */
osThreadId_t microROSTaskHandle;
const osThreadAttr_t microROSTask_attributes = {
  .name = "microROSTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 3100 * 4
};
/* Definitions for tempControlTask */
osThreadId_t tempControlTaskHandle;
const osThreadAttr_t tempControlTask_attributes = {
  .name = "tempControlTask",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for BatteryState */
osThreadId_t BatteryStateHandle;
const osThreadAttr_t BatteryState_attributes = {
  .name = "BatteryState",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 300 * 4
};
/* Definitions for SocTask */
osThreadId_t SocTaskHandle;
const osThreadAttr_t SocTask_attributes = {
  .name = "SocTask",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for batteryCellSocMutex */
osMutexId_t batteryCellSocMutexHandle;
const osMutexAttr_t batteryCellSocMutex_attributes = {
  .name = "batteryCellSocMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);

void timer_callback(rcl_timer_t * timer, int64_t last_call_time);
void cmd_vel_callback(const void * msgin);

double getftemperature(void);
void microrosConfig(void);
void microrosMessageAllocation(void);
//extern int clock_gettime( int clock_id, struct timespec * tp );
extern void UTILS_NanosecondsToTimespec( int64_t llSource, struct timespec * const pxDestination );

/* USER CODE END FunctionPrototypes */

void microROSTaskFunction(void *argument);
void ftemperatureControlTask(void *argument);
void BatteryStateFunction(void *argument);
void SocTaskFunction(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of batteryCellSocMutex */
  batteryCellSocMutexHandle = osMutexNew(&batteryCellSocMutex_attributes);

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
  /* creation of microROSTask */
  microROSTaskHandle = osThreadNew(microROSTaskFunction, NULL, &microROSTask_attributes);

  /* creation of tempControlTask */
  tempControlTaskHandle = osThreadNew(ftemperatureControlTask, NULL, &tempControlTask_attributes);

  /* creation of BatteryState */
  BatteryStateHandle = osThreadNew(BatteryStateFunction, NULL, &BatteryState_attributes);

  /* creation of SocTask */
  SocTaskHandle = osThreadNew(SocTaskFunction, NULL, &SocTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_microROSTaskFunction */
/**
  * @brief  Function implementing the microROSTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_microROSTaskFunction */
void microROSTaskFunction(void *argument)
{
  /* USER CODE BEGIN microROSTaskFunction */

	if(osThreadFlagsWait(SOC_READY, osFlagsWaitAny, 200)!=osFlagsErrorTimeout)
	{
		// micro-ROS configuration
		microrosConfig();
		// create battery_state publisher
		rclc_publisher_init_default(
			&battery_state_pub,
			&node,
			ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
			"/battery_state");
		// allocate memory to messages
		microrosMessageAllocation();
		// Create a timer
		rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(100), timer_callback);
		// Create executor
		rclc_executor_init(&executor, &support.context, 2, &allocator);
		rclc_executor_add_timer(&executor, &timer);
		// Run executor
		rclc_executor_spin(&executor);
	}
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END microROSTaskFunction */
}

/* USER CODE BEGIN Header_ftemperatureControlTask */
/**
* @brief Function implementing the tempControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ftemperatureControlTask */
void ftemperatureControlTask(void *argument)
{
  /* USER CODE BEGIN ftemperatureControlTask */
  /* Infinite loop */
  for(;;)
  {
	  if(osThreadFlagsWait(READ_ftemperature, osFlagsWaitAny, 100)!=osFlagsErrorTimeout)
	  {
		  // get ftemperature
		  if (osMutexAcquire(batteryCellSocMutexHandle, 100) == osOK)
		  {
			  soc.ftemperature = getTemperature();
			  osMutexRelease(batteryCellSocMutexHandle);
		  }
		  if(soc.ftemperature >= TEMP_MAX)
		  {
			  HAL_GPIO_WritePin(COOLER_GPIO_Port, COOLER_Pin, GPIO_PIN_SET);
		  }
		  else
			  HAL_GPIO_WritePin(COOLER_GPIO_Port, COOLER_Pin, GPIO_PIN_RESET);
	  }
    osDelay(1);
  }
  /* USER CODE END ftemperatureControlTask */
}

/* USER CODE BEGIN Header_BatteryStateFunction */
/**
* @brief Function implementing the BatteryState thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BatteryStateFunction */
void BatteryStateFunction(void *argument)
{
  /* USER CODE BEGIN BatteryStateFunction */
  /* Infinite loop */
  uint16_t raw;
  float fv1, fv2;
  float i= 0;
  uint8_t ui8Buff[2];
  HAL_StatusTypeDef halStatus;

  // battery constants
  soc.batteryCell[0].fadcConstant = (3.6*(10+56)/(10*4096));
  soc.batteryCell[1].fadcConstant = (3.6*(470+560)/(470*4096));
  for(;;)
  {
	  // get adc value
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  raw = HAL_ADC_GetValue(&hadc1);
	  HAL_ADC_Stop(&hadc1);
	  //convert to V
	  fv1 = (float)raw*soc.batteryCell[0].fadcConstant;
	  // get adc value
	  HAL_ADC_Start(&hadc2);
	  HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);
	  raw = HAL_ADC_GetValue(&hadc2);
	  //convert to V
	  fv2 = (float)raw*soc.batteryCell[1].fadcConstant -fv1;
	  HAL_ADC_Stop(&hadc2);
	  // get fcurrent
  	  ui8Buff[0]= 0x01;
      halStatus = HAL_I2C_Master_Transmit(&hi2c4, (0x40 << 1), ui8Buff, 1, 100);
      if(halStatus == HAL_OK)
      {
         halStatus = HAL_I2C_Master_Receive(&hi2c4, (0x40 << 1), ui8Buff, 2, 100);
         if(halStatus == HAL_OK)
         {
        	// Blink led to debug
           HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
           i= (float)(ui8Buff[0]<<8 | ui8Buff[1])/10; // pois o resistor Shunt é de 100ohm, ou 0,1 kOhm (saida em miliVolt)
           if (i < 0)
               i = i * (-1);
           }
      }


	  //acquire mutex to write in the soc and set BATTERY_DATA_READY
	  if (osMutexAcquire(batteryCellSocMutexHandle, 10) == osOK)
	  {
		 soc.batteryCell[0].fvoltage = fv1;
		 soc.batteryCell[1].fvoltage = fv2;
		 soc.fcurrent = i;
		 osMutexRelease(batteryCellSocMutexHandle);
		 osThreadFlagsSet(SocTaskHandle, BATTERY_DATA_READY);
	  }
	  osDelay(1);
  }
  /* USER CODE END BatteryStateFunction */
}

/* USER CODE BEGIN Header_SocTaskFunction */
/**
* @brief Function implementing the SocTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SocTaskFunction */
void SocTaskFunction(void *argument)
{
  /* USER CODE BEGIN SocTaskFunction */
  /* Infinite loop */
  soc.fcapacity = 2.2; //Ah
  soc.fpercentage = 1;
  for(;;)
  {
	  if(osThreadFlagsWait(BATTERY_DATA_READY, osFlagsWaitAny, 200)!=osFlagsErrorTimeout)
	  {
		  // coulomb counting
		  now = HAL_GetTick();
		  if (osMutexAcquire(batteryCellSocMutexHandle, 10) == osOK)
		  {
			 float dsoc = soc.fcurrent*(now-previous_time)/(1000*3600); // time in msec
			 soc.fpercentage = soc.fpercentage -(1/soc.fcapacity)*dsoc;
			 osMutexRelease(batteryCellSocMutexHandle);
		  }
		  previous_time = now;
		  osThreadFlagsSet(microROSTaskHandle, SOC_READY);
	  }
    osDelay(1);
  }
  /* USER CODE END SocTaskFunction */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* ***************************************************** */
/* Method name: timer_callback */
/* Method description: Executes the callback function to the microros task */
/* Input params: timer and the last call time */
/* Output params: NA*/
/* ***************************************************** */
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	if (timer != NULL) {
		// Fill the message timestamp
		struct timespec ts;
		int64_t time_ns;
		time_ns = rmw_uros_epoch_nanos();
		UTILS_NanosecondsToTimespec(time_ns, &ts);
		// Publish message
		 battery_state_msg.header.stamp.sec = ts.tv_sec;
		 battery_state_msg.header.stamp.nanosec = ts.tv_nsec;
		 battery_state_msg.capacity = 2.2;
		 if (osMutexAcquire(batteryCellSocMutexHandle, 10) == osOK)
		 {
			 battery_state_msg.temperature = soc.ftemperature;
			 battery_state_msg.voltage = soc.batteryCell[1].fvoltage + soc.batteryCell[0].fvoltage;
			 battery_state_msg.current = soc.fcurrent;
			 osMutexRelease(batteryCellSocMutexHandle);
		 }
		 rcl_ret_t ret = rcl_publish(&battery_state_pub, &battery_state_msg, NULL);
		 if (ret != RCL_RET_OK)
		 {
			  printf("Error publishing battery state (line %d)\n", __LINE__);
		 }


	}
}

/* ***************************************************** */
/* Method name: microrosConfig */
/* Method description: Executes the configuration of the microros node  to the microros task */
/* Input params: NA*/
/* Output params: NA*/
/* ***************************************************** */
void microrosConfig()
{
	  rmw_uros_set_custom_transport(
		true,
		(void *) &hlpuart1,
		cubemx_transport_open,
		cubemx_transport_close,
		cubemx_transport_write,
		cubemx_transport_read);

	  freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	  freeRTOS_allocator.allocate = microros_allocate;
	  freeRTOS_allocator.deallocate = microros_deallocate;
	  freeRTOS_allocator.reallocate = microros_reallocate;
	  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

	  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
		  printf("Error on default allocators (line %d)\n", __LINE__);
	  }


	  allocator = rcl_get_default_allocator();
	  init_options = rcl_get_zero_initialized_init_options();
	  rcl_ret_t ret = rcl_init_options_init(&init_options, allocator);
	  if(ret != RCL_RET_OK)
	  {
		  printf("Error on init options (line %d)\n", __LINE__);
	  }

	  // create init_options
	  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

	  // create node
	  node_ops = rcl_node_get_default_options();
	  node_ops.domain_id = 25;
	  rclc_node_init_with_options(&node, "acquisition_system", "", &support, &node_ops);


	  //time sync
	  if( rmw_uros_sync_session(1000) != RMW_RET_OK)
		  printf("Error on time sync (line %d)\n", __LINE__);

}
/* ***************************************************** */
/* Method name: microrosMessageAllocation */
/* Method description: Do the memory allocation to the microros message. In this case
 * it is using the battery_state_msg*/
/* Input params: NA*/
/* Output params: NA*/

/* ***************************************************** */
void microrosMessageAllocation()
{
	  // battery_state_msg allocation
	  battery_state_msg.header.frame_id.capacity = 20;
	  battery_state_msg.header.frame_id.data = (char*) pvPortMalloc(battery_state_msg.header.frame_id.capacity  * sizeof(char));
	  battery_state_msg.header.frame_id.size = strlen(battery_state_msg.header.frame_id.data);

	  battery_state_msg.temperature = soc.ftemperature;
	  battery_state_msg.capacity =2.20;
	  battery_state_msg.present = true;

	  battery_state_msg.voltage = 0;
	  battery_state_msg.current = soc.fcurrent;
	  battery_state_msg.percentage = soc.fpercentage;
	  battery_state_msg.power_supply_health = sensor_msgs__msg__BatteryState__POWER_SUPPLY_STATUS_DISCHARGING;
	  battery_state_msg.power_supply_technology = sensor_msgs__msg__BatteryState__POWER_SUPPLY_TECHNOLOGY_LION;
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
