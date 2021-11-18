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

#include <sensor_msgs/msg/joint_state.h>             // for the encoder msg
#include <geometry_msgs/msg/twist.h>                 // for the motors control
#include <sensor_msgs/msg/temperature.h>             // for the temperature msg
#include "LM75.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern UART_HandleTypeDef hlpuart1;

/*global variables - temperature, battery state, battery SOC*/
double temperature;
typedef struct batteryState{
	float tension;
	float current;
} batteryState;
typedef struct batterySoc{
	batteryState currentState;
	float soc;
} batterySoc;

/* Publisher declaration */
rcl_publisher_t temperature_state_pub;
/* ROS timer declaration */
rcl_timer_t timer;

/* Messages declaration */
sensor_msgs__msg__Temperature temperature_msg;

/* USER CODE END Variables */
/* Definitions for microROSTask */
osThreadId_t microROSTaskHandle;
const osThreadAttr_t microROSTask_attributes = {
  .name = "microROSTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 3000 * 4
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
  .stack_size = 128 * 4
};
/* Definitions for SocTask */
osThreadId_t SocTaskHandle;
const osThreadAttr_t SocTask_attributes = {
  .name = "SocTask",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for temperatureMutex */
osMutexId_t temperatureMutexHandle;
const osMutexAttr_t temperatureMutex_attributes = {
  .name = "temperatureMutex"
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

double get_temperature(void);

//extern int clock_gettime( int clock_id, struct timespec * tp );
extern void UTILS_NanosecondsToTimespec( int64_t llSource, struct timespec * const pxDestination );

/* USER CODE END FunctionPrototypes */

void microROSTaskFunction(void *argument);
void temperatureControlTask(void *argument);
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
  /* creation of temperatureMutex */
  temperatureMutexHandle = osMutexNew(&temperatureMutex_attributes);

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
  tempControlTaskHandle = osThreadNew(temperatureControlTask, NULL, &tempControlTask_attributes);

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
	  // micro-ROS configuration
	  rmw_uros_set_custom_transport(
		true,
		(void *) &hlpuart1,
		cubemx_transport_open,
		cubemx_transport_close,
		cubemx_transport_write,
		cubemx_transport_read);

	  rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	  freeRTOS_allocator.allocate = microros_allocate;
	  freeRTOS_allocator.deallocate = microros_deallocate;
	  freeRTOS_allocator.reallocate = microros_reallocate;
	  freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

	  if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
		  printf("Error on default allocators (line %d)\n", __LINE__);
	  }

	  rclc_support_t support;
	  rcl_allocator_t allocator;
	  rcl_node_t node;
	  rclc_executor_t executor;
	  rcl_init_options_t init_options;

	  allocator = rcl_get_default_allocator();
	  init_options = rcl_get_zero_initialized_init_options();
	  rcl_init_options_init(&init_options, allocator);

	  // create init_options
	  rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator);

	  // create node
	  rcl_node_options_t node_ops = rcl_node_get_default_options();
	  node_ops.domain_id = 25;
	  rclc_node_init_with_options(&node, "acquisition_system", "", &support, &node_ops);


	  //time sync
	  if( rmw_uros_sync_session(1000) != RMW_RET_OK)
		  printf("Error on time sync (line %d)\n", __LINE__);

	  // create tempreature publisher
	  rclc_publisher_init_default(
		&temperature_state_pub,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Temperature),
		"/temperature_state");

	  // tempreature_msg message allocation. Described in https://micro.ros.org/docs/tutorials/advanced/handling_type_memory/

	  temperature_msg.header.frame_id.capacity = 20;
	  temperature_msg.header.frame_id.data = (char*) pvPortMalloc(temperature_msg.header.frame_id.capacity  * sizeof(char));
	  temperature_msg.header.frame_id.size = strlen(temperature_msg.header.frame_id.data);
	  temperature_msg.temperature = -1;
	  temperature_msg.variance = 0;
	  // Create a timer
	  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(10), timer_callback);

	  // Create executor
	  rclc_executor_init(&executor, &support.context, 2, &allocator);
	  rclc_executor_add_timer(&executor, &timer);

	  // Run executor
	  rclc_executor_spin(&executor);

  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END microROSTaskFunction */
}

/* USER CODE BEGIN Header_temperatureControlTask */
/**
* @brief Function implementing the tempControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_temperatureControlTask */
void temperatureControlTask(void *argument)
{
  /* USER CODE BEGIN temperatureControlTask */
  /* Infinite loop */
  for(;;)
  {
	  if(osThreadFlagsWait(READ_TEMPERATURE, osFlagsWaitAny, 3000)!=osFlagsErrorTimeout)
	  {
		  // get temperature
		  if (osMutexAcquire(temperatureMutexHandle, 100) == osOK)
		  {
			  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
			  temperature = get_temperature();
			  osMutexRelease(temperatureMutexHandle);
		  }
	  }
    osDelay(1);
  }
  /* USER CODE END temperatureControlTask */
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
  for(;;)
  {
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
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END SocTaskFunction */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	if (timer != NULL) {
		// Blink the LED1 (yellow) for debugging
		//HAL_GPIO_TogglePin(LD2_GPIO_Port , LD2_Pin);

		// Fill the message timestamp
		struct timespec ts;
		int64_t time_ns;
		time_ns = rmw_uros_epoch_nanos();
		UTILS_NanosecondsToTimespec(time_ns, &ts);
		//clock_gettime(CLOCK_REALTIME, &ts);

		// Create the Header
		temperature_msg.header.stamp.sec = ts.tv_sec;
		temperature_msg.header.stamp.nanosec = ts.tv_nsec;
		if (osMutexAcquire(temperatureMutexHandle, 10) == osOK)
		{
			temperature_msg.temperature = temperature;
			osMutexRelease(temperatureMutexHandle);
		}
		rcl_ret_t ret = rcl_publish(&temperature_state_pub, &temperature_msg, NULL);
		if (ret != RCL_RET_OK)
		{
		  printf("Error publishing temperate (line %d)\n", __LINE__);
		}


	}
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
