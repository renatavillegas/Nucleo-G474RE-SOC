/*
 * LM75.c
 *
 *  Created on: Nov 6, 2021
 *      Author: renata
 */

/* Includes ------------------------------------------------------------------*/
#include "LM75.h"
#include "main.h"
#include "string.h"
/* Variables ------------------------------------------------------------------*/
HAL_StatusTypeDef ret;
uint8_t buf[12];
uint16_t raw;
float temp_c;

/* External variables --------------------------------------------------------*/
extern UART_HandleTypeDef hlpuart1;
extern I2C_HandleTypeDef hi2c1;
/******************************************************************************/
/*          LM75A i2c												          */
/******************************************************************************/
/**
  * @brief This function reads the temperature.
*/

double get_temperature()
{
	  //tell LM75 that we want to read from the temperature register
	  buf[0]= REG_TEMP;
	  ret = HAL_I2C_Master_Transmit(&hi2c1, LM75_ADDR, buf, 1, HAL_MAX_DELAY);

	  if(ret != HAL_OK)
	  {
		  strcpy((char *)buf, "Error Tx \r\n");
	  }
	  else
	  {
		  // receive data
		  ret = HAL_I2C_Master_Receive(&hi2c1, LM75_ADDR, buf, 2, HAL_MAX_DELAY);
		  if (ret != HAL_OK)
		  {
			  strcpy((char *)buf, "Error Rx \r\n");
		  }
		  else
		  {
			  // combine bytes
			  raw = buf[0]<<8 | buf[1];
			  temp_c = raw >> 5;
			  // to do: implement negative temperatures!
			  // to do: improve the code, please!
			  temp_c = temp_c*0.125;
		  }
	  }
	  HAL_UART_Transmit(&hlpuart1, buf, strlen((char *)buf), HAL_MAX_DELAY);
	  return temp_c;

}
