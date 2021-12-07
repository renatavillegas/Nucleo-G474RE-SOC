/*
 * File name: LM75.h
 *	Description: This file contains all the function prototypes for
  *          the LM75.c file

 *  Created on: Nov 6, 2021
 *      Author: renata
 *  Revision Date: Nov 6, 2021
 */

#ifndef INC_LM75_H_
#define INC_LM75_H_


/* Private defines -----------------------------------------------------------*/
#define LM75_ADDR  0x90 //LM75 Address
#define REG_TEMP  0x00 // write address


double getTemperature();

#endif /* INC_LM75_H_ */
