/*
 * screen_task.h
 *
 *  Created on: Sep 4, 2024
 *      Author: bozse
 */

#ifndef TASKS_SCREEN_SCREEN_TASK_H_
#define TASKS_SCREEN_SCREEN_TASK_H_

#include "screen_task_constants.h"



/*
   This struct helps us constructing the I2C output based on data and control outputs.
   Because the LCD is set to 4-bit mode, 4 bits of the I2C output are for the control outputs
   while the other 4 bits are for the 8 bits of data which are send in parts using the enable output.
*/
typedef struct{
	uint8_t row;
	uint8_t col_start;
	uint8_t col_end;
	uint8_t column[16];
}TASK_SCREEN_MESSAGE_;

void TASK_SCREEN_Init(I2C_HandleTypeDef * p_i2c, void * p_queue, void * p_mutex);
void TASK_SCREEN_Run(void);
void TASK_SCREEN_UpdateScreen(TASK_SCREEN_MESSAGE_ * p_message);









#endif /* TASKS_SCREEN_SCREEN_TASK_H_ */
