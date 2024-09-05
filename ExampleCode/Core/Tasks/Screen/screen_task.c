/*
 * screen_task.c
 *
 *  Created on: Sep 4, 2024
 *      Author: bozse
 */
//Includes
#include "main.h"
#include "cmsis_os.h"
#include "screen_task.h"
#include "string.h"

//Definitions
#define LCD_ADDRESS 0x4E//0x7E
#define ROW_SIZE 2
#define COL_SIZE 16

//Local function prototypes
static void SendCommand4BitFrame(uint8_t cmd);
static void SendData4BitFrame(uint8_t cmd);
static void LcdInit(void);
static void LcdSendString (char *str);
static void LcdSetCursr(int row, int col);
static void UpdateLcd(void);
static void delayUs(uint16_t time);
//Global variables

//Local variables
static I2C_HandleTypeDef * l_psti2chandle = NULL;
static osMessageQueueId_t * l_pvQueue = NULL;
static osMutexId_t * l_pvMutex = NULL;
static uint32_t l_ulI2CErrorCounter = 0;
static uint32_t l_ulQueueErrorCounter = 0;
static uint32_t l_ulMutexErrorCounter = 0;
static uint8_t l_ucInit = 0;

static char l_cLcdData[ROW_SIZE][COL_SIZE] = {
		{'H', 'E', 'L', 'P', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		{'M', 'E', 'E', 'E', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

//STATIC function definitions
static volatile uint16_t l_ucTime;
static void delayUs(uint16_t time)
{
	uint16_t i;
	l_ucTime = time;
	for(i = 0; i < time; i++)
	{
		l_ucTime++;
		l_ucTime--;
		l_ucTime++;
		l_ucTime--;
		l_ucTime++;
		l_ucTime--;
		l_ucTime++;
		l_ucTime--;
	}
}

static void SendCommand4BitFrame(uint8_t cmd)
{
	uint8_t data[4];
	uint8_t size = 0;
	char data_u, data_l;
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data[0] = data_u|0x0C;  //en=1, rs=0 -> bxxxx1100
	data[1] = data_u|0x08;  //en=0, rs=0 -> bxxxx1000
	data[2] = data_l|0x0C;  //en=1, rs=0 -> bxxxx1100
	data[3] = data_l|0x08;  //en=0, rs=0 -> bxxxx1000


	while(l_psti2chandle->State != HAL_I2C_STATE_READY)
	{
		osDelay(1);
	}
	if(l_ucInit == 0)
	{
		size = 2;
	}
	else
	{
		size = 4;
	}
	if(HAL_I2C_Master_Transmit(l_psti2chandle,LCD_ADDRESS,&data[0], size, 1) != HAL_OK)
	{
		l_ulI2CErrorCounter++;
	}
}

static void SendData4BitFrame(uint8_t cmd)
{
	uint8_t data[4];
	char data_u, data_l;
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data[0] = data_u|0x0D;  //en=1, rs=1 -> bxxxx1101
	data[1] = data_u|0x09;  //en=0, rs=1 -> bxxxx1001
	data[2] = data_l|0x0D;  //en=1, rs=1 -> bxxxx1101
	data[3] = data_l|0x09;  //en=0, rs=1 -> bxxxx1001

	while(l_psti2chandle->State != HAL_I2C_STATE_READY)
	{
		osDelay(1);
	}
	if(HAL_I2C_Master_Transmit(l_psti2chandle,LCD_ADDRESS,&data[0], 4, 1) != HAL_OK)
	{
		l_ulI2CErrorCounter++;
	}
}

static void LcdInit (void)
{
  // 4 bit initialisation
  l_ucInit = 1;
  osDelay(40);
  SendCommand4BitFrame (0x30);
  osDelay(5);  // wait for >4.1ms
  SendCommand4BitFrame (0x30);
  osDelay(1);  // wait for >100us
  SendCommand4BitFrame (0x20);
  osDelay(10);
  SendCommand4BitFrame (0x20);  // 4bit mode
  osDelay(10);
  l_ucInit = 0;

  // display initialisation
  SendCommand4BitFrame (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
  osDelay(1);
  SendCommand4BitFrame (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
  osDelay(1);
  SendCommand4BitFrame (0x01);  // clear display
  osDelay(2);
  SendCommand4BitFrame (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
  osDelay(1);
  SendCommand4BitFrame (0x0c); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
}

static void LcdSendString (char *str)
{
	uint8_t i;
	for(i = 0; i < COL_SIZE; i++)
	{
		SendData4BitFrame(str[i]);
		osDelay(1);
	}
}

static void LcdSetCursr(int row, int col)
{
	switch (row)
	{
		case 0:
			col |= 0x80;
			break;
		case 1:
			col |= 0xC0;
			break;
	}
	SendCommand4BitFrame (col);
}

static void UpdateLcd(void)
{
	LcdSetCursr(0, 0);
	LcdSendString (&l_cLcdData[0][0]);
	LcdSetCursr(1, 0);
	LcdSendString(&l_cLcdData[1][0]);
}


//INTERFACE FUNCTIONS
void TASK_SCREEN_Init(I2C_HandleTypeDef * p_i2c, void * p_queue, void * p_mutex)
{
	l_psti2chandle = p_i2c;
	l_pvQueue = (osMessageQueueId_t *)p_queue;
	l_pvMutex = (osMutexId_t *)p_mutex;
	//l_ucDevAddress = FindAddress(p_i2c);
	LcdInit();
}

void TASK_SCREEN_Run(void)
{
	TASK_SCREEN_MESSAGE_ tmp = {0};
	//Eventually add a queue to read from.
	if(osMessageQueueGet(*l_pvQueue,(void *)&tmp, NULL, 30) == osOK)
	{
		memcpy(&l_cLcdData[tmp.row][tmp.col_start],
			   &tmp.column[tmp.col_start],
			   tmp.col_end-tmp.col_start);
	}
	UpdateLcd();
	osDelay(1);
}

void TASK_SCREEN_UpdateScreen(TASK_SCREEN_MESSAGE_ * p_message)
{
	if(osMutexAcquire(*l_pvMutex,1) == osOK)
	{
		if(osMessageQueuePut(*l_pvQueue,(void *)p_message, osPriorityNormal, 10) != osOK)
		{
			l_ulQueueErrorCounter++;
		}
		osMutexRelease(*l_pvMutex);
	}
	else
	{
		l_ulMutexErrorCounter++;
	}
}



