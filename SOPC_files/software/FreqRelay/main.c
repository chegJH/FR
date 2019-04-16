/*
 * main.c
 *
 *  Created on: Apr 15, 2019
 *      Author: Junjie He
 */

#include <stdio.h>
#include <stdlib.h>
#include "system.h"
#include "altera_avalon_pio_regs.h"
#include "../FreeRTOS/FreeRTOS.h"
#include "../FreeRTOS/task.h"
#include "../FreeRTOS/timers.h"
#include "../FreeRTOS/queue.h"
#include "../Freertos/semphr.h"
#include "io.h"
#include "altera_up_avalon_ps2.h"
#include "altera_up_ps2_keyboard.h"
#include <unistd.h>
#include "altera_avalon_pio_regs.h"
#include "altera_up_avalon_video_pixel_buffer_dma.h"
#include "altera_up_avalon_video_character_buffer_with_dma.h"

//Task Priorities:
#define Counter_Task_P      	(tskIDLE_PRIORITY)
#define FreqDisp_Task_P      	(tskIDLE_PRIORITY)
#define UpdateLoads_Task_P 		(tskIDLE_PRIORITY+3)
#define UpdateThresholds_P		(tskIDLE_PRIORITY+2)
#define UpdateScreen_P			(tskIDLE_PRIORITY+1)
//Functions
//...
void initCreateTask(void);
static void counter_task(void *pvParameters);
static void freqRelay_task(void *pvParameters);

// Definition of Task Stacks
#define   TASK_STACKSIZE       				2048
// Definition of Task Priorities
#define PRINT_STATUS_TASK_PRIORITY 			14
#define CHECKFREQ_GETFREQ_PRIORITY      	13
#define CHECKFREQ_UPDATELED_PRIORITY   		12

void updateLCD_TASK(){
	// the pointer to the lcd
	FILE *lcd;
	// open the character LCD
	lcd = fopen(CHARACTER_LCD_NAME, "w");
	// if the lcd is open successfully
	if(lcd != NULL)
	{
		// print the value of the buttons in the character lcd
		#define ESC 27
		#define CLEAR_LCD_STRING "[2J"
//		fprintf(lcd, "%c%s", ESC, CLEAR_LCD_STRING);
//		fprintf(lcd, "Net Freq: %d\n", freq_relay());
	}
}
//counting numbers
static void counter_task(void *pvParameters)
{
	int count = 0;
	count++;
	IOWR(SEVEN_SEG_BASE, 0, count);
//	vTaskDelay(500);
}
//show current freq on lcd
static void freqRelay_task(void *pvParameters)
{
	unsigned int temp = IORD(FREQUENCY_ANALYSER_BASE, 0);
	FILE* lcd;
	lcd = fopen(CHARACTER_LCD_NAME,"w");
	if (lcd != NULL)
	{
		fprintf(lcd, "%c%s", ESC, CLEAR_LCD_STRING);
		fprintf(lcd, "BUTTON VALUE: %d\n", &temp);
	}
	printf("freq:%d\n",temp);
}

void initCreateTask()
{
	xTaskCreate(
			counter_task,
			"task1",
			configMINIMAL_STACK_SIZE,
			NULL,
			Counter_Task_P,
			NULL );

	xTaskCreate(
			freqRelay_task,
			"task2",
			configMINIMAL_STACK_SIZE,
			NULL,
			FreqDisp_Task_P,
			NULL );

}
int main()
{
	printf("Hello from Nios II!\n");
	initCreateTask();
	vTaskStartScheduler();
	while (1)
	{

	}

	return 0;
}
