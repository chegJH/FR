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
#include <unistd.h>
#include "altera_avalon_pio_regs.h"
#include "altera_up_avalon_video_pixel_buffer_dma.h"
#include "altera_up_avalon_video_character_buffer_with_dma.h"
#include "altera_up_ps2_keyboard.h"		//keyboard isr
#include "alt_types.h"                 	// alt_u32 is a kind of alt_types
#include "sys/alt_irq.h"              	// to register interrupts

/*Task Priorities:*/
// #define Counter_Task_P      	(tskIDLE_PRIORITY)
#define FreqAnalyser_Task_P		(tskIDLE_PRIORITY+5)
#define LoadManager_Task_P 		(tskIDLE_PRIORITY+4)
#define UpdateLED_Task_P 		(tskIDLE_PRIORITY+3)
#define UpdateThresholds_P		(tskIDLE_PRIORITY+2)
#define UpdateScreen_P			(tskIDLE_PRIORITY+1)
/*Functions*/
//...
void initCreateTask(void);
void initSetupInterrupts(void);
void initSetupSystem(void);


// Definition of Task Stacks
#define   TASK_STACKSIZE       				2048
// Definition of Task Priorities
#define PRINT_STATUS_TASK_PRIORITY 			14
#define CHECKFREQ_GETFREQ_PRIORITY      	13
#define CHECKFREQ_UPDATELED_PRIORITY   		12

/*DataType*/
struct FreqInfo
{
	/* Frequency value and recording time */
	double freq_value;
	TickType_t record_time;
}
preFreq{0,0},
curFreq{0,0};

/*Enum and Constants*/
enum NumLoads = 5;
enum MaxRecordFreqs = 20;
typedef enum{ MAINTAIN=0, RUN} SystemMode;
typedef enum{true,false} bool;
typedef enum{OFF=0,ON,SHED} LoadState;
typedef enum{UNSTABLE,STABLE} SystemCondition;
const unsigned int redMask = 0x1F;
const unsigned int redL1Mask = 0x10;
const unsigned int redL2Mask = 0x8;
const unsigned int redL3Mask = 0x4;
const unsigned int redL4Mask = 0x2;
const unsigned int redL5Mask = 0x1;

/*Threshold*/
double Threshold_Freq = 50;
double Threshold_RoC = 5;

/*Global Variables
 * Use to store the frequency history
 * Graphic use*/
SystemMode currentSysMode = SystemMode::RUN;
FreqInfo Freq_History[MaxRecordFreqs];
int FreqHyIndex = 0;
LoadState LoadBank[NumLoads] = {ON,ON,ON,ON,ON};
SystemCondition currentSysStability = STABLE;
unsigned int LEDBank = redMask;

/*Timer*/
TimeHandler_t timer_System;
TimeHandler_t timer_LoadManager;

/*Queues:*/
static QueueHandle_t Q_FreqInfo;
static QueueHandle_t Q_KeyboardInput;
static QueueHandle_t Q_LoadOperation;
static QueueHandle_t Q_VGAUpdateValues;
static QueueHandle_t Q_VGAUpdateTime;

/*--------------------Inits--------------------*/
void initSetupSystem()
{	
	//Start with Five loads, indicate by RED LEDs
	//TODO:if not boot with maintainence mode
	if(currentSysMode != SystemMode::MAINTAIN)
		IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, LEDBank);

	//Initiate Queues
	Q_FreqInfo = xQueueCreate(100,sizeof(FreqInfo));
	Q_KeyboardInput = xQueueCreate(100,sizeof(SystemCondition));
	Q_LoadOperation = xQueueCreate(100,sizeof(unsigned char));
	Q_VGAUpdateValues = xQueueCreate(100,sizeof(unsigned char));//Change type
	Q_VGAUpdateTime = xQueueCreate(100,sizeof(unsigned char));//change type
}
void initSetupInterrupts(void)
{
	//Frequency Analyser ISR
	alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, freq_relay);

	//pushbtn interrupts for maintainence mode
	int buttonValue = 0;
	// clears the edge capture register. Writing 1 to bit clears pending interrupt for corresponding button.
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
	// enable interrupts for all buttons //TODO:specify wchich button.
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x7);
	// register the ISR
	alt_irq_register(PUSH_BUTTON_IRQ,(void*)&buttonValue, button_interrupts_function);

	//Keyboard interrupts
	alt_up_ps2_dev * ps2_device = alt_up_ps2_open_dev(PS2_NAME);

	if(ps2_device == NULL){
    	printf("can't find PS/2 device\n");
    	return 1;
	}
	alt_up_ps2_clear_fifo (ps2_device) ;

	alt_irq_register(PS2_IRQ, ps2_device, ps2_isr);
}
void initCreateTask()
{
	xTaskCreate(
			load_manager,
			"LoadManager_TASK",
			configMINIMAL_STACK_SIZE,
			NULL,
			LoadManager_Task_P,
			NULL );

	xTaskCreate(
			freq_analyser,
			"freqRelay_task",
			configMINIMAL_STACK_SIZE,
			NULL,
			FreqAnalyser_Task_P,
			NULL );

	xTaskCreate(
			update_LED,
			"updateLED_task",
			configMINIMAL_STACK_SIZE,
			NULL,
			UpdateLED_Task_P,
			NULL );
	
	//UpdateThresholds_P
	//UpdateScreen_P

}
/*-------------Features--------------------*/
/* PushBtn ISR,
 * use to go into maintenence mode 
 */
void button_interrupts_function(void* context, alt_u32 id)
{
  // need to cast the context first before using it
  int* temp = (int*) context;
  (*temp) = IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE);

  // clears the edge capture register
  IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);

  //Set System mode
  switch(currentSysMode) {
	  case SystemMode::RUN : {
		  currentSysMode=SystemMode::MAINTAIN;
		  break;
	  }
	  case SystemMode::MAINTAIN : {
		  currentSysMode=SystemMode::RUN;
		  break;
	  }
  }
}
/* KeyboardISR,
 * called when keyboard is pressed
 */
void ps2_isr (void* context, alt_u32 id)
{
  char ascii;
  int status = 0;
  unsigned char key = 0;
  KB_CODE_TYPE decode_mode;
  status = decode_scancode (context, &decode_mode , &key , &ascii) ;
  if ( status == 0 ) //success
  {
    // print out the result
    switch ( decode_mode )
    {
      case KB_ASCII_MAKE_CODE :
	  	xQueueSendToBackFromISR(Q_KeyboardInput, &key, pdFALSE);
        printf ( "ASCII   : %x\n", key ) ;
        break ;
      default :
        printf ( "DEFAULT   : %x\n", key ) ;
        break ;
    }
    IOWR(SEVEN_SEG_BASE,0 ,key);
  }
}
//TODO, followed keyboard isr, update freq threshold
//Created this feature in individual function or combine in ps2_isr?
void updateThreshold()
{


}
/* Frequency relay ISR
 * Record frequency value and time on interupt
 */
void freq_relay(void *pvParameters)
{
	FreqInfo freqData = {
		xTaskGetTickCountFromISR(),
		(16000/(double)IORD(FREQUENCY_ANALYSER_BASE, 0));
	}
	if (SystemMode == RUN)
	{
		xQueueSendToBackFromISR( Q_FreqInfo, &freqData, pdFALSE );
		xQueueSendToBackFromISR( Q_VGAUpdateValues, &freqData.record_time, pdFALSE );
	}
	
/* Frequency analyser
 * The core function of Frequency relay
 * Read from frequency queue, and calculate the rate of changes
 * Check wether the frequency is lower than frequency_threshold
 * OR the Rate of Change if above RoC threshold.
 */
void freq_analyser(void *pvParameters)
{
	double RoC = 0;

	/* read from queue
	 * First check the current reading, if it's lower than threshold, call load ctr
	 * If the avaliable data is more than 2, calculate RoC
	 */
	while (1)
	{
		if (xQueueReceive( Q_FreqInfo, &curFreq, portMax_DELAY) == pdTRUE)
		{
			if(curFreq.record_time == 0){//for first reading
				preFreq = curFreq;
			}
			RoC = abs(curFreq.freq_value - preFreq.freq_value)/abs(curFreq.record_time - preFreq.record_time);
		}
		/*Store readings to Freq_History[]*/
		Freq_History[FreqHyIndex%MaxRecordFreqs] = curFreq;
		
		if(curFreq.freq_value > Threshold_Freq || RoC > Threshold_RoC)
		{
			xQueueSendToBackFromISR(Q_LoadOperation,SystemCondition::UNSTABLE, pdFALSE);
			currentSysStability = SystemCondition::UNSTABLE;
		}else{
			xQueueSendToBackFromISR(Q_LoadOperation,SystemCondition::STABLE, pdFALSE);
			currentSysStability = SystemCondition::STABLE;
		}
	}	
	//calculate the RoC
	//TODO: implement system timer
	
}

/* Load manager
 * Control the load operation,
 * Read from Q_LoadOperation
 * ON,OFF,SHED
 */
void load_manager(void *pvParameters)
{
	printf("Load manager");
	SystemCondition tempSysCon;
	while(1)
	{
		if (xQueueReceive(Q_LoadOperation,&tempSysCon,portMax_DELAY) == pdTrue)
		{
			switch(tempSysCon){
				case UNSTABLE: {
					printf("Take one load off");
					//TODO: need a timer here to count 500 ms before bring load backon?
					break;
				}
				case STABLE: {
					printf("Take one load back");					
					break;
				}
			}
			
		}
	}
	printf("Load manager Error");
}
/* UpdateLED
 * Read the switch value 
 * Turn on the LED if system is stable
 */
void update_LED(void *pvParameters)
{
	while(currentSysStability)
	{
		//read switch value
		uiSwitchValue = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE);
		//For Most 5 right
		LEDBank = LEDBank & uiSwitchValue; 

		// write the value of the switches to the red LEDs
		IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, LEDBank);
	}
	printf("update_LED,system unstable, skipped");
}


int main()
{
	printf("Hello Junjie!\n");
	initSetupSystem();
	initSetupInterrupts();
	initCreateTask();
	vTaskStartScheduler();
	while (1)
	{

	}

	return 0;
}
