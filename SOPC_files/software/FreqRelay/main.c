/*
 * main.c
 *
 *  Created on: 17/04/2019
 *      Author: jhe654
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <tgmath.h>
#include "system.h"
#include "altera_avalon_pio_regs.h"
#include "FreeRTOS_Source/include/FreeRTOS.h"
#include "FreeRTOS_Source/include/task.h"
#include "FreeRTOS_Source/include/timers.h"
#include "FreeRTOS_Source/include/queue.h"
#include "Freertos_Source/include/semphr.h"
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
#define LoadManager_Task_P 		(tskIDLE_PRIORITY+5)
#define UpdateLED_Task_P 		(tskIDLE_PRIORITY+4)
#define UpdateThresholds_P		(tskIDLE_PRIORITY+4)
#define UpdateScreen_P			(tskIDLE_PRIORITY+4)
//#define SystemMonitor_P			(tskIDLE_PRIORITY)


/*Functions*/
//#define BGMASK				1 // enable this to print debug messages.
void initCreateTask(void);
void initSetupInterrupts(void);
void initSetupSystem(void);



/*Task and Definition of Task Priorities*/
#define   TASK_STACKSIZE       				2048
TaskHandle_t xHandle_thresholdUpdate;
TaskHandle_t xHandle_loadManager;
TaskHandle_t xHandle_freqAnalazer;
TaskHandle_t xHandle_systemCheck;

/*Enum and Constants*/
#define MaxRecordNum  5
typedef enum{ MAINTAIN=0, RUN=1, UNDEFINED =2} SystemMode;
typedef enum{UNSTABLE=0,STABLE=1,UNDEFINED_SysCon =2} SystemCondition;
const unsigned int NumLoads=5;
unsigned int redMask = 0x1F;
unsigned int redZeroMask = 0x0;
unsigned int redL1Mask = 0x10;
unsigned int redL2Mask = 0x8;
unsigned int redL3Mask = 0x4;
unsigned int redL4Mask = 0x2;
unsigned int redL5Mask = 0x1;

/*DataType*/
/*SysCond,
 * Stores system condition and the time it records
 * mode, the system operating mode at the time it was recorded,
 * time, the recording time
 */
//typedef struct SysCond{
//	SystemMode mode,
//	TickType_t time;
//}SysCond;

/* FreqInfo,
 * Stores individual frequency and the time it records
 * freq_value, the frequency value.
 * record_time, the time frequency value was recorded.
 */
typedef struct FreqInfo{
	double freq_value;
	TickType_t record_time;
}FreqInfo;
FreqInfo preFreq,curFreq;
FreqInfo historyFreq[MaxRecordNum];

/*Semaphore*/

/*Threshold*/
double Threshold_Freq = 50;
double Threshold_RoC = 5;

/*Global Variables
 * Use to store the frequency history
 * Graphic use*/
SystemCondition currentSysStability = STABLE;//1 for STABLE;
SystemMode CurSysMode = RUN;
//SystemMode PreSysMode;
char keyInputbuffer[10] ;
int keyInputIndex = 1;
int keyInputSum = 0;

//
//enum LoadStatus{
//	OFF=0,
//	ON,
//	SHED,
//} LoadBank[] = {ON,ON,ON,ON,ON};//NOTE: currently the uiLEDBank did the job

/*Record of current ON loads, in terms of LEDs*/
unsigned int uiLoadBank = 0;
unsigned int uiManageTime = 0;
unsigned int dropCounter = 0;

/*PreSysCon,
 * stores the previous system condition,
 * used to when to drop load
 */
SystemCondition PreSysCon= STABLE;//STABLE;

/*Timer*/
//TimeHandler_t timer_System; // use vTaskTickCount
TimerHandle_t timer_LoadManager;

/*Queues:*/
static QueueHandle_t Q_FreqInfo; //for record and analysis
static QueueHandle_t Q_KeyboardInput;
static QueueHandle_t Q_LoadOperation;
static QueueHandle_t Q_VGAUpdateValues;
static QueueHandle_t Q_VGAUpdateTime;


/*-------------Interrupts--------------------*/
/* PushBtn ISR,
 * use to go into maintenence mode
 */
void pushbutton_ISR(void* context, alt_u32 id)
{
//	printf("\n button interrupt triggers\n alt_u32 id = %d\n", id);
	// need to cast the context first before using it
	int* temp = (int*) context;
	(*temp) = IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE);

	switch(CurSysMode)
	{
	  case MAINTAIN : {
		  printf("CurSysMode = Run\n");
		  CurSysMode= RUN;
		  alt_irq_disable(PS2_IRQ);
//		  alt_irq_enable(FREQUENCY_ANALYSER_IRQ);
		  printf("PS2 irq disabled\n");
		  break;
	  }
	  case UNDEFINED : {
		  printf("CurSysMode have not been defined\n=");
		  break;
	  }
	  case RUN : {
		  printf("CurSysMode = Maintain\n");
		  CurSysMode= MAINTAIN;
		  alt_up_ps2_dev * ps2_device = alt_up_ps2_open_dev(PS2_NAME);
		  alt_up_ps2_clear_fifo (ps2_device) ; //clear keyboard input queue
		  alt_irq_enable(PS2_IRQ); //enable keyboard interrupt
//		  alt_irq_disable(FREQUENCY_ANALYSER_IRQ);
		  printf("PS2 irq enabled\n");
		  break;
	  }
	}
	(CurSysMode == RUN)? vTaskSuspend(xHandle_thresholdUpdate) : vTaskResume(xHandle_thresholdUpdate);
//	(CurSysMode == RUN)? vTaskResume(xHandle_loadManager) : vTaskSuspend(xHandle_loadManager);
//	(CurSysMode == RUN)? vTaskResume(xHandle_freqAnalazer) : vTaskSuspend(xHandle_freqAnalazer);
	// clears the edge capture register
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);

}
/* KeyboardISR,
 * called when keyboard is pressed
 */
void ps2_isr (void* context, alt_u32 id)
{
	printf("\nKeyPressed:\t");
	char ascii;
	int status = 0;
	unsigned char key = 0;
	KB_CODE_TYPE decode_mode;
	status = decode_scancode (context, &decode_mode , &key , &ascii) ;
//	int keyValue = 0;
	if ( status == 0 ) //success
	{
		xQueueSendToBackFromISR(Q_KeyboardInput, &key, pdFALSE);//TODO: using SEVEN-segment up store and update threshold
		printf ( "pushto Q_keyBoard  : hex:%x\t int:%d", key ,atoi(key)) ;
	}

	vTaskResume(xHandle_thresholdUpdate);
	printf("\n task notified\n");

	// print out the result
//		switch ( decode_mode )
//		{
//		  case KB_ASCII_MAKE_CODE :
//		  {
//			if(CurSysMode == MAINTAIN)
//			{
//				switch((int)key)
//				{
//				case 22:{keyValue = 1; break;}
//				case 30:{keyValue = 2; break;}
//				case 38:{keyValue = 3; break;}
//				case 37:{keyValue = 4; break;}
//				case 46:{keyValue = 5; break;}
//				case 54:{keyValue = 6; break;}
//				case 61:{keyValue = 7; break;}
//				case 62:{keyValue = 8; break;}
//				case 70:{keyValue = 9; break;}
//				case 90:{printf("Enter\n"); break;}
//
//				default:
//					break;
//				}
//				{
//					//when enter is not pressed
//					if( (int)key != 90)
//					{
//						if (keyInputIndex >= 0 ){
//							keyInputSum+= keyValue * pow(10,keyInputIndex);
//							--keyInputIndex;
//							printf("\t\tkeyInputSum=%d, keyInputIndex = %d\n",keyInputSum,keyInputIndex);
//							unsigned int hex=0;
//							itoa(keyInputSum,hex,16);
//							IOWR_ALTERA_AVALON_PIO_DATA(SEVEN_SEG_BASE,hex);
//						}else{
//							Threshold_Freq = keyInputSum;
//							printf("Threshold set to %f\n",Threshold_Freq);
//							keyInputIndex = 1;
//							keyInputSum = 0;
//						}
//
//					}
//				}
//				}
//			}
//
//			printf ( "ASCII   : %x\t", key ) ;
//			break ;
//		  default :
//			printf ( "Int Form : %d\n", key ) ;
//			break ;
//		}
//	}
}




/* Frequency relay ISR
 * Record frequency value and time on interrupt
 */
void freq_relay(void *pvParameters)
{
	FreqInfo freqData = {
			(double)16000/(double)IORD(FREQUENCY_ANALYSER_BASE, 0),
			xTaskGetTickCountFromISR()/1000
	};
//	printf("freeq_value = %f\t time=%d\n",freqData.freq_value, freqData.record_time);

	if (CurSysMode == RUN)
	{
		xQueueSendToBackFromISR( Q_FreqInfo, &freqData, pdFALSE );
		xQueueSendToBackFromISR( Q_VGAUpdateValues, &freqData.record_time, pdFALSE );
	}
	usleep(5000);
}

/*-------------TASKs--------------------*/
/* updateThreshold,
 * Since ps2_isr is collecting keyboard input into a buffer,
 * this update function scans the end of entry - a Enter '\n', ideally
 */
void updateThreshold(void *pvParameters0)
{
	double FREQ_CHANGE = 0.0;
	double ROC_CHANGE = 0.0;
	int numb = 0;
	int len = 2;
	int sum = 0;
	while(1)
	{
		printf("\nUpdateThreshold\n");
		if (CurSysMode == MAINTAIN)
		{
			if ( xQueueReceive(Q_KeyboardInput,&numb,portMax_DELAY) == pdTRUE )
			{
				printf("\t toke from Q_key : %d\n", atoi(numb));
				if (atoi(numb) == 90)
				{
					printf("\nEnter pressed!\n");
					Threshold_Freq = sum;
					printf("Threshold_Freq sets to:%f", Threshold_Freq);
					sum = 0; numb = 0; len =2;//reset
				}else{
//					sum+= atoi(numb)*pow(10,--len);
					printf("numb = %d\t sum=%d\t len = %d\n", atoi(numb), sum);
				}
			}
		}else{
			//wait
			vTaskDelay(500);
		}
	}
}

/* Frequency analyser
 * The core function of Frequency relay
 * Read from frequency queue, and calculate the rate of changes
 * Check wether the frequency is lower than frequency_threshold
 * OR the Rate of Change if above RoC threshold.
 */
void freq_analyser(void *pvParameters)
{
	printf("freq_analyser\n");
	double RoC = 0;
	int FreqHyIndex = 0;

	/* read from queue
	 * First check the current reading, if it's lower than threshold, call load ctr
	 * If the avaliable data is more than 2, calculate RoC
	 */
	while (1)
	{
		if (xQueueReceive( Q_FreqInfo, &curFreq, portMax_DELAY) == pdTRUE)
		{
//			printf("Readout from Q_FreqInfo\n \tFreq value:%f Time: %d\n",curFreq.freq_value, curFreq.record_time );
			if(curFreq.record_time == 0){//for first reading
				preFreq = curFreq;
			}
			RoC = (double)abs(curFreq.freq_value - preFreq.freq_value)/abs((double)curFreq.record_time - (double)preFreq.record_time);
		}
		/*Store readings to historyFreq[]*/
		historyFreq[FreqHyIndex%MaxRecordNum] = curFreq;
		if (FreqHyIndex != 5){
			++FreqHyIndex;
		}else{
			FreqHyIndex = 0;
		}
		if(curFreq.freq_value < Threshold_Freq || RoC > Threshold_RoC)
		{
			currentSysStability = UNSTABLE;//UNSTABLE;
			xQueueSendToBackFromISR(Q_LoadOperation,&currentSysStability, pdFALSE);
//			printf("\tRoc:%f freq:%f \tSys UNSTABLE\n",RoC,curFreq.freq_value);
		}else{
			currentSysStability = STABLE;//STABLE;
			xQueueSendToBackFromISR(Q_LoadOperation,&currentSysStability, pdFALSE);
//			printf("\tRoc:%f freq:%f \tSys STABLE\n",RoC,curFreq.freq_value);
		}
	}

}
/* Load manager
 * Control the load operation,
 * Read from Q_LoadOperation
 * ON,OFF,SHED
 */
void load_manager(void *pvParameters)
{
	SystemCondition sysCon = UNDEFINED_SysCon;
	while(1)
	{
		if(CurSysMode == RUN)
		{
			if (xQueueReceive(Q_LoadOperation,&sysCon,portMax_DELAY) == pdTRUE)
			{
	//			printf("Load_manager: sysCon:%d Loadbank=%d\t uiManageTime=%d dropCounter=%d freqThreshold=%d\n",sysCon,uiLoadBank,uiManageTime,dropCounter,Threshold_Freq);
				switch(sysCon)
				{
					case UNSTABLE: {
	//					printf("sysCon=UNSTABLE\n");
						if (sysCon != PreSysCon)
						{
							if (uiLoadBank == redMask)//check if all loads are present - If so,shed the first load(lowest priority)
							{
								/*Drop very first load, the load is not dropped until update_led process it */
	//							xSemaphoreTake(shareSource_LED_sem,portMax_DELAY);
								uiLoadBank -= (unsigned int)pow(2,dropCounter);
								++dropCounter;
	//							printf("\n##UNSTABLE-->dropLoad_FIRST, uiLoadBank=%d, dropCounter=%d ##\n",uiLoadBank,dropCounter);
	//							xSemaphoreGive(shareSource_LED_sem);
							}
							uiManageTime = xTaskGetTickCount();
						}else{
							if ( xTaskGetTickCount() - 500 > uiManageTime )
							{
								/*Drop loads after first load is dropped, 500ms waiting applies*/
	//							xSemaphoreTake(shareSource_LED_sem,portMax_DELAY);
								if (uiLoadBank != 0){
									uiLoadBank -= pow(2,dropCounter);
									++dropCounter;
	//								printf("\n##UNSTABLE-->dropLoad, uiLoadBank=%d, dropCounter=%d ##\n",uiLoadBank,dropCounter);
								}
	//							xSemaphoreGive(shareSource_LED_sem);
								uiManageTime = xTaskGetTickCount();
							}

						}
						PreSysCon = sysCon;
						break;
					}
					case STABLE: {
	//					printf("sysCon=STABLE\n");
						if (sysCon != PreSysCon)
						{
							/*Start put loads back on */
							uiManageTime = xTaskGetTickCount();	//Restart timer
						}else{
							if ( xTaskGetTickCount() - 500 > uiManageTime )
							{
	//							xSemaphoreTake(shareSource_LED_sem,portMax_DELAY);
								if (uiLoadBank != redMask)
								{
									--dropCounter;
									uiLoadBank += pow(2,dropCounter);//Add loads
//									printf("\n##STABLE-->Add loads, uiLoadBank=%d,dropCounter=%d ##\n",uiLoadBank,dropCounter);
								}
	//							xSemaphoreGive(shareSource_LED_sem);
								uiManageTime = xTaskGetTickCount();
							}
						}
						PreSysCon = sysCon;
						break;
					}

					case UNDEFINED:{
						printf("sysCon doesn't get value!!!\n");
						break;
					}
				}
			}
		}
	}
}
/* UpdateLED
 * Read the switch value
 * Turn on the LED if system is stable
 */
void update_LED(void *pvParameters)
{
	printf("update LED\n");
	unsigned int uiSwitchValue;
	unsigned int uiRedLED;
	unsigned int uiGreenLED;
//	unsigned int uiSKPGreenLED;
	/*indicate load condition with LEDs*/
	while(1)
	{
		/*If the system is stable, switch can be used to control load on/off correspondingly*/
//			xSemaphoreTake(shareSource_LED_sem,portMax_DELAY);
			uiSwitchValue = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE);
//			uiSKPGreenLED = ~(uiSwitchValue & redMask);
			uiRedLED = uiSwitchValue & uiLoadBank;
//			uiGreenLED = (uiLoadBank ^ redMask);
			uiGreenLED = (~uiRedLED & uiSwitchValue)&redMask ;
		if(currentSysStability == 1)
		{
			//For Most 5 right
			IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, uiRedLED);
			IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, uiGreenLED);
		}else{
			IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, uiRedLED);
			IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, uiGreenLED);
		}
//			xSemaphoreGive(shareSource_LED_sem);
	}
			printf("update_LED,system unstable");

}

/*System Check
 * Checking the system operation status and make correction
 */
void system_checker(void *pvParameters)
{

}

int main()
{
	printf("Hello Junjie!\n");
	initSetupSystem();
	printf("\tinitSetupSystem\n");
	initSetupInterrupts();
	printf("\tinitSetupInterrupts\n");
	initCreateTask();
	printf("\tinitCreateTask\n");
	vTaskStartScheduler();
	vTaskSuspend(xHandle_thresholdUpdate);
	while (1)
	{

	}

	return 0;
}

/*--------------------Inits--------------------*/
void initSetupSystem()
{
	//Start with Five loads, indicate by RED LEDs
	IOWR_ALTERA_AVALON_PIO_DATA(SEVEN_SEG_BASE,0);
	uiLoadBank = redMask;
	if(CurSysMode != MAINTAIN)
		IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, uiLoadBank);

	//Init Pushbtn
	// clears the edge capture register
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
	//Initiate Queues
	Q_FreqInfo = xQueueCreate(1000,sizeof(FreqInfo));
	Q_KeyboardInput = xQueueCreate(1000,sizeof(char));
	Q_LoadOperation = xQueueCreate(1000,sizeof(int));
	Q_VGAUpdateValues = xQueueCreate(1000,sizeof(unsigned char));//Change type
	Q_VGAUpdateTime = xQueueCreate(1000,sizeof(unsigned char));//change type
}
void initSetupInterrupts(void)
{
	/*Frequency Analyzer ISR*/
	alt_irq_register(FREQUENCY_ANALYSER_IRQ,(void*)0, freq_relay);

	/*Pushbutton interrupts,for maintenance mode*/
	int buttonValue = 0;
	// clears the edge capture register. Writing 1 to bit clears pending interrupt for corresponding button.
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
	// enable interrupts for all buttons
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(PUSH_BUTTON_BASE, 0x7);//enable interrupts for first btn on right side.
	// register the ISR
	alt_irq_register(PUSH_BUTTON_IRQ,(void*)&buttonValue, pushbutton_ISR);

	/*Keyboard interrupts*/
		alt_up_ps2_dev * ps2_device = alt_up_ps2_open_dev(PS2_NAME);
		if(ps2_device == NULL){
			printf("\ncan't find PS/2 device\n");
		}
		alt_up_ps2_clear_fifo (ps2_device) ;
		alt_irq_register(PS2_IRQ, ps2_device, ps2_isr); //only enabled in maintain mode
	(CurSysMode == MAINTAIN)? alt_irq_enable(PS2_IRQ) : alt_irq_disable(PS2_IRQ);

	IOWR_8DIRECT(PS2_BASE,4,1);
}
void initCreateTask()
{
	xTaskCreate(
			load_manager,
			"LoadManager_TASK",
			configMINIMAL_STACK_SIZE,
			NULL,
			LoadManager_Task_P,
			&xHandle_loadManager );

	xTaskCreate(
			freq_analyser,
			"freqRelay_task",
			configMINIMAL_STACK_SIZE,
			NULL,
			FreqAnalyser_Task_P,
			&xHandle_freqAnalazer );

	xTaskCreate(
			update_LED,
			"updateLED_task",
			configMINIMAL_STACK_SIZE,
			NULL,
			UpdateLED_Task_P,
			NULL );

	xTaskCreate(
			updateThreshold,
			"updateThreshold_task",
			configMINIMAL_STACK_SIZE,
			NULL,
			UpdateThresholds_P,
			&xHandle_thresholdUpdate );

	//UpdateThresholds_P
	//UpdateScreen_P

}

//int debugPrint(const &string str)
//{
//	if (BGMSK)
//	{
//		printf(str);
//		printf("\n");
//	}
//	return 0;
//}
