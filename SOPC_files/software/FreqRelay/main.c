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
#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include "FreeRTOS/timers.h"
#include "FreeRTOS/queue.h"
#include "Freertos/semphr.h"
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
#define UpdateLED_Task_P 		(tskIDLE_PRIORITY+5)
#define LoadManager_Task_P 		(tskIDLE_PRIORITY+4)
#define UpdateThresholds_P		(tskIDLE_PRIORITY+5)
#define UpdateScreen_P			(tskIDLE_PRIORITY+4)
#define SystemChecker_Task_P 	(tskIDLE_PRIORITY+1)


/*Functions*/
//#define BGMASK				1 // enable this to print debug messages.
void initTask(void);
void initInterrupts(void);
void initSystem(void);
void initSharedResources(void);
static void initKeyBDISR(void);



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
static unsigned char esc = 0x1b;

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
SemaphoreHandle_t SharedSource_KB_update; //use semaphore to control update procedure.
SemaphoreHandle_t SharedSource_LCD_show; //use to gard lcd usage.
SemaphoreHandle_t xSemaphore_Freq_analyzer;
SemaphoreHandle_t xSemaphore_Load_manager; // use with load manager
SemaphoreHandle_t xSemaphore_LED_updater;
UBaseType_t uxMaxCount_sem=50;
UBaseType_t uxInitialCount=0;
/*Threshold*/
double Threshold_Freq = 50;
double Threshold_RoC = 5;
bool Threshold_dec = false;

/*Global Variables
 * Use to store the frequency history
 * Graphic use*/
SystemCondition currentSysStability = STABLE;//1 for STABLE;
SystemMode CurSysMode = RUN;
char keyInputbuffer[10] ;
int keyInputIndex = 1;
int keyInputSum = 0;
char keyboardRaiseEdge = false;
/*Timer*/
TimerHandle_t xTimer_LoadShed;
//TimerHandle_t xTimer_
/*For Threshold setting,
 * setFreqOrRoc = r/R for setting RoC threshold
 * setFreqOrRoc = f/F for setting Frequency threshold
 */
char setFreqOrRoc = 'N';

/*Record of current ON loads, in terms of LEDs*/
unsigned int uiLoadBank = 0;
unsigned int uiManageTime = 0;
TickType_t xTimer1 ; //stores the time cost of load dropping..
TickType_t xTimer2;
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
 * use to go into maintenance mode
 */
void pushbutton_ISR(void* context, alt_u32 id)
{
//	printf("\n button interrupt triggers\n alt_u32 id = %d\n", id);
	// need to cast the context first before using it
	int* temp = (int*) context;
	(*temp) = IORD_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE);
	xSemaphoreTakeFromISR(SharedSource_LCD_show,pdFALSE);
	FILE* lcd = fopen(CHARACTER_LCD_NAME, "w");
	switch(CurSysMode)
	{
	  case MAINTAIN : {
		  printf("CurSysMode = Run\n");
		  CurSysMode= RUN;
		  alt_irq_disable(PS2_IRQ);
		  fprintf(lcd, "%c RUN mode",esc);
		  fclose(lcd);
		  xSemaphoreGiveFromISR(SharedSource_LCD_show,pdFALSE);
		  break;
	  }
	  case UNDEFINED : {
		  printf("CurSysMode have not been defined\n=");
		  break;
	  }
	  case RUN : {
		  printf("CurSysMode = Maintain\n");
		  CurSysMode = MAINTAIN;
		  fprintf(lcd, "%c%s %s",esc,"[2J","Maintenance\nEnter r or f\n");
		  fclose(lcd);
		  alt_irq_enable(PS2_IRQ);
		  xSemaphoreGiveFromISR(SharedSource_KB_update,UpdateThresholds_P);
		  printf("SEM given in pushbtn fn\n");
		  xSemaphoreGiveFromISR(SharedSource_LCD_show,pdFALSE);
		  break;
	  }
	}

	// clears the edge capture register
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
//	vTaskNotifyGiveFromISR(xHandle_systemCheck,SystemChecker_Task_P);
	return;
}
/* KeyboardISR,
 * called when keyboard is pressed
 * fill the Q_KeyboardInput with pressed raw value
 */
void ps2_isr (void* context, alt_u32 id)
{
	printf("\nps2_isr:\t");
	unsigned char byte, temp;
	temp = byte;
	alt_up_ps2_dev * ps2_device = alt_up_ps2_open_dev(PS2_NAME);
	alt_up_ps2_read_data_byte_timeout(ps2_device, &byte);
	printf("Scan code: %x\n", byte);
	//filte out invalid values
	switch (byte){
		case 0x16:
		case 0x1e:
		case 0x26:
		case 0x25:
		case 0x2e:
		case 0x36:
		case 0x3d:
		case 0x3e:
		case 0x46:
		case 0x45:
		{
			if (keyboardRaiseEdge)
			{
				keyboardRaiseEdge = false;
				xQueueSendToBackFromISR(Q_KeyboardInput, &byte,pdFALSE);
				break;
			}else{
				keyboardRaiseEdge = true;
				return;
				break;
			}
		}
	}
	//1to0	16,1e,26,25,...etc
	//R = 0x2d
	//F = 0x2b
	//Enter = 0x5a
	FILE* lcd = fopen(CHARACTER_LCD_NAME, "w");
	if (byte == 0x5a){
		//TODO: notify the updateThreshold task to run.
		fclose(lcd);
		return;
	}
	else if ( byte == 0x2d){
		printf("\t setting for Roc threshold\n");
		fprintf(lcd,"%c%s %s",esc,"[2J","Set Roc \n");
		fclose(lcd);
		setFreqOrRoc = 'r';
		return;
	}else if( byte == 0x2b){
		printf("\t setting for Frequency threshold\n");
		fprintf(lcd,"%c%s %s",esc,"[2J","Set Freq \n");
		fclose(lcd);
		setFreqOrRoc = 'f';
		return;
	}else if ( byte == 0x49){
		printf("\t DOT\n");
		Threshold_dec = true;
		fclose(lcd);
		return;
	}
	return;
}



/* Frequency relay ISR
 * Record frequency value and time on interrupt
 * Stores data into Q_FreqInfo
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
        xSemaphoreGiveFromISR(xSemaphore_Freq_analyzer,FreqAnalyser_Task_P);
	}else if(CurSysMode == MAINTAIN)
	{
		/* In maintain mode, the frequency info is only send to queue for VGA display*/
		//TODO: send freq and time for plot
		xQueueSendToBackFromISR( Q_VGAUpdateValues, &freqData.record_time, pdFALSE );
	}
//	usleep(5000);
	return;
}

/*-------------TASKs--------------------*/
/* updateThreshold,
 * Read from Q_KeyboardInput, figure out new threshold
 * New value end with "ENTER" key
 */
void updateThreshold(void *pvParameters)
{
	int updateCounter = 0;
	while(1)
	{
		printf("\nUpdateThreshold\t");
		xSemaphoreTake(SharedSource_KB_update, portMax_DELAY);
		char key_q = 0;
		int i_index = 0, j_index = 0;
		char temp[10], result[5];
		while(uxQueueMessagesWaiting(Q_KeyboardInput) != pdFALSE)
		{
			xQueueReceive(Q_KeyboardInput,&key_q,portMax_DELAY);
			int numb = 0;
			switch (key_q)
			{
				case 0x16: 	{ numb = 1; break;}
				case 0x1e:  { numb = 2; break;}
				case 0x26:  { numb = 3; break;}
				case 0x25:  { numb = 4; break;}
				case 0x2e:  { numb = 5; break;}
				case 0x36:  { numb = 6; break;}
				case 0x3d:  { numb = 7; break;}
				case 0x3e:  { numb = 8; break;}
				case 0x46:  { numb = 9; break;}
				case 0x45:  { numb = 0; break;}
			}
			temp[i_index]= numb;
			++i_index;//length of input
		}
		
		/*Calculate for threshold*/
		int thre = 0;
		for (int i=0,j=i_index;i<i_index,j>0;++i,--j)
		{
			thre += i * pow(10,j); //31 = 3*pow(10,1) + 1*pow(10,0)
		}
		printf("threshold=%f",(double)(thre/10));
// 		for( int i=0,j=0; i<i_index;++i)
// 		{
// 			if(i%2 == 0)
// 			{
// 				result[j] = temp[i];
// 				j_index = ++j;
// 			}
// 		}
		/*take every two element from array*/
// 			int threshold = 0;
// 			for( int  i = 0 , j = j_index ; i < j_index, j > 0; ++i , --j )
// 			{
// 				printf("result[%d]=%d ",i,result[i]);
// 				threshold += result[i]*pow(10,j);
// 			}
//			printf("threshold=%f",(double)(threshold/10));
		
		/*prepare LCD for display threshold*/
		xSemaphoreTake(SharedSource_LCD_show,pdFALSE);
		FILE* fp;
		fp = fopen(CHARACTER_LCD_NAME, "w"); //open the character LCD as a file stream for write

		if (fp == NULL) {
			printf("open failed\n");
		}
		if (setFreqOrRoc == 'r')
		{
			Threshold_RoC = (Threshold_dec)? (double)(threshold/100.0) : (double)(threshold/10.0) ;
			fprintf(fp, "%c%s RoC:%.1f\n Freq:%.1f\n", esc,"[2J",Threshold_RoC,Threshold_Freq);
			printf("ROC=%f",Threshold_RoC);
		}
		else if(setFreqOrRoc == 'f')
		{
			Threshold_Freq = (Threshold_dec)? (double)(threshold/100.0) : (double)(threshold/10.0) ;
			fprintf(fp, "%c%s RoC:%.1f\n Freq:%.1f\n", esc,"[2J",Threshold_RoC,Threshold_Freq);
			printf("Freq=%f",Threshold_Freq);
		}
		else
		{
			printf("\n##ENTER r for ROC, f for Frequency\n");
		}
		setFreqOrRoc = 'N';
		fclose(fp);
		xSemaphoreGive(SharedSource_LCD_show);
		++updateCounter;
		Threshold_dec = false;
//		printf("updateCounter = %d\n",updateCounter);
	}
//	vTaskDelay(5000);
	return;
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
	bool isFirstLoadDrop = true;
	/* read from queue
	 * First check the current reading, if it's lower than threshold, call load ctr
	 * If the available data is more than 2, calculate RoC
	 */
	while (1)
	{
		xSemaphoreTake(xSemaphore_Freq_analyzer,portMax_DELAY);
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
//			if (isFirstLoadDrop && curFreq.record_time > 0){
//				xTimer1 = xTaskGetTickCount();
//				isFirstLoadDrop = false;
//				printf("\nxTimer1 = %d\n",(int)xTimer1);
//			}
			xSemaphoreGive(xSemaphore_Load_manager);
		}else{
			currentSysStability = STABLE;//STABLE;
			xQueueSendToBackFromISR(Q_LoadOperation,&currentSysStability, pdFALSE);
//			printf("\tRoc:%f freq:%f \tSys STABLE\n",RoC,curFreq.freq_value);
			xSemaphoreGive(xSemaphore_Load_manager);
		}
	}
	return;

}
/* Load manager
 * Control the load operation,
 * Read from Q_LoadOperation
 * ON,OFF,SHED
 */
void load_manager(void *pvParameters)
{
	bool isFirstLoadDrop = true;
	SystemCondition sysCon = UNDEFINED_SysCon;
	while(1)
	{
        xSemaphoreTake(xSemaphore_Load_manager,portMax_DELAY);
		if(CurSysMode == RUN)
		{
			if (xQueueReceive(Q_LoadOperation,&sysCon,portMax_DELAY) == pdTRUE)
			{
	//			printf("Load_manager: sysCon:%d Loadbank=%d\t uiManageTime=%d dropCounter=%d freqThreshold=%d\n",sysCon,uiLoadBank,uiManageTime,dropCounter,Threshold_Freq);
				switch(sysCon)
				{
					case UNSTABLE: {
						if (sysCon != PreSysCon)
						{
							if (uiLoadBank == redMask)//check if all loads are present - If so,shed the first load(lowest priority)
							{
								/*Drop very first load, the load is not dropped until update_led process it */
								uiLoadBank -= (unsigned int)pow(2,dropCounter);
								if (isFirstLoadDrop){
									xTimer1 = xTaskGetTickCount();
									isFirstLoadDrop = false;
								}
								++dropCounter;
	//							printf("\n##UNSTABLE-->dropLoad_FIRST, uiLoadBank=%d, dropCounter=%d ##\n",uiLoadBank,dropCounter);
								xSemaphoreGive(xSemaphore_LED_updater);							}
							uiManageTime = xTaskGetTickCount();
						}else{
							if ( xTaskGetTickCount() - 500 > uiManageTime )
							{
								/*Drop loads after first load is dropped, 500ms waiting applies*/
								if (uiLoadBank != 0){
									uiLoadBank -= pow(2,dropCounter);
									++dropCounter;
	//								printf("\n##UNSTABLE-->dropLoad, uiLoadBank=%d, dropCounter=%d ##\n",uiLoadBank,dropCounter);
								}
								uiManageTime = xTaskGetTickCount();
							}
							xSemaphoreGive(xSemaphore_LED_updater);
						}
						PreSysCon = sysCon;
						break;
					}
					case STABLE: {
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
						xSemaphoreGive(xSemaphore_LED_updater);
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
	return;
}
/* UpdateLED
 * Read the switch value
 * Turn on the LED if system is stable
 */
void update_LED(void *pvParameters)
{
	bool isFirstUpdate = true;
	printf("update LED\n");
	unsigned int uiSwitchValue;
	unsigned int uiRedLED;
	unsigned int uiGreenLED;
	/*indicate load condition with LEDs*/
	while(1)
	{
		xSemaphoreTake(xSemaphore_LED_updater,portMax_DELAY);
		/*If the system is stable, switch can be used to control load on/off correspondingly*/
			uiSwitchValue = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE);
		if( CurSysMode == RUN){
			uiRedLED = uiSwitchValue & uiLoadBank;
			uiGreenLED = (~uiRedLED & uiSwitchValue)&redMask ;
			if(currentSysStability == 1)
			{
				//For Most 5 right
				IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, uiRedLED);
				IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, uiGreenLED);
			}else{
				IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, uiRedLED);
				if (isFirstUpdate && xTimer1 != 0){
					xTimer2 = xTaskGetTickCount();
					printf("First Load Drop Cost:%d ms\n",(int)(xTimer2 - xTimer1));
					isFirstUpdate = false;
				}
				IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, uiGreenLED);
			}
		}else{
			/*In Maintenance mode, Loads are controlled by switches no matter what in the uiLoadBank at the moment
			 * Update the uiLoadBank after control*/
			uiRedLED = uiSwitchValue & redMask;
			unsigned int diff = uiLoadBank ^ uiSwitchValue;
			IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, uiRedLED);
			uiLoadBank = uiRedLED & redMask;
			int itr = 0;
			for(itr = 0; diff != 0x1; diff = diff >> 1,++itr);
			dropCounter = itr;
			printf("\ndropCounter  = %d\n",dropCounter);
		}
	}
			printf("update_LED,system unstable");
			return;
}

/*System Check
 * Checking the system operation status and make correction
 * Manage updateThreshold operation
 */
void system_checker(void *pvParameters)
{
	SystemMode prevMode;
	while(1)
	{
		printf("\n System Checker\t");

		if (CurSysMode != prevMode)
		{
			if (CurSysMode == RUN)
			{
				alt_irq_disable(PS2_IRQ);
				printf("\n Disable ps2_irq\n");
				vTaskSuspend(xHandle_thresholdUpdate);
				printf("thresholdUpdate suspended\n");
				prevMode = CurSysMode;
			}
			/*In Maintenance mode*/
			else
			{
				initKeyBDISR();
				vTaskResume(xHandle_thresholdUpdate);
				printf("thresholdUpdate resumed\n");
				prevMode = CurSysMode;
			}
		}
//		vTaskSuspend(NULL);
	}
	return;
}

int main()
{
	printf("Hello Junjie!\n");
	initSystem();
	printf("\tinitSetupSystem\n");
	initInterrupts();
	printf("\tinitSetupInterrupts\n");
	initTask();
	printf("\tinitCreateTask\n");
	initSharedResources();
//	xSemaphoreTake(SharedSource_KB_update,portMax_DELAY);
	printf("\tinitSharedResources\n");
	vTaskStartScheduler();
	while (1)
	{

	}

	return 0;
}

/*--------------------Inits--------------------*/
void initSystem()
{
	/*Start with Five loads, indicate by RED LEDs*/
		FILE* lcd = fopen(CHARACTER_LCD_BASE,'w');
		fprintf(lcd,"%c%s Maintenance\n",esc,"[2J");
		fclose(lcd);
		uiLoadBank = redMask;
		if(CurSysMode != MAINTAIN)
			IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, uiLoadBank);

	/*Init Pushbtn*/
		// clears the edge capture register
		IOWR_ALTERA_AVALON_PIO_EDGE_CAP(PUSH_BUTTON_BASE, 0x7);
	/*Initiate Queues*/
		Q_FreqInfo = xQueueCreate(1000,sizeof(FreqInfo));
		Q_KeyboardInput = xQueueCreate(1000,sizeof(char));
		Q_LoadOperation = xQueueCreate(1000,sizeof(int));
		Q_VGAUpdateValues = xQueueCreate(1000,sizeof(unsigned char));//Change type
		Q_VGAUpdateTime = xQueueCreate(1000,sizeof(unsigned char));//change type
		return;
}
void initInterrupts(void)
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
		initKeyBDISR();
		alt_irq_disable(PS2_IRQ);
		return;
}
static void initKeyBDISR()
{
	/*Keyboard interrupts*/
		alt_up_ps2_dev * ps2_device = alt_up_ps2_open_dev(PS2_NAME);
		if(ps2_device == NULL){			printf("\ncan't find PS/2 device\n");		}
		alt_up_ps2_clear_fifo (ps2_device) ;
		alt_irq_register(PS2_IRQ, ps2_device, ps2_isr); //only enabled in maintain mode
		(CurSysMode == MAINTAIN)? alt_irq_enable(PS2_IRQ) : alt_irq_disable(PS2_IRQ);
		IOWR_8DIRECT(PS2_BASE,4,1);
		printf("Enable ps2_irq\n");
		return;
}

void initSharedResources()
{
	vSemaphoreCreateBinary(SharedSource_KB_update);
	vSemaphoreCreateBinary(SharedSource_LCD_show);
    xSemaphore_Load_manager = xSemaphoreCreateCounting(uxMaxCount_sem,uxInitialCount);
    xSemaphore_Freq_analyzer =xSemaphoreCreateCounting(uxMaxCount_sem,uxInitialCount);
    xSemaphore_LED_updater =xSemaphoreCreateCounting(3,uxInitialCount);
//    xTimer_LoadShed = xTimerCreate("ManagerTimer",1000,pdFALSE,0,tmr_dropLoad);
	return;
}
void initTask()
{
//	xTaskCreate(
//			system_checker,
//			"systemChecker_TASK",
//			configMINIMAL_STACK_SIZE,
//			NULL,
//			SystemChecker_Task_P,
//			&xHandle_systemCheck );

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
	//UpdateScreen_P
	return;

}

