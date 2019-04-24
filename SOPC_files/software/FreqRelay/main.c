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

//For frequency plot
#define FREQPLT_ORI_X 101		//x axis pixel position at the plot origin
#define FREQPLT_GRID_SIZE_X 5	//pixel separation in the x axis between two data points
#define FREQPLT_ORI_Y 199.0		//y axis pixel position at the plot origin
#define FREQPLT_FREQ_RES 20.0	//number of pixels per Hz (y axis scale)

#define ROCPLT_ORI_X 101
#define ROCPLT_GRID_SIZE_X 5
#define ROCPLT_ORI_Y 259.0
#define ROCPLT_ROC_RES 0.5		//number of pixels per Hz/s (y axis scale)

#define MIN_FREQ 45.0 //minimum frequency to draw



/*Task Priorities:*/
// #define Counter_Task_P      	(tskIDLE_PRIORITY)
#define FreqAnalyser_Task_P		(tskIDLE_PRIORITY+5)
#define LoadManager_Task_P 		(tskIDLE_PRIORITY+5)
#define UpdateLED_Task_P 		(tskIDLE_PRIORITY+5)
#define UpdateThresholds_P		(tskIDLE_PRIORITY+5)
#define PRVGADraw_Task_P      	(tskIDLE_PRIORITY+4)
#define UpdateScreen_P			(tskIDLE_PRIORITY+4)
//#define SystemChecker_Task_P 	(tskIDLE_PRIORITY+1)


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
TaskHandle_t PRVGADraw;

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

typedef struct{
	unsigned int x1;
	unsigned int y1;
	unsigned int x2;
	unsigned int y2;
}Line;

/*Semaphore*/
SemaphoreHandle_t SharedSource_KB_update; 	 //use semaphore to control update procedure.
SemaphoreHandle_t SharedSource_LCD_show; 	 //use to guard LCD usage.
SemaphoreHandle_t SharedSource_Load_manager; // use with load manager
SemaphoreHandle_t ShareSource_LED_sem;
SemaphoreHandle_t xSem_FreqAnalyzer;
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
bool KeyboardRaiseEdge = false;

/*For Threshold setting,
 * setFreqOrRoc = r/R for setting RoC threshold
 * setFreqOrRoc = f/F for setting Frequency threshold
 */
char setFreqOrRoc = 'N';

/*Record of current ON loads, in terms of LEDs*/
unsigned int uiLoadBank = 0;
unsigned int uiManageTime = 0;
//stores the time cost of load dropping. in sec.
unsigned int uiTimer_LoadDropped = 0, uiTimer_isUnstable = 0, uiTimer_DropTime_reg = 0,
		uiTimer_firstUnstable = 0,  uiTimer_firstLoadDropped = 0,uiTimer_DropTime_first=0,
		uiTimer_DropTimer_History[5] = {0}, uiTimer_DropTimer_index=0;
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
static QueueHandle_t Q_freq_data;

/*-------------Interrupts--------------------*/
/* PushBtn ISR,
 * use to go into maintenance mode
 */
void pushbutton_ISR(void* context, alt_u32 id)
{
	//printf("\n button interrupt triggers\n alt_u32 id = %d\n", id);
	//need to cast the context first before using it
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
//		  xSemaphoreGiveFromISR(SharedSource_KB_update,UpdateThresholds_P);
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

	bool isEnterHit = false;
	temp = byte;
	alt_up_ps2_dev * ps2_device = alt_up_ps2_open_dev(PS2_NAME);
	alt_up_ps2_read_data_byte_timeout(ps2_device, &byte);
	printf("Scan code: %x\n", byte);
	//filter out invalid values
	//1to0	16,1e,26,25,...etc
	//R = 0x2d
	//F = 0x2b
	//Enter = 0x5a
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
				xQueueSendToBackFromISR(Q_KeyboardInput, &byte,pdFALSE);
				isEnterHit = false;
				break;
		}
		case 0x5a:
		{
			if (KeyboardRaiseEdge){
				xSemaphoreGiveFromISR(SharedSource_LCD_show,portMax_DELAY);
				xSemaphoreGiveFromISR(SharedSource_KB_update,portMax_DELAY);
				isEnterHit = true;
				KeyboardRaiseEdge = false;
				printf("KEYBOARD:ENTER_RAISE\n");
			}else{
				KeyboardRaiseEdge = true;
				printf("KEYBOARD:ENTER_DOWN\n");
			}
			break;
		}
	}

	FILE* lcd = fopen(CHARACTER_LCD_NAME, "w");
	if (byte == 0x5a){
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
	}else if (isEnterHit){
		fprintf(lcd,"%c%s %s",esc,"[2J","Confirmed \n");
		fclose(lcd);

	}
	return;
}

/****** VGA display ******/

void PRVGADraw_Task(void *pvParameters ){

	char s[20];
	char t[20];
	char u[30];
	char loadHis[40];
	//initialize VGA controllers
	alt_up_pixel_buffer_dma_dev *pixel_buf;
	pixel_buf = alt_up_pixel_buffer_dma_open_dev(VIDEO_PIXEL_BUFFER_DMA_NAME);
	if(pixel_buf == NULL){
		printf("can't find pixel buffer device\n");
	}
	alt_up_pixel_buffer_dma_clear_screen(pixel_buf, 0);

	alt_up_char_buffer_dev *char_buf;
	char_buf = alt_up_char_buffer_open_dev("/dev/video_character_buffer_with_dma");
	if(char_buf == NULL){
		printf("can't find char buffer device\n");
	}
	alt_up_char_buffer_clear(char_buf);



	//Set up plot axes
	alt_up_pixel_buffer_dma_draw_hline(pixel_buf, 100, 590, 200, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_hline(pixel_buf, 100, 590, 300, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_vline(pixel_buf, 100, 50, 200, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);
	alt_up_pixel_buffer_dma_draw_vline(pixel_buf, 100, 220, 300, ((0x3ff << 20) + (0x3ff << 10) + (0x3ff)), 0);

	alt_up_char_buffer_string(char_buf, "Frequency(Hz)", 4, 4);
	alt_up_char_buffer_string(char_buf, "52", 10, 7);
	alt_up_char_buffer_string(char_buf, "50", 10, 12);
	alt_up_char_buffer_string(char_buf, "48", 10, 17);
	alt_up_char_buffer_string(char_buf, "46", 10, 22);

	alt_up_char_buffer_string(char_buf, "df/dt(Hz/s)", 4, 26);
	alt_up_char_buffer_string(char_buf, "60", 10, 28);
	alt_up_char_buffer_string(char_buf, "30", 10, 30);
	alt_up_char_buffer_string(char_buf, "0", 10, 32);
	alt_up_char_buffer_string(char_buf, "-30", 9, 34);
	alt_up_char_buffer_string(char_buf, "-60", 9, 36);






	double freq[100], dfreq[100];
	int i = 99, j = 0;
	Line line_freq, line_roc;


	while(1){
		//receive frequency data from queue
		while(uxQueueMessagesWaiting( Q_freq_data ) != 0){
			xQueueReceive( Q_freq_data, freq+i, 0 );

			//calculate frequency RoC

			if(i==0){
				dfreq[0] = (freq[0]-freq[99]) * 2.0 * freq[0] * freq[99] / (freq[0]+freq[99]);
			}
			else{
				dfreq[i] = (freq[i]-freq[i-1]) * 2.0 * freq[i]* freq[i-1] / (freq[i]+freq[i-1]);
			}

			if (dfreq[i] > 100.0){
				dfreq[i] = 100.0;
			}


			i =	++i%100; //point to the next data (oldest) to be overwritten

		}

		//clear old graph to draw new graph
		alt_up_pixel_buffer_dma_draw_box(pixel_buf, 101, 0, 639, 199, 0, 0);
		alt_up_pixel_buffer_dma_draw_box(pixel_buf, 101, 201, 639, 299, 0, 0);

		for(j=0;j<99;++j){ //i here points to the oldest data, j loops through all the data to be drawn on VGA
			if (((int)(freq[(i+j)%100]) > MIN_FREQ) && ((int)(freq[(i+j+1)%100]) > MIN_FREQ)){
				//Calculate coordinates of the two data points to draw a line in between
				//Frequency plot
				line_freq.x1 = FREQPLT_ORI_X + FREQPLT_GRID_SIZE_X * j;
				line_freq.y1 = (int)(FREQPLT_ORI_Y - FREQPLT_FREQ_RES * (freq[(i+j)%100] - MIN_FREQ));

				line_freq.x2 = FREQPLT_ORI_X + FREQPLT_GRID_SIZE_X * (j + 1);
				line_freq.y2 = (int)(FREQPLT_ORI_Y - FREQPLT_FREQ_RES * (freq[(i+j+1)%100] - MIN_FREQ));

				//Frequency RoC plot
				line_roc.x1 = ROCPLT_ORI_X + ROCPLT_GRID_SIZE_X * j;
				line_roc.y1 = (int)(ROCPLT_ORI_Y - ROCPLT_ROC_RES * dfreq[(i+j)%100]);

				line_roc.x2 = ROCPLT_ORI_X + ROCPLT_GRID_SIZE_X * (j + 1);
				line_roc.y2 = (int)(ROCPLT_ORI_Y - ROCPLT_ROC_RES * dfreq[(i+j+1)%100]);

				//Draw
				alt_up_pixel_buffer_dma_draw_line(pixel_buf, line_freq.x1, line_freq.y1, line_freq.x2, line_freq.y2, 0x3ff << 0, 0);
				alt_up_pixel_buffer_dma_draw_line(pixel_buf, line_roc.x1, line_roc.y1, line_roc.x2, line_roc.y2, 0x3ff << 0, 0);

			}
		}

		sprintf(s, "Freq Thresh: %.2f", Threshold_Freq);
		sprintf(t, "ROC Thresh: %.2f", Threshold_RoC);
		sprintf(u, "First Drop in(ms):%.2i", uiTimer_DropTime_first);
		sprintf(loadHis, "Drop History(ms):%.2i %.2i %.2i %.2i %.2i",
				uiTimer_DropTimer_History[0],uiTimer_DropTimer_History[1],
				uiTimer_DropTimer_History[2],uiTimer_DropTimer_History[3],
				uiTimer_DropTimer_History[4]);
		alt_up_char_buffer_string(char_buf, s, 2, 46);
		alt_up_char_buffer_string(char_buf, t, 2, 48);
		alt_up_char_buffer_string(char_buf, u, 2, 50);
		alt_up_char_buffer_string(char_buf, loadHis, 2, 55);

		vTaskDelay(10);

	}
}

/* Frequency relay ISR
 * Record frequency value and time on interrupt
 * Stores data into Q_FreqInfo
 */
void freq_relay(void *pvParameters)
{
	printf("\nfreq_relay\n");
	#define SAMPLING_FREQ 16000.0
	double temp = SAMPLING_FREQ/(double)IORD(FREQUENCY_ANALYSER_BASE, 0);

	xQueueSendToBackFromISR( Q_freq_data, &temp, pdFALSE );


	FreqInfo freqData = {
			(double)16000/(double)IORD(FREQUENCY_ANALYSER_BASE, 0),
			xTaskGetTickCountFromISR()
	};
//	printf("freeq_value = %f\t time=%d\n",freqData.freq_value, freqData.record_time);

	if (CurSysMode == RUN)
	{
		xQueueSendToBackFromISR( Q_FreqInfo, &freqData, pdFALSE );
		xQueueSendToBackFromISR( Q_VGAUpdateValues, &freqData.record_time, pdFALSE );

//        xSemaphoreGiveFromISR(SharedSource_Load_manager,LoadManager_Task_P);
        xSemaphoreGiveFromISR(xSem_FreqAnalyzer,FreqAnalyser_Task_P);
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
		xSemaphoreTake(SharedSource_KB_update, portMAX_DELAY);
		printf("\nUpdateThreshold\t");
		char key_q = 0;
		unsigned int temp[10]={0},result[5]={0};
		int inputLength = 0, resultLength = 0;
		bool isEnterHit = false;
		double threshold = 0;
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
//				case 0x5a:	{isEnterHit = true;break;}
			}
			temp[inputLength]= numb;
			printf("temp[%d]=%d ",inputLength,temp[inputLength]);
			++inputLength;

			for( int i=0,j=0; i<=inputLength;++i)//take every two element from array
			{
				if(i%2 == 0)
				{
					result[j] = temp[i];
					resultLength = ++j;
				}
			}
			printf("\nRESULT:");
			for (int index = 0; index<=resultLength; ++index)
			{
				printf("result[%d]=%d ",index,result[index]);
			}
			printf("\n");

		}
		/*calculate thread*/
		for( int  i=0 , j=resultLength ; i<resultLength, j>0; ++i , --j )
		{
			threshold += result[i]*pow(10,j);
		}

		//prepare LCD for display threshold
//		if(KeyboardRaiseEdge)
//		{
			xSemaphoreTake(SharedSource_LCD_show,pdFALSE);
			FILE* fp;
			fp = fopen(CHARACTER_LCD_NAME, "w"); //open the character LCD as a file stream for write

			if (fp == NULL) {
				printf("open failed\n");
			}
			if (setFreqOrRoc == 'r')
			{
				Threshold_RoC = (Threshold_dec)? (double)(threshold/1000.0) : (double)(threshold/100.0) ;
				printf("\nCalculated Threshold=%f",threshold);
				fprintf(fp, "%c%s RoC:%.1f\n Freq:%.1f\n", esc,"[2J",Threshold_RoC,Threshold_Freq);
				xSemaphoreGive(SharedSource_LCD_show);
				printf("ROC=%f",Threshold_RoC);
			}
			else if(setFreqOrRoc == 'f')
			{
				Threshold_Freq = (Threshold_dec)? (double)(threshold/1000.0) : (double)(threshold/100.0) ;
				printf("\nCalculated Threshold=%f",threshold);
				fprintf(fp, "%c%s RoC:%.1f\n Freq:%.1f\n", esc,"[2J",Threshold_RoC,Threshold_Freq);
				xSemaphoreGive(SharedSource_LCD_show);
				printf("Freq=%f",Threshold_Freq);
			}
			else
			{
				printf("\n##ENTER r for ROC, f for Frequency\n");
			}
			setFreqOrRoc = 'N';
			fclose(fp);
			++updateCounter;
			Threshold_dec = false;
			//		printf("updateCounter = %d\n",updateCounter);
//			isEnterHit = false;
//		}
	vTaskDelay(10);
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
	double RoC = 0;
	int FreqHyIndex = 0;
	bool isFirstUnstable = true;

	/* read from queue
	 * First check the current reading, if it's lower than threshold, call load ctr
	 * If the available data is more than 2, calculate RoC
	 */
	while (1)
	{
		xSemaphoreTake(xSem_FreqAnalyzer,portMAX_DELAY);
		printf("freq_analyser\n");
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
			if(isFirstUnstable)
			{
				uiTimer_firstUnstable = xTaskGetTickCount();
				isFirstUnstable = false;
			}else{
				uiTimer_isUnstable = xTaskGetTickCount();
			}
			xQueueSendToBackFromISR(Q_LoadOperation,&currentSysStability, pdFALSE);
//			printf("\tRoc:%f freq:%f \tSys UNSTABLE\n",RoC,curFreq.freq_value);
		}else{
			currentSysStability = STABLE;//STABLE;
			xQueueSendToBackFromISR(Q_LoadOperation,&currentSysStability, pdFALSE);
//			printf("\tRoc:%f freq:%f \tSys STABLE\n",RoC,curFreq.freq_value);
		}
//		xSemaphoreGive(xSem_FreqAnalyzer);
		xSemaphoreGiveFromISR(SharedSource_Load_manager,LoadManager_Task_P);
		vTaskDelay(10);
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
        xSemaphoreTake(SharedSource_Load_manager,portMAX_DELAY);
        printf("\nLoadManager\n");
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
								xSemaphoreTake(ShareSource_LED_sem,portMax_DELAY);
								uiLoadBank -= (unsigned int)pow(2,dropCounter);
								xSemaphoreGive(ShareSource_LED_sem);
								++dropCounter;
	//							printf("\n##UNSTABLE-->dropLoad_FIRST, uiLoadBank=%d, dropCounter=%d ##\n",uiLoadBank,dropCounter);
							}
							uiManageTime = xTaskGetTickCount();
						}else{
							if ( xTaskGetTickCount() - 500 > uiManageTime )
							{
								/*Drop loads after first load is dropped, 500ms waiting applies*/
								xSemaphoreTake(ShareSource_LED_sem,portMax_DELAY);
								if (uiLoadBank != 0){
									uiLoadBank -= pow(2,dropCounter);
									++dropCounter;
	//								printf("\n##UNSTABLE-->dropLoad, uiLoadBank=%d, dropCounter=%d ##\n",uiLoadBank,dropCounter);
								}
								xSemaphoreGive(ShareSource_LED_sem);
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
								xSemaphoreTake(ShareSource_LED_sem,portMax_DELAY);
								if (uiLoadBank != redMask)
								{
									--dropCounter;
									uiLoadBank += pow(2,dropCounter);//Add loads
//									printf("\n##STABLE-->Add loads, uiLoadBank=%d,dropCounter=%d ##\n",uiLoadBank,dropCounter);
								}
								xSemaphoreGive(ShareSource_LED_sem);
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
		vTaskDelay(10);
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
	unsigned int uiSwitchValue;
	unsigned int uiRedLED;
	unsigned int uiGreenLED;
	/*indicate load condition with LEDs*/
	while(1)
	{
		/*If the system is stable, switch can be used to control load on/off correspondingly*/
			xSemaphoreTake(ShareSource_LED_sem,portMAX_DELAY);
			printf("update LED\n");
			uiSwitchValue = IORD_ALTERA_AVALON_PIO_DATA(SLIDE_SWITCH_BASE);
			uiRedLED = uiSwitchValue & uiLoadBank;
			uiGreenLED = (~uiRedLED & uiSwitchValue)&redMask ;
		if(currentSysStability == 1)
		{
			//For Most 5 right
			IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, uiRedLED);
			IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, uiGreenLED);
		}else{
			IOWR_ALTERA_AVALON_PIO_DATA(RED_LEDS_BASE, uiRedLED);
			if (isFirstUpdate){
				uiTimer_firstLoadDropped = xTaskGetTickCount()- uiTimer_firstUnstable;
				isFirstUpdate = false;
			}else{
				uiTimer_DropTime_reg = xTaskGetTickCount() - uiTimer_isUnstable;
				uiTimer_DropTimer_History[uiTimer_DropTimer_index%5]=uiTimer_DropTime_reg;
				++uiTimer_DropTimer_index;
			}
			IOWR_ALTERA_AVALON_PIO_DATA(GREEN_LEDS_BASE, uiGreenLED);
		}
		vTaskDelay(10);
	}
			printf("update_LED,system unstable");
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
		Q_VGAUpdateTime = xQueueCreate(1000,sizeof(unsigned char));
		Q_freq_data = xQueueCreate( 100, sizeof(double) );
		return;
}
void initInterrupts(void)
{
	/*Frequency Analyzer ISR*/
		alt_irq_register(FREQUENCY_ANALYSER_IRQ, 0, freq_relay);

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
	vSemaphoreCreateBinary(xSem_FreqAnalyzer);
	vSemaphoreCreateBinary(SharedSource_KB_update);
	vSemaphoreCreateBinary(SharedSource_LCD_show);
    vSemaphoreCreateBinary(SharedSource_Load_manager);
    vSemaphoreCreateBinary(ShareSource_LED_sem);
	return;
}
void initTask()
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
	//UpdateScreen_P

	xTaskCreate(
			PRVGADraw_Task,
			"DrawTsk",
			configMINIMAL_STACK_SIZE,
			NULL,
			PRVGADraw_Task_P,
			&PRVGADraw );
	return;

}

