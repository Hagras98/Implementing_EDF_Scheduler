/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2019 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */

/* 
	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the demo application tasks.
 * 
 * Main.c also creates a task called "Check".  This only executes every three 
 * seconds but has the highest priority so is guaranteed to get processor time.  
 * Its main function is to check that all the other tasks are still operational.
 * Each task (other than the "flash" tasks) maintains a unique count that is 
 * incremented each time the task successfully completes its function.  Should 
 * any error occur within such a task the count is permanently halted.  The 
 * check task inspects the count of each task to ensure it has changed since
 * the last time the check task executed.  If all the count variables have 
 * changed all the tasks are still executing error free, and the check task
 * toggles the onboard LED.  Should any task contain an error at any time 
 * the LED toggle rate will change from 3 seconds to 500ms.
 *
 */

/* Standard includes. */
#include <stdlib.h>
#include <stdio.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "lpc21xx.h"

/* Peripheral includes. */
#include "serial.h"
#include "GPIO.h"

/*-----------------------------------------------------------*/

/* Constants to setup I/O and processor. */
#define mainBUS_CLK_FULL	( ( unsigned char ) 0x01 )

/* Constants for the ComTest demo application tasks. */
#define mainCOM_TEST_BAUD_RATE	( ( unsigned long ) 115200 )

/*
 * GPIO pins used to trace tasks on logic analyzer
 */
/*------------------------------------------------------------*/
#define BUTTON_1_MONITOR_DEBUG_PIN		PIN2
#define BUTTON_2_MONITOR_DEBUG_PIN      PIN3
#define PERIODIC_TRANSMITTER_DEBUG_PIN  PIN4
#define UART_RECEIVER_DEBUG_PIN         PIN5
#define LOAD_1_SIMULATION_DEBUG_PIN     PIN6
#define LOAD_2_SIMULATION_DEBUG_PIN     PIN7
#define TICK_HOOK_PIN					PIN8
#define IDLE_HOOK_PIN					PIN9
/*------------------------------------------------------------*/


#define QUEUE_LENGTH 3
#define QUEUE_ITEM_SIZE sizeof( message_t )


/*
 * Configure the processor for use with the Keil demo board.  This is very
 * minimal as most of the setup is managed by the settings in the project
 * file.
 */
static void prvSetupHardware( void );

/*
 * Task Hooks prototypes
 */
static BaseType_t prvButton1MonitorTaskHook( void * pvParameter );
static BaseType_t prvButton2MonitorTaskHook( void * pvParameter );
static BaseType_t prvPeriodicTransmitterTaskHook( void * pvParameter );
static BaseType_t prvUartRecieverTaskHook( void * pvParameter );
static BaseType_t prvLoad1SimulationTaskHook( void * pvParameter );
static BaseType_t prvLoad2SimulationTaskHook( void * pvParameter );

/*
 * Task Prototypes
 */
void Button_1_Monitor_task (void *pvParameters);
void Button_2_Monitor_task (void *pvParameters);
void Periodic_Transmitter_task (void *pvParameters);
void Uart_Receiver_task (void *pvParameters);
void Load_1_Simulation_task (void *pvParameters);
void Load_2_Simulation_task (void *pvParameters);
/*--------------------------------------------------------------------*/

typedef struct 
{
    char ucMessageID;
    char *ucData;
	unsigned short u8messageLength;
}message_t;
	
BaseType_t xTaskCreatePeriodic(	TaskFunction_t pvTaskCode,
								const char * const pcName,
								uint16_t usStackDepth,
								void *pvParameters,
								UBaseType_t uxPriority,
								TaskHandle_t *pvCreatedTask,
								TickType_t period );																		

								/*
 * Global Variables
 */
	uint32_t CPU_TimeIn, CPU_TimeOut, CPU_TotalTime, CPU_Load;

/*
 * Application entry point:
 * Starts all the other tasks, then starts the scheduler. 
 */

int main( void )
{
	/* Setup the hardware for use with the Keil demo board. */
	volatile const QueueHandle_t xQueue = xQueueCreate( QUEUE_LENGTH, QUEUE_ITEM_SIZE );	
	
	prvSetupHardware();
	
    /* Create Tasks here */
	xTaskCreatePeriodic( Button_1_Monitor_task,
						( const char * ) "Button_1_Monitor",
						configMINIMAL_STACK_SIZE, 
						(void*) xQueue,
						1, 
						NULL, 
						pdMS_TO_TICKS(50) );
										
	xTaskCreatePeriodic( Button_2_Monitor_task, 
						( const char * ) "Button_2_Monitor",
						configMINIMAL_STACK_SIZE, 
						(void*) xQueue,
						1,
						NULL,
						pdMS_TO_TICKS(50) );
	
	xTaskCreatePeriodic( Periodic_Transmitter_task,
						( const char * ) "Periodic_Transmitter",
						configMINIMAL_STACK_SIZE, 
						(void*) xQueue,
						1,
						NULL,
						pdMS_TO_TICKS(100) );
	
	xTaskCreatePeriodic( Uart_Receiver_task, 
						( const char * ) "Uart_Reciever",
						configMINIMAL_STACK_SIZE, 
						(void*) xQueue,
						1,
						NULL,
						pdMS_TO_TICKS(20) );
	
	xTaskCreatePeriodic( Load_1_Simulation_task,
						( const char * ) "task 1",
						configMINIMAL_STACK_SIZE, 
						NULL,
						1,
						NULL, 
						pdMS_TO_TICKS(10) );
		
	xTaskCreatePeriodic( Load_2_Simulation_task,
						( const char * ) "task 2",
						configMINIMAL_STACK_SIZE, 
						NULL,
						1,
						NULL, 
						pdMS_TO_TICKS(100) );


	/* Now all the tasks have been started - start the scheduler.

	NOTE : Tasks run in system mode and the scheduler runs in Supervisor mode.
	The processor MUST be in supervisor mode when vTaskStartScheduler is 
	called.  The demo applications included in the FreeRTOS.org download switch
	to supervisor mode prior to main being called.  If you are not using one of
	these demo application projects then ensure Supervisor mode is used here. */
	vTaskStartScheduler();

	/* Should never reach here!  If you do then there was not enough heap
	available for the idle task to be created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/* Function to reset timer 1 */
void timer1Reset(void)
{
	T1TCR |= 0x2;
	T1TCR &= ~0x2;
}

/* Function to initialize and start timer 1 */
static void configTimer1(void)
{
	T1PR = 1000;
	T1TCR |= 0x1;
}

static void prvSetupHardware( void )
{
	/* Perform the hardware setup required.  This is minimal as most of the
	setup is managed by the settings in the project file. */

	/* Configure UART */
	xSerialPortInitMinimal(mainCOM_TEST_BAUD_RATE);

	/* Configure GPIO */
	GPIO_init();
	
	/* Config trace timer 1 and read T1TC to get current tick */
	configTimer1();

	/* Setup the peripheral bus to be the same as the PLL output. */
	VPBDIV = mainBUS_CLK_FULL;	
}

void Button_1_Monitor_task (void *pvParameters)//third
{
	const TickType_t periodInTicks = pdMS_TO_TICKS(50);
	QueueHandle_t xQueue = ((QueueHandle_t)(pvParameters));
	static	message_t button1Message = {0, ""};
	pinState_t oldState, state;
	TickType_t startTick;
	vTaskSetApplicationTaskTag( NULL , prvButton1MonitorTaskHook );
	startTick = xTaskGetTickCount();				
	for(;;)
	{			
		state = GPIO_read(PORT_0, PIN0);
		if (oldState != state)
		{
			if(state == PIN_IS_LOW)
			{
				button1Message.ucData = "Button 1 is Low";
				button1Message.u8messageLength = 15;
			}
			else if (state == PIN_IS_HIGH)
			{
				button1Message.ucData = "Button 1 is High";
				button1Message.u8messageLength = 16;
			}
			oldState = state;
			xQueueSend(xQueue, (const void *) &button1Message, (TickType_t) 0);
		}		
		vTaskDelayUntil(&startTick, periodInTicks)
	}
}

void Button_2_Monitor_task (void *pvParameters)//fourth
{
	const TickType_t periodInTicks = pdMS_TO_TICKS(50);
	QueueHandle_t xQueue = ((QueueHandle_t)(pvParameters));
	static	message_t button2Message = {0, ""};
	pinState_t oldState, state;
	TickType_t startTick;
	vTaskSetApplicationTaskTag( NULL, prvButton2MonitorTaskHook );
	startTick = xTaskGetTickCount();				
	for(;;)
	{			
		state = GPIO_read(PORT_0, PIN1);
		if (oldState != state)
		{
			if(state == PIN_IS_LOW)
			{
				button2Message.ucData = "Button 2 is Low";
				button2Message.u8messageLength = 15;
			}
			else if (state == PIN_IS_HIGH)
			{
				button2Message.ucData = "Button 2 is High";
				button2Message.u8messageLength = 16;
			}
			oldState = state;
			xQueueSend(xQueue, (const void *) &button2Message, (TickType_t) 0);
		}		
		vTaskDelayUntil(&startTick, periodInTicks)
	}
}

void Periodic_Transmitter_task (void *pvParameters)//fifth
{
	TickType_t startTick;
	const TickType_t periodInTicks = pdMS_TO_TICKS(100);
	QueueHandle_t xQueue = ((QueueHandle_t)(pvParameters));
	const message_t periodicMessage = {2, "Periodic", 8};
	vTaskSetApplicationTaskTag( NULL, prvPeriodicTransmitterTaskHook );
	startTick = xTaskGetTickCount();
	for(;;)
	{		
		xQueueSend(xQueue, &periodicMessage, (TickType_t) 0);
		vTaskDelayUntil(&startTick, periodInTicks)
	}		
}

void Uart_Receiver_task (void *pvParameters)//second
{
	TickType_t startTick;
	message_t recievedMessage;
	const TickType_t periodInTicks = pdMS_TO_TICKS(20);
	QueueHandle_t xQueue = ((QueueHandle_t)(pvParameters));
	vTaskSetApplicationTaskTag( NULL, prvUartRecieverTaskHook );
	startTick = xTaskGetTickCount();
	for(;;)
	{			
		if( xQueueReceive( xQueue, &recievedMessage, (TickType_t) 0) == pdTRUE )
		{
			vSerialPutString( (const signed char*)((message_t*) (&recievedMessage))-> ucData, ((message_t*) (&recievedMessage))->u8messageLength);
			xSerialPutChar('\n');
		}
		vTaskDelayUntil(&startTick, periodInTicks)
	}
}

void Load_1_Simulation_task (void *pvParameters)//First
{
	TickType_t startTick;
	volatile uint32_t i;
	const TickType_t periodInTicks = pdMS_TO_TICKS(10);
	vTaskSetApplicationTaskTag( NULL, prvLoad1SimulationTaskHook );
	startTick = xTaskGetTickCount();
	for(;;)
	{
		for(i = 0; i<18800; i++)
		{
			;
		}
		vTaskDelayUntil(&startTick, periodInTicks)
	}
}

void Load_2_Simulation_task (void *pvParameters)//Sixth
{
	TickType_t startTick;
	volatile uint32_t i;
	const TickType_t periodInTicks = pdMS_TO_TICKS(100);
	vTaskSetApplicationTaskTag( NULL, prvLoad2SimulationTaskHook );
	startTick = xTaskGetTickCount();
	for(;;)
	{
		for(i = 0; i<45120;i++)
		{
			;
		}
		vTaskDelayUntil(&startTick, periodInTicks)
	}
}

void vApplicationTickHook(void)
{
	GPIO_write(PORT_0, TICK_HOOK_PIN, PIN_IS_HIGH);
	GPIO_write(PORT_0, TICK_HOOK_PIN, PIN_IS_LOW);
}

void vApplicationIdleHook(void)
{
	GPIO_write(PORT_0, IDLE_HOOK_PIN, PIN_IS_HIGH);
}

static BaseType_t prvButton1MonitorTaskHook( void * pvParameter )
{	
	GPIO_write(PORT_0, IDLE_HOOK_PIN, PIN_IS_LOW);
	if (*((int*)pvParameter) == 1)
	{
		GPIO_write(PORT_0, BUTTON_1_MONITOR_DEBUG_PIN, PIN_IS_HIGH);
		CPU_TimeIn = T1TC;
	}
	else if (*((int*)pvParameter) == 0)
	{
		GPIO_write(PORT_0, BUTTON_1_MONITOR_DEBUG_PIN, PIN_IS_LOW);
		CPU_TimeOut = T1TC;
		CPU_TotalTime += CPU_TimeOut - CPU_TimeIn;
		if (T1TC >= 100)
		{
			CPU_Load = CPU_TotalTime / (T1TC / 100);
		}
		else
		{
			CPU_Load = (CPU_TotalTime * 100) / T1TC;
		}
	}
	return 0;
}

static BaseType_t prvButton2MonitorTaskHook( void * pvParameter )
{	
	GPIO_write(PORT_0, IDLE_HOOK_PIN, PIN_IS_LOW);
	if (*((int*)pvParameter) == 1)
	{
		GPIO_write(PORT_0, BUTTON_2_MONITOR_DEBUG_PIN, PIN_IS_HIGH);
		CPU_TimeIn = T1TC;
	}
	else if (*((int*)pvParameter) == 0)
	{
		GPIO_write(PORT_0, BUTTON_2_MONITOR_DEBUG_PIN, PIN_IS_LOW);
		CPU_TimeOut = T1TC;
		CPU_TotalTime += CPU_TimeOut - CPU_TimeIn;
		if (T1TC >= 100)
		{
			CPU_Load = CPU_TotalTime / (T1TC / 100);
		}
		else
		{
			CPU_Load = (CPU_TotalTime * 100) / T1TC;
		}
	}
	return 0;
}

static BaseType_t prvPeriodicTransmitterTaskHook( void * pvParameter )
{	
	GPIO_write(PORT_0, IDLE_HOOK_PIN, PIN_IS_LOW);
		if (*((int*)pvParameter) == 1)
	{
		GPIO_write(PORT_0, PERIODIC_TRANSMITTER_DEBUG_PIN, PIN_IS_HIGH);
		CPU_TimeIn = T1TC;
	}
	else if (*((int*)pvParameter) == 0)
	{
		GPIO_write(PORT_0, PERIODIC_TRANSMITTER_DEBUG_PIN, PIN_IS_LOW);
		CPU_TimeOut = T1TC;
		CPU_TotalTime += CPU_TimeOut - CPU_TimeIn;
		if (T1TC >= 100)
		{
			CPU_Load = CPU_TotalTime / (T1TC / 100);
		}
		else
		{
			CPU_Load = (CPU_TotalTime * 100) / T1TC;
		}
	}
	return 0;
}

static BaseType_t prvUartRecieverTaskHook( void * pvParameter )
{	
	GPIO_write(PORT_0, IDLE_HOOK_PIN, PIN_IS_LOW);
	if (*((int*)pvParameter) == 1)
	{
		GPIO_write(PORT_0, UART_RECEIVER_DEBUG_PIN, PIN_IS_HIGH);
		CPU_TimeIn = T1TC;
	}
	else if (*((int*)pvParameter) == 0)
	{
		GPIO_write(PORT_0, UART_RECEIVER_DEBUG_PIN, PIN_IS_LOW);
		CPU_TimeOut = T1TC;
		CPU_TotalTime += CPU_TimeOut - CPU_TimeIn;
		if (T1TC >= 100)
		{
			CPU_Load = CPU_TotalTime / (T1TC / 100);
		}
		else
		{
			CPU_Load = (CPU_TotalTime * 100) / T1TC;
		}
	}
	return 0;
}

static BaseType_t prvLoad1SimulationTaskHook( void * pvParameter )
{	
	GPIO_write(PORT_0, IDLE_HOOK_PIN, PIN_IS_LOW);
	if (*((int*)pvParameter) == 1)
	{
		GPIO_write(PORT_0, LOAD_1_SIMULATION_DEBUG_PIN, PIN_IS_HIGH);
		CPU_TimeIn = T1TC;
	}
	else if (*((int*)pvParameter) == 0)
	{
		GPIO_write(PORT_0, LOAD_1_SIMULATION_DEBUG_PIN, PIN_IS_LOW);
		CPU_TimeOut = T1TC;
		CPU_TotalTime += CPU_TimeOut - CPU_TimeIn;
		if (T1TC >= 100)
		{
			CPU_Load = CPU_TotalTime / (T1TC / 100);
		}
		else
		{
			CPU_Load = (CPU_TotalTime * 100) / T1TC;
		}
	}
	return 0;
}

static BaseType_t prvLoad2SimulationTaskHook( void * pvParameter )
{
	GPIO_write(PORT_0, IDLE_HOOK_PIN, PIN_IS_LOW);
		if (*((int*)pvParameter) == 1)
	{
		GPIO_write(PORT_0, LOAD_2_SIMULATION_DEBUG_PIN, PIN_IS_HIGH);
		CPU_TimeIn = T1TC;
	}
	else if (*((int*)pvParameter) == 0)
	{
		GPIO_write(PORT_0, LOAD_2_SIMULATION_DEBUG_PIN, PIN_IS_LOW);
		CPU_TimeOut = T1TC;
		CPU_TotalTime += CPU_TimeOut - CPU_TimeIn;
		if (T1TC >= 100)
		{
			CPU_Load = CPU_TotalTime / (T1TC / 100);
		}
		else
		{
			CPU_Load = (CPU_TotalTime * 100) / T1TC;
		}
	}
	return 0;
}


/*-----------------------------------------------------------*/
