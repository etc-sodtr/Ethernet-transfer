/*
 * ethernet_demo.c
 *
 *  Created on: Nov 28, 2016
 *      Author: Norbert
 */

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/* Standard demo includes. */
#include "partest.h"

#define QUEUE_LENGTH    (10)

#define INITIAL_SEM_TASK_PRIORITY			( tskIDLE_PRIORITY + 4 )
#define READ_QUEUE_TASK_PRIORITY            ( tskIDLE_PRIORITY + 3 )
#define WRITE_QUEUE_TASK_PRIORITY           ( tskIDLE_PRIORITY + 2 )
#define LED_TASK_PRIORITY                   ( tskIDLE_PRIORITY + 1 )


static void initial_sem(void *pvParameters);
static void rd_queue_task(void *pvParameters);
static void wr_queue_task(void *pvParameters);
//static void flash_led_task(void *pvParameters);
static void led_sent_status(void);
static void led_received_status(void);

static int led_base_adr = 0x41200000;

static SemaphoreHandle_t xSemaphore = NULL;
static QueueHandle_t xQueue1 = NULL;

TaskHandle_t xHandle = NULL;

void ethernet_demo(void)
{
    BaseType_t task0;
    BaseType_t task1;
    BaseType_t task2;
//    BaseType_t task3;



    // Queue creation
    xQueue1 = xQueueCreate(QUEUE_LENGTH, sizeof(uint32_t));

    if(xQueue1 == NULL)
    {
        xil_printf("\n\rThe queue was not created! Memory required to create the queue could not be allocated!\n\r");
    }
    else
    {
    	task0 = xTaskCreate( initial_sem,                       /*The function that implements the task. */
                             "CREATE_SEMAPHORE",                /*The text name assigned to the task. */
                             configMINIMAL_STACK_SIZE,          /*The size of the stack to allocate to the task. */
                             NULL,                              /* Parameter passed into the task. */
							 INITIAL_SEM_TASK_PRIORITY,         /* Priority at which the task is created. */
							 &xHandle );                        /* Used to pass out the created task's handle. */

    	task1 = xTaskCreate( rd_queue_task,                     /*The function that implements the task. */
                             "READ_QUEUE",                      /*The text name assigned to the task. */
                             configMINIMAL_STACK_SIZE,          /*The size of the stack to allocate to the task. */
                             NULL,                              /* Parameter passed into the task. */
                             READ_QUEUE_TASK_PRIORITY,          /* Priority at which the task is created. */
                             NULL );                            /* Used to pass out the created task's handle. */

        task2 = xTaskCreate( wr_queue_task,                     /*The function that implements the task. */
                             "WRITE_QUEUE",                     /*The text name assigned to the task. */
                             configMINIMAL_STACK_SIZE,          /*The size of the stack to allocate to the task. */
                             NULL,                              /* Parameter passed into the task. */
                             WRITE_QUEUE_TASK_PRIORITY,         /* Priority at which the task is created. */
                             NULL );                            /* Used to pass out the created task's handle. */

//        task3 = xTaskCreate( flash_led_task,                    /*The function that implements the task. */
//                             "LED",                             /*The text name assigned to the task. */
//                             configMINIMAL_STACK_SIZE,          /*The size of the stack to allocate to the task. */
//                             NULL,                              /* Parameter passed into the task. */
//                             LED_TASK_PRIORITY,                 /* Priority at which the task is created. */
//                             NULL );                            /* Used to pass out the created task's handle. */

        if((task0 == pdPASS) && (task1 == pdPASS) && (task2 == pdPASS))
            xil_printf("\n\r	Tasks created succesfully!	\n\r");
        else
            xil_printf("\n\r###errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY!###\n\r");

        /* Start the tasks and timer running. */
        xil_printf("\n\rStarting the scheduler........\n\r");
        vTaskStartScheduler();
    }

    // This line should not be reached
    xil_printf("\n\rError: program freeze. Insufficient RAM!\n\r");
    for(;;);
}

static void initial_sem(void *pvParameters)
{
    /* Remove compiler warning about unused parameter. */
    ( void ) pvParameters;

    xil_printf("\n\r############	Creating semaphore.....		############\n\r");

//	Creating the semaphore
    xSemaphore = xSemaphoreCreateBinary();
    xSemaphoreGive( xSemaphore );

    if(xSemaphore == NULL)
    	xil_printf("\n\r##############There was insufficient FreeRTOS heap available for the semaphore to be created successfully.##############\n\r");
    else
    	xil_printf("\n\r############## Semaphore created succesfully! ##############\n\r");
    vTaskDelete( xHandle );
}

static void rd_queue_task(void *pvParameters)
{
    /* Remove compiler warning about unused parameter. */
    ( void ) pvParameters;

    const TickType_t xFrequency = 3000;
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    BaseType_t rd_queue = (BaseType_t)NULL;
    uint32_t rd_buffer = 0;

    for(;;)
    {
        rd_buffer = 0;
        rd_queue = (BaseType_t)NULL;

        xil_printf("\n\r###### Waiting for item to receive... ######\n\r");
		rd_queue = xQueueReceive(   xQueue1,
									&rd_buffer,
									portMAX_DELAY
								);

		if( xSemaphoreTake( xSemaphore, portMAX_DELAY) == pdTRUE )
		{
			xil_printf("\n\r");
			xil_printf("\n\r###### Read task obtained the semaphore! ######\n\r");

			if(rd_queue == pdTRUE )
				xil_printf("\n\r###### Item succesfully received! ######\n\r");
			else
				xil_printf("\n\r###### Couldn't read the queue! ######\n\r");

			if(rd_buffer != 0)
				vTaskDelayUntil( &xLastWakeTime, xFrequency );

			led_received_status();

			xil_printf("\n\r###### Read task: releasing the semaphore! ######\n\r");
			xil_printf("\n\r");

			if( xSemaphoreGive( xSemaphore ) != pdTRUE)
				xil_printf("\n\r###### Read task couldn't release the semaphore! ######\n\r");

		}
		else
			xil_printf("\n\r###### Read task couldn't obtain the semaphore! ######\n\r");
    }
}

static void wr_queue_task(void *pvParameters)
{
    /* Remove compiler warning about unused parameter. */
    ( void ) pvParameters;

    BaseType_t wr_queue = (BaseType_t)NULL;
    uint32_t wr_buffer = 10;

    for(;;)
    {
		wr_queue = (BaseType_t)NULL;


    	if( xSemaphoreTake( xSemaphore, portMAX_DELAY) == pdTRUE )
    	{
    		xil_printf("\n\r");
			xil_printf("\n\r###### Write task obtained the semaphore! ######\n\r");


			xil_printf("\n\r###### Sendind item... ######\n\r");
			wr_queue = xQueueSend(      xQueue1,
										&wr_buffer,
										portMAX_DELAY
								  );

			if(wr_queue == pdTRUE )
				xil_printf("\n\r###### Item succesfully sent! ######\n\r");
			else
				xil_printf("\n\r###### Couldn't write to the queue! ######\n\r");

			led_sent_status();

			xil_printf("\n\r###### Write task: releasing the semaphore! ######\n\r");
			xil_printf("\n\r");

			if( xSemaphoreGive( xSemaphore ) != pdTRUE)
				xil_printf("\n\r###### Write task couldn't release the semaphore! ######\n\r");
    	}
    	else
    	    xil_printf("\n\r###### Write task couldn't obtain the semaphore! ######\n\r");
    }
}
//static void flash_led_task(void *pvParameters)
//{
//    /* Remove compiler warning about unused parameter. */
//    ( void ) pvParameters;
//
//    const TickType_t xFrequency = 80;
//    TickType_t xLastWakeTime;
//    xLastWakeTime = xTaskGetTickCount();
//
//    int led_base_adr = 0x41200000;
//    uint8_t led_value = 0xAA;
//
//    for(;;)
//    {
//        Xil_Out8(led_base_adr, led_value);
//        xil_printf("\n\r###### Queue full! ######\n\r", led_value);
//        led_value = (led_value << 1) | (led_value >> 3);
//        vTaskDelayUntil( &xLastWakeTime, xFrequency );
//    }
//}

static void led_sent_status(void)
{
	const TickType_t xDelay = 500 / portTICK_PERIOD_MS;
	uint8_t led_value = 0xAA;

	for(int i = 0; i < 3; i++ )
	{
		Xil_Out8(led_base_adr, led_value);
		vTaskDelay( xDelay );
		led_value = (led_value << 1) | (led_value >> 3);
	}
}

static void led_received_status(void)
{
	const TickType_t xDelay = 500 / portTICK_PERIOD_MS;
	uint8_t led_value = 0xFF;

	for(int i = 0; i < 3; i++ )
	{
		Xil_Out8(led_base_adr, led_value);
		vTaskDelay( xDelay );
		led_value = ~led_value ;
	}
}
