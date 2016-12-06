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

#define READ_QUEUE_TASK_PRIORITY            ( tskIDLE_PRIORITY + 3 )
#define WRITE_QUEUE_TASK_PRIORITY           ( tskIDLE_PRIORITY + 2 )
#define LED_TASK_PRIORITY                   ( tskIDLE_PRIORITY + 1 )


static void rd_queue_task(void *pvParameters);
static void wr_queue_task(void *pvParameters);
static void flash_led_task(void *pvParameters);

static QueueHandle_t xQueue1 = NULL;

void ethernet_demo(void)
{
    BaseType_t task1;
    BaseType_t task2;
    BaseType_t task3;

    // Queue creation
    xQueue1 = xQueueCreate(QUEUE_LENGTH, sizeof(uint32_t));

    if(xQueue1 == NULL)
    {
        xil_printf("\n\rThe queue was not created! Memory required to create the queue could not be allocated!\n\r");
    }
    else
    {
    	task1 = xTaskCreate( rd_queue_task,                     /*The function that implements the task. */
                             "READ_QUEUE",                      /*The text name assigned to the task. */
                             configMINIMAL_STACK_SIZE,          /*The size of the stack to allocate to the task. */
                             NULL,                              /* Parameter passed into the task. */
                             READ_QUEUE_TASK_PRIORITY,          /* Priority at which the task is created. */
                             NULL );                            /* Used to pass out the created task's handle. */

        task2 = xTaskCreate( wr_queue_task,                          /*The function that implements the task. */
                             "WRITE_QUEUE",                     /*The text name assigned to the task. */
                             configMINIMAL_STACK_SIZE,          /*The size of the stack to allocate to the task. */
                             NULL,                              /* Parameter passed into the task. */
                             WRITE_QUEUE_TASK_PRIORITY,         /* Priority at which the task is created. */
                             NULL );                            /* Used to pass out the created task's handle. */

        task3 = xTaskCreate( flash_led_task,                         /*The function that implements the task. */
                             "LED",                             /*The text name assigned to the task. */
                             configMINIMAL_STACK_SIZE,          /*The size of the stack to allocate to the task. */
                             NULL,                              /* Parameter passed into the task. */
                             LED_TASK_PRIORITY,                 /* Priority at which the task is created. */
                             NULL );                            /* Used to pass out the created task's handle. */

        if((task1 == pdPASS) && (task2 == pdPASS) && (task3 == pdPASS))
            xil_printf("\n\rTasks created succesfully!\n\r");
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

        if(rd_queue == pdTRUE )
            xil_printf("\n\r###### Item succesfully received! ######\n\r");
        else
            xil_printf("\n\r###### Couldn't read the queue! ######\n\r");

        if(rd_buffer != 0)
            vTaskDelayUntil( &xLastWakeTime, xFrequency );
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

        xil_printf("\n\r###### Sendind item... ######\n\r");
        wr_queue = xQueueSend(      xQueue1,
                                    &wr_buffer,
                                    portMAX_DELAY
                             );

        if(wr_queue == pdTRUE )
                xil_printf("\n\r###### Item succesfully sent! ######\n\r");
            else
                xil_printf("\n\r###### Couldn't write to the queue! ######\n\r");
    }
}
static void flash_led_task(void *pvParameters)
{
    /* Remove compiler warning about unused parameter. */
    ( void ) pvParameters;

    const TickType_t xFrequency = 80;
    TickType_t xLastWakeTime;
    xLastWakeTime = xTaskGetTickCount();

    int led_base_adr = 0x41200000;
    uint8_t led_value = 0xAA;

    for(;;)
    {
        Xil_Out8(led_base_adr, led_value);
        xil_printf("\n\r###### Queue full! ######\n\r", led_value);
        led_value = (led_value << 1) | (led_value >> 3);
        vTaskDelayUntil( &xLastWakeTime, xFrequency );
    }
}
