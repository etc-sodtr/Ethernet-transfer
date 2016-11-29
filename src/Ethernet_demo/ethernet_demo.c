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


static QueueHandle_t xQueue1 = NULL;

void ethernet_demo(void)
{
    // Queue creation
    xQueue1 = xQueueCreate(10, sizeof(uint32_t));

    if(xQueue1 == NULL)
        xil_printf("\n\The queue was not created!\n\r");

    else
    {
        xTaskCreate
        xTaskCreate

        vtaskStartScheduler();
    }

    // This line should not be reached
    xil_printf("\n\Error: program freeze. Insufficient RAM!\n\r");
    for(;;);
}
