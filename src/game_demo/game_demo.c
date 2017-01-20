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

// DEBUG //
#define DEBUG1
//#define DEBUG2

// Project Parameters //
#define QUEUE_LENGTH    (10)

#define FLASH_LED_TASK_PRIORITY				( tskIDLE_PRIORITY + 2)
#define READ_QUEUE_TASK_PRIORITY            ( tskIDLE_PRIORITY + 1)
#define WRITE_QUEUE_TASK_PRIORITY           ( tskIDLE_PRIORITY)


// Color Print Defines //
#define CSI             "\x1b["
// clear terminal
#define CLS             DEF CSI "2J" CSI "0;0H"
// set colors (fixed 16-color palette)
// 30..37 sets foreground color, add ";1" for bright colors
// 40..47 sets background color, add ";1" for bright colors
#define SET_FG(c)       CSI "3" #c "m"
#define SET_BG(c)       CSI "4" #c "m"
// set extended colors
// color is (16 + 36 * r + 6 * g + b), where r,g,b = 0..5
#define SET_FGE(c)      CSI "38;5;" #c "m"
#define SET_BGE(c)      CSI "48;5;" #c "m"
#define BLACK           SET_FG(0)
#define RED             SET_FG(1;1)
#define RED_BG          SET_BGE(9)
#define GREEN           SET_FGE(46)
#define YELLOW          SET_FGE(226)
#define BLUE            SET_FGE(21)
#define MAGENTA         SET_FGE(201)
#define CYAN            SET_FGE(51)
#define WHITE           SET_FGE(231)
#define ORANGE          SET_FGE(214)
#define OLIVE           SET_FGE(142)
#define GRAY1           SET_FGE(59)
#define GRAY2           SET_FGE(102)
#define GRAY3           SET_FGE(145)
#define GRAY4           SET_FGE(188)
// reset to default colors
#define DEF             CSI "m"


// Base Addresses //
u32 led_base_addr = 0x41200000;
u32 sw_base_addr = 0x41210000;

// Number to reach by players //
u8 random_nr = 0;

// Player Values and ID//
u8 player1_val = 0;
u8 player2_val = 0;
u8 players_ID = 0; //1 - player 1 , 2 - player 2

static void rd_queue_task(void *pvParameters);
static void wr_queue_task(void *pvParameters);
static void flash_led(void *pvParameters);

static QueueHandle_t xQueue1 = NULL;
static SemaphoreHandle_t xSemaphore;

void game_demo(void)
{

#if defined DEBUG1
	int switch_sts = 0;
	Xil_Out8(led_base_addr, 0xFF);
	switch_sts = Xil_In8(sw_base_addr);
	BaseType_t swit0 = 0;
	BaseType_t swit1 = 0;
	swit0 = vParTestGetSW0();
	swit1 = vParTestGetSW1();
#endif

	xil_printf(CLS);
	xil_printf(GREEN"\n\r........................Starting the PushButton game");

	for(int i = 0; i<20; i++)
	{	xil_printf(".");
		usleep(100000);	//	wait 100 ms //
	}
	xil_printf(WHITE"\n\r");


    BaseType_t task1;
    BaseType_t task2;
    BaseType_t task3;

	 // Queue creation //
	xQueue1 = xQueueCreate(QUEUE_LENGTH, sizeof(u8));

	// Create Semaphore //
	xSemaphore = xSemaphoreCreateBinary();
	xSemaphoreGive( xSemaphore );

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

        task2 = xTaskCreate( wr_queue_task,                     /*The function that implements the task. */
                             "WRITE_QUEUE",                     /*The text name assigned to the task. */
                             configMINIMAL_STACK_SIZE,          /*The size of the stack to allocate to the task. */
                             NULL,                              /* Parameter passed into the task. */
                             WRITE_QUEUE_TASK_PRIORITY,         /* Priority at which the task is created. */
                             NULL );                            /* Used to pass out the created task's handle. */

        task3 = xTaskCreate( flash_led,                     /*The function that implements the task. */
							 "FLASH LED",                     /*The text name assigned to the task. */
							 configMINIMAL_STACK_SIZE,          /*The size of the stack to allocate to the task. */
							 NULL,                              /* Parameter passed into the task. */
							 FLASH_LED_TASK_PRIORITY,         /* Priority at which the task is created. */
							 NULL );                            /* Used to pass out the created task's handle. */

#if defined DEBUG1
        if((task1 == pdPASS) && (task2 == pdPASS)&& (task3 == pdPASS))
            xil_printf("\n\r	Tasks created succesfully!	\n\r");
        else
            xil_printf("\n\r###errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY!###\n\r");
#endif

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

    BaseType_t rd_queue = (BaseType_t)NULL;
    u8 rd_buffer = 0;

    for(;;)
    {
        rd_buffer = 0;
        rd_queue = (BaseType_t)NULL;

#if defined DEBUG2
        xil_printf("\n\r###### Waiting for item to receive... ######\n\r");
#endif
		rd_queue = xQueueReceive(   xQueue1,
									&rd_buffer,
									portMAX_DELAY
								);

			if(rd_queue == pdTRUE )
				xil_printf(WHITE"\n\r###### Item succesfully received! ######\n\r");
			else
				xil_printf("\n\r###### Couldn't read the queue! ######\n\r");

			if( xSemaphoreTake( xSemaphore, portMAX_DELAY ) == pdTRUE )
			{
				if(rd_buffer == 1)
				{
					for(int i = 0; i < 15; i++)
					{
						xil_printf(CLS);
						Xil_Out8(led_base_addr, 0x00);
						usleep(200000);
						if(player1_val == random_nr)
						{
							xil_printf(RED"\n\r###### Player 1 won the game! ######\n\r");
							xil_printf(CYAN"\n\r###### Player 2 lost the game! ######\n\r");
							Xil_Out8(led_base_addr, 0x70);
						}
						else
						{
							xil_printf(RED"\n\r###### Player 1 lost the game! ######\n\r");
							xil_printf(CYAN"\n\r###### Player 2 won the game! ######\n\r");
							Xil_Out8(led_base_addr, 0x07);
						}
						usleep(350000);
					}
				}
				if(rd_buffer == 2)
				{
					for(int i = 0; i < 10; i++)
					{
						xil_printf(CLS);
						Xil_Out8(led_base_addr, 0x00);
						usleep(200000);
						if(player2_val == random_nr)
						{
							xil_printf(RED"\n\r###### Player 1 lost the game! ######\n\r");
							xil_printf(CYAN"\n\r###### Player 2 won the game! ######\n\r");
							Xil_Out8(led_base_addr, 0x07);
						}
						else
						{
							xil_printf(RED"\n\r###### Player 1 won the game! ######\n\r");
							xil_printf(CYAN"\n\r###### Player 2 lost the game! ######\n\r");
							Xil_Out8(led_base_addr, 0x70);
						}
						usleep(350000);
					}
				}
				if(rd_buffer == 3)
					xil_printf(WHITE"\n\r###### ERROR!! GO HOME! ######\n\r");
				xSemaphoreGive( xSemaphore );
			}
    }
}

static void wr_queue_task(void *pvParameters)
{
    /* Remove warning  */
    ( void ) pvParameters;


    BaseType_t wr_queue = (BaseType_t)NULL;
    uint32_t wr_buffer = 10;

    u8 pl_sw_status = 0;
	u32 swit0 = 0;
	u32 swit1 = 0;



    for(;;)
    {
    	random_nr = rand()%40;
    	wr_queue = (BaseType_t)NULL;
    	players_ID = 0;
    	player1_val = 0;
    	player2_val = 0;
    	pl_sw_status = 0;
    	swit0 = 0;
    	swit1 = 0;

    	Xil_Out8(led_base_addr, 0x00);
    	xil_printf(CLS);
    	xil_printf(GREEN"\n\r                                                                    !!!!! START !!!!!\n\r");
    	xil_printf(GREEN"                                                                    Number to reach: %d\n\r", random_nr);
		while((swit0 == 0) && (swit1 == 0))
		{
			while((pl_sw_status == 0) && (swit0 == 0) && (swit1 == 0))
					{
						pl_sw_status = Xil_In8(sw_base_addr);
						swit0 = vParTestGetSW1();
						swit1 = vParTestGetSW0();
					}
			if(pl_sw_status == 1)
				player1_val++;
			if(pl_sw_status == 2)
				player2_val++;

			xil_printf(RED"\n\r###### Player 1 value: %d #####\n\r", player1_val);
			xil_printf(CYAN"\n\r###### Player 2 value: %d #####\n\r", player2_val);
			usleep(10000);
			while((pl_sw_status != 0))
				pl_sw_status = Xil_In8(sw_base_addr);
			usleep(10000);
		}

		if(swit0 == 1)
		{
			players_ID = 1;
			xil_printf(RED"\n\r###### Sendind player 1 response... ######\n\r");
			usleep(10);
		}
		else
		{
			players_ID = 2;
			xil_printf(CYAN"\n\r###### Sendind player 2 response... ######\n\r");
			usleep(10);
		}

		wr_queue = xQueueSend(      xQueue1,
									&players_ID,
									portMAX_DELAY);

		if(wr_queue != pdTRUE )
			xil_printf(WHITE"\n\r###### Couldn't write to the queue! ######\n\r");
    }
}

static void flash_led(void *pvParameters)
{
    /* Remove warning  */
    ( void ) pvParameters;

    const TickType_t xDelay = 100 / portTICK_PERIOD_MS;

//    Block for x ms. //
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = 100;

    // Initialise the xLastWakeTime variable with the current time.
    xLastWakeTime = xTaskGetTickCount();

    u8 a = 0;
    u8 b = 0;
    for(;;)
    {
    	if( xSemaphoreTake( xSemaphore, portMAX_DELAY ) == pdTRUE )
    	{
			xSemaphoreGive( xSemaphore );
			Xil_Out8(led_base_addr, 0x00);
			vTaskDelayUntil( &xLastWakeTime, xFrequency );
			Xil_Out8(led_base_addr, 0xFF);
			vTaskDelayUntil( &xLastWakeTime, xFrequency );
    	}
    }
}
