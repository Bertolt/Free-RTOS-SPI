<<<<<<< HEAD
/*
    FreeRTOS V8.2.3 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/


/*
 *
 * Message Queues
 * 2015-2016
 *
	Ruben Bertelo - 1120803
	Miguel Santos - 1120587
 */
 

/* Standard includes. */
#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Task priorities. */
#define mainFLASH_TASK_PRIORITY	( tskIDLE_PRIORITY + 1)
#define mainLCD_TASK_PRIORITY	( tskIDLE_PRIORITY + 2)
#define mainACCEL_TASK_PRIORITY	( tskIDLE_PRIORITY + 4)
#define mainUSART_TASK_PRIORITY	( tskIDLE_PRIORITY + 3)
#define mainRCV_USART_TASK_PRIORITY	( tskIDLE_PRIORITY + 3)
/* The rate at which the flash task toggles the LED. */
#define mainFLASH_DELAY1000			( ( TickType_t ) 3000 / portTICK_RATE_MS )

 /* Configure the RCC clocks */
static void prvSetupRCC( void );

 /* Configure the GPIO. */
static void prvSetupGPIO( void );

/* Configure the SPI*/
static void prvSetupSPI(void);

/* Configure the Accelerometer */
static void prvSetupAccel( void );

/* USART2 configuration. */
static void prvSetupUSART2( void );

 /* Configure NVIC. */
static void prvSetupNVIC( void );

/**********  Tasks and functions **********/

//Para c-p debug
//GPIO_WriteBit(GPIOB, GPIO_Pin_9, (1-GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_9)));

/* Simple LED toggle task + USART. */
static void prvFlashTask( void *pvParameters );

/* LCD activity task. */
//static void prvLcdTask( void *pvParameters );

/* SPI accleretion read task. */
static void prvAccelTask( void *pvParameters );

/* USART2 send message. */
static void prvRcvUsart(void *pvParameters);

/* LCD display messages. */
static void prvDisplayMessageLCD( void *pvParameters );

/*receive data struct from SPI read and send it to SART char by char*/
static void prvSendDataTask( void *pvParameters );

static void USARTPutChar (char put_char);

/***************** Handlers **********************/

/* Task 1 handle variable. */
TaskHandle_t HandleTask1;
/* Task 2 handle variable. */
TaskHandle_t HandleTask2;
/* Task 3 handle variable. */
TaskHandle_t HandleTask3;
/* Task 4 handle variable. */
TaskHandle_t HandleTask4;
/* Task 5 handle variable. */
TaskHandle_t HandleTask5;


/* Filas de mensagens */
QueueHandle_t xQueueDadosSPI;
QueueHandle_t xQueueSingleSPI;
QueueHandle_t xQueueUsart;
QueueHandle_t xQueueUsartRec;


//SPI data structure
typedef struct
{
        int16_t x;
        int16_t y;
        int16_t z;
        uint32_t xTickCount;
        uint32_t xTickDiff;

}xdadosSPI;

/********************* MAIN *********************/
int main( void )
{


	/*Setup the hardware, RCC, GPIO, etc...*/
    prvSetupRCC();
    prvSetupGPIO();
    prvSetupSPI();
    prvSetupAccel();
    prvSetupUSART2();

    /* Create Queue */
    xQueueDadosSPI = xQueueCreate(30, sizeof( xdadosSPI ));
    xQueueUsart = xQueueCreate(30, sizeof( char ));
    xQueueUsartRec = xQueueCreate(30, sizeof( char ));
    xQueueSingleSPI = xQueueCreate(30, sizeof( int8_t ));
    
    prvSetupNVIC();
    DRAW_Init();
	
	
    if( xQueueSingleSPI == 0)
    {
       /* Queue was not created and must not be used */
    
    }
    else
    {
       /* Queue created successfully */
 
    }
	/* Start the tasks */
    xTaskCreate( prvDisplayMessageLCD, "Lcd", configMINIMAL_STACK_SIZE + 200, NULL, mainLCD_TASK_PRIORITY, &HandleTask1); 	
    xTaskCreate( prvFlashTask, "Flash", configMINIMAL_STACK_SIZE , NULL , mainFLASH_TASK_PRIORITY, &HandleTask2 );
    xTaskCreate( prvAccelTask, "Accel", configMINIMAL_STACK_SIZE + 200, NULL, mainACCEL_TASK_PRIORITY, &HandleTask3 );
    xTaskCreate( prvSendDataTask, "SendDadosUsart", configMINIMAL_STACK_SIZE+200, NULL, mainUSART_TASK_PRIORITY, &HandleTask4 );
    xTaskCreate( prvRcvUsart, "ReceiveDadosUsart", configMINIMAL_STACK_SIZE+200, NULL, mainRCV_USART_TASK_PRIORITY, &HandleTask5 );
	/* Start the scheduler. */
	vTaskStartScheduler();
	
	/* Will only get here if there was not enough heap space to create the idle task. */
	return 0;
}


/********************* INTERRUPTIONS *********************/
/* Handler da interrupçao da USART*/

void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2, USART_IT_TXE) == SET)
    {

        BaseType_t pxHigherPriorityTaskWoken;
        char carater;
        if(xQueueUsart != 0)
        {
            if (xQueueReceiveFromISR(xQueueUsart, &carater, &pxHigherPriorityTaskWoken) != pdPASS);
                {
                    USART_ITConfig(USART2, USART_IT_TXE, DISABLE); 
                }
        }
        
        USART_SendData(USART2, carater);
        USART_ClearITPendingBit(USART2, USART_IT_TXE);

    }
    else if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
    {
        
        char cRxData;
        BaseType_t pxHigherPriorityTaskWoken;
    
        //Ler os dados da USART
        cRxData = USART_ReceiveData(USART2);
        xQueueSendToBackFromISR( xQueueUsartRec, &cRxData, &pxHigherPriorityTaskWoken);
        
        if( pxHigherPriorityTaskWoken == pdTRUE )
        //forces the context change
        taskYIELD();   
        //Limpar a flag
        USART_ClearITPendingBit(USART2,USART_IT_RXNE);
    }
      
}

/* Handler da interrupçao da SPI*/



void SPI2_IRQHandler(void)
{
    int singleSPI;
 
    if(SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_RXNE) == SET)
    {
    SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, DISABLE); 
    BaseType_t pxHigherPriorityTaskWoken;
    if(xQueueSingleSPI != 0)
        {
        singleSPI = SPI_I2S_ReceiveData(SPI2);;//Ler o valor spi
        xQueueSendToBackFromISR(xQueueSingleSPI,&singleSPI, &pxHigherPriorityTaskWoken);
           if( pxHigherPriorityTaskWoken == pdTRUE )
            //forces the context change
            taskYIELD();   
        }
            //Limpar a flag
       SPI_I2S_ClearITPendingBit(SPI2, SPI_I2S_IT_RXNE);
    } 
 
     if(SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_TXE) == SET){
        SPI_I2S_SendData(SPI2, 0xE8);                  //primeiro conjunto de dados ( primeiro bit a 1 (Leitura), segundo bit a 1 (Autotincremento), 0x28 x_low)
        SPI_I2S_ClearITPendingBit(SPI2, SPI_I2S_IT_TXE);
     }
}
    


/********************* TASKS *********************/

/* Example task to present characteres in ther display. */
static void prvDisplayMessageLCD( void *pvParameters )
{
    /*The LCD must be initialized first with function DRAW_Init();
    Use only 10 numbers in the var 'line_number' (0 to 9)*/
    char buffer1 [25];
    char buffer2 [25];
    char buffer3 [25];

    xdadosSPI dadosSPI ;
 
    TickType_t xLastExecutionTime;

    xLastExecutionTime = xTaskGetTickCount();

    for( ;; )
	{
        uint8_t noQueueSpiItems = uxQueueMessagesWaiting( xQueueDadosSPI );
        uint8_t noQueueUsartItems = uxQueueMessagesWaiting( xQueueUsart );
        uint8_t noTaskKernel = uxTaskGetNumberOfTasks();
    
        if( xQueueDadosSPI != 0 )
            {
                 if(xQueuePeek(xQueueDadosSPI , &dadosSPI, (TickType_t) 10) == pdTRUE)
                     {
                        sprintf (buffer3 ,"read time:%d ", dadosSPI.xTickDiff );
                        DRAW_DisplayString(5, 90, buffer3 , strlen( buffer3) );
                     }
                
            }
        

        sprintf (buffer1 ,"NoSPI:%d UartIt:%d ", noQueueSpiItems, noQueueUsartItems );
        sprintf (buffer2 ,"no Tasks:%d ", noTaskKernel );
        
        
        /*Clear the lcd*/

        vTaskDelayUntil( &xLastExecutionTime, mainFLASH_DELAY1000 );        
        LCD_FillRect(0,0,127,127,0xFFFF);
		/*write the message in the display*/
        DRAW_DisplayString(5, 50, buffer1 , strlen( buffer1) );
        DRAW_DisplayString(5, 70, buffer2 , strlen( buffer2) );
        
        
    
    
    }
}
/*-----------------------------------------------------------*/

/* Debug Task - LED blink. */

static void prvFlashTask( void *pvParameters )
{
    TickType_t xLastExecutionTime;

    xLastExecutionTime = xTaskGetTickCount();
    for( ;; )
	{
        vTaskDelayUntil( &xLastExecutionTime, mainFLASH_DELAY1000 );
        GPIO_WriteBit(GPIOB, GPIO_Pin_9, (1-GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_9)));
        
	}
}
static void prvRcvUsart( void *pvParameters )
{
    char cRxData;
    
    
	for( ;; )
	{   
        xQueueReceive(xQueueUsartRec, &cRxData, (TickType_t) 10);
    
        if (cRxData == 's' || cRxData == 'S'){
        
               vTaskSuspend ( HandleTask4 );
        
        }
        if (cRxData == 'r' || cRxData == 'R'){
        
               vTaskResume( HandleTask4 );
        
        }
	}
}
/*-----------------------------------------------------------*/

/* Acceleration task -  to read the SPI and get the x y z values. */

static void prvAccelTask( void *pvParameters )
{
    xdadosSPI dadosSPI ;

    TickType_t xLastExecutionTime;
    
    int8_t singleSPI[7];

    int cont=0;


for(;;){

        
        GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_RESET);  //Chip Select enable

        for(cont=0; cont<7; cont++){
            SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, ENABLE);
            xQueueReceive(xQueueSingleSPI, &singleSPI[cont], (TickType_t) portMAX_DELAY); 
        }
      
        GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_SET);                          //Chip Select disable
        
        dadosSPI.x = (singleSPI[2] << 8) + singleSPI[1];
        dadosSPI.y = (singleSPI[4] << 8) + singleSPI[3];
        dadosSPI.z = (singleSPI[6] << 8) + singleSPI[5];
    
        dadosSPI.xTickDiff = dadosSPI.xTickCount;
        dadosSPI.xTickCount = xTaskGetTickCount();
        dadosSPI.xTickDiff = dadosSPI.xTickCount - dadosSPI.xTickDiff;
        
        xQueueSendToBack(xQueueDadosSPI, (void *) &dadosSPI, (TickType_t) portMAX_DELAY);
	}

    
}
/*-----------------------------------------------------------*/

static void prvSendDataTask( void *pvParameters )
{
    xdadosSPI dadosSPI;
    char buffer [30];
    int i;
    for( ;; )
	{   
    if(xQueueReceive(xQueueDadosSPI, &dadosSPI, (TickType_t) portMAX_DELAY)== pdTRUE)
        {
            sprintf (buffer ,"x:%d y:%d z:%d ticks:%d\r\n", dadosSPI.x, dadosSPI.y, dadosSPI.z, (uint32_t) dadosSPI.xTickCount );
            for(i=0; i<strlen(buffer);i++)
                {
                    USARTPutChar(buffer [i]); 
                }
        }
    }
}


/*********************   FUNCTIONS   *********************/
static void USARTPutChar ( char put_char )
{
    xQueueSendToBack(xQueueUsart, (void *) &put_char, (TickType_t) portMAX_DELAY);
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);           // Ativar a flag TXE da interrupção
    
}

/*********************     SETUP     *********************/

static void prvSetupRCC( void )
{
    /* RCC configuration - 72 MHz */
 
    ErrorStatus HSEStartUpStatus;

    RCC_DeInit();
    /*Enable the HSE*/
    RCC_HSEConfig(RCC_HSE_ON); 
    /* Wait untill HSE is ready or time out */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
    if(HSEStartUpStatus == SUCCESS)
    {
        /* Enable The Prefetch Buffer */
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
        /* 72 MHZ - 2 wait states */
        FLASH_SetLatency(FLASH_Latency_2);
    
        /* No division HCLK = SYSCLK */
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        /* PCLK1 = HCLK/2 (36MHz) */
        RCC_PCLK1Config(RCC_HCLK_Div2);
        /* PCLK2 = HCLK (72MHz)*/
        RCC_PCLK2Config(RCC_HCLK_Div1);

        /* Use PLL with HSE=12MHz */
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_6);
        /* Enable the PLL */
        RCC_PLLCmd(ENABLE);
        /* Wait for PLL ready */
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET );
        
        /* Select the PLL as system clock source */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        /* Wait until PLL is used as system clock */
        while( RCC_GetSYSCLKSource() != 0x08 );
    }
    else 
    {
        while(1);
    }
}
/*-----------------------------------------------------------*/


static void prvSetupGPIO( void )
{
    /* GPIO configuration - GREEN LED*/

    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIOB clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB , ENABLE );

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOB, &GPIO_InitStructure);	
}

/*-----------------------------------------------------------*/

//SPI Configuração
static void prvSetupSPI(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);       // Enable do clock do periferico
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);        // Enable do SPI

    // SPI
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13 | GPIO_Pin_15;           // Configura o SCK MISO MOSI
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                         //  SPI é alternate function
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;                         //  SPI é alternate function
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;                               // Chip Select
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure); 

    SPI_InitTypeDef SPI_InitStructure;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;      // 9 MHz
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    
    SPI_Init(SPI2, &SPI_InitStructure); //Initializes SPI peripheral with Struct values
    SPI_Cmd(SPI2, ENABLE); //Enable SPI peripheral
    
    
 }   
/*-----------------------------------------------------------*/


static void prvSetupAccel( void )
{
    GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_RESET);
    SPI_I2S_SendData(SPI2, 0x60);                                                 //Primeiro bit a 0 (Escrita), segundo bit a 1 (Autoincremento), 0x20 Control Register 1
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI2, 0xC7);                                                 //Ligar o Accel e ativar os 3 eixos
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI2, 0x44);                                                 //Full Scale(6g), 12 bits right justified, little endian
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_SET);
}
/*-----------------------------------------------------------*/



void prvSetupUSART2( void )
{
USART_InitTypeDef USART_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;

    /* USART2 is configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - 1 Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled */

    /* Enable GPIOA clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA , ENABLE );

    /* USART Periph clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    
    /* Configure the USART2 */ 
    USART_Init(USART2, &USART_InitStructure);
    /* Enable the USART2 */
    USART_Cmd(USART2, ENABLE);
 }
/*-----------------------------------------------------------*/


static void prvSetupNVIC (void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel= USART2_IRQChannel;  // Interrupção da USART2
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; // Prioridade 0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;        // Sub prioridade 0
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;           // Ativação da interrupção

    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel= SPI2_IRQChannel;  // Interrupção da SPI2
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; // Prioridade 0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;        // Sub prioridade 0
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;           // Ativação da interrupção
    NVIC_Init(&NVIC_InitStructure); 
    
    // Inicialização da USART
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);          // Ativar a flag RXNE da interrupção
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);           // Ativar a flag TXE da interrupção
    SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);          // Ativar a flag TXE da interrupção
    //SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, ENABLE);          // Ativar a flag RXNE da interrupção

    
}
/*-----------------------------------------------------------*/

/**************** OTHER FUNCTIONS and TASKS *************/

/*
static void prvLcdTask( void *pvParameters )
{
    char cRxData;
    
    int y=0;
    LCD_FillRect( 0, 0, 127, 127, 0xFFFF);
	for( ;; )
	{   
        xQueueReceive(xQueueUsartRec, &cRxData, (TickType_t) portMAX_DELAY);
        //DRAW_DisplayString (32, 0 , cRxData, sizeof(char));
        LCD_DisplayChar( 32, 0, cRxData, 0x0000, 0xFFFF, 10);
        vTaskDelay( ( TickType_t ) 300 / portTICK_RATE_MS);
	}
}
/*-----------------------------------------------------------

static void prvSendMessageUSART2(char *message)
{
uint16_t cont_aux=0;
    
    while(cont_aux != strlen(message))
    {
        USART_SendData(USART2, (uint8_t) message[cont_aux]);
        while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
        {
        }
        cont_aux++;
    }
}
/*-----------------------------------------------------------*/
=======
/*
    FreeRTOS V8.2.3 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/


/*
 *
 * Message Queues
 * 2015-2016
 *
	Ruben Bertelo - 1120803
	Miguel Santos - 1120587
 */
 

/* Standard includes. */
#include <string.h>
#include <stdio.h>
#include <stdint.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Task priorities. */
#define mainFLASH_TASK_PRIORITY	( tskIDLE_PRIORITY + 1)
#define mainLCD_TASK_PRIORITY	( tskIDLE_PRIORITY + 2)
#define mainACCEL_TASK_PRIORITY	( tskIDLE_PRIORITY + 4)
#define mainUSART_TASK_PRIORITY	( tskIDLE_PRIORITY + 3)
#define mainRCV_USART_TASK_PRIORITY	( tskIDLE_PRIORITY + 3)
/* The rate at which the flash task toggles the LED. */
#define mainFLASH_DELAY1000			( ( TickType_t ) 3000 / portTICK_RATE_MS )

 /* Configure the RCC clocks */
static void prvSetupRCC( void );

 /* Configure the GPIO. */
static void prvSetupGPIO( void );

/* Configure the SPI*/
static void prvSetupSPI(void);

/* Configure the Accelerometer */
static void prvSetupAccel( void );

/* USART2 configuration. */
static void prvSetupUSART2( void );

 /* Configure NVIC. */
static void prvSetupNVIC( void );

/**********  Tasks and functions **********/

//Para c-p debug
//GPIO_WriteBit(GPIOB, GPIO_Pin_9, (1-GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_9)));

/* Simple LED toggle task + USART. */
static void prvFlashTask( void *pvParameters );

/* LCD activity task. */
//static void prvLcdTask( void *pvParameters );

/* SPI accleretion read task. */
static void prvAccelTask( void *pvParameters );

/* USART2 send message. */
static void prvRcvUsart(void *pvParameters);

/* LCD display messages. */
static void prvDisplayMessageLCD( void *pvParameters );

/*receive data struct from SPI read and send it to SART char by char*/
static void prvSendDataTask( void *pvParameters );

static void USARTPutChar (char put_char);

/***************** Handlers **********************/

/* Task 1 handle variable. */
TaskHandle_t HandleTask1;
/* Task 2 handle variable. */
TaskHandle_t HandleTask2;
/* Task 3 handle variable. */
TaskHandle_t HandleTask3;
/* Task 4 handle variable. */
TaskHandle_t HandleTask4;
/* Task 5 handle variable. */
TaskHandle_t HandleTask5;


/* Filas de mensagens */
QueueHandle_t xQueueDadosSPI;
QueueHandle_t xQueueSingleSPI;
QueueHandle_t xQueueUsart;
QueueHandle_t xQueueUsartRec;


//SPI data structure
typedef struct
{
        int16_t x;
        int16_t y;
        int16_t z;
        uint32_t xTickCount;
        uint32_t xTickDiff;

}xdadosSPI;

/********************* MAIN *********************/
int main( void )
{


	/*Setup the hardware, RCC, GPIO, etc...*/
    prvSetupRCC();
    prvSetupGPIO();
    prvSetupSPI();
    prvSetupAccel();
    prvSetupUSART2();

    /* Create Queue */
    xQueueDadosSPI = xQueueCreate(30, sizeof( xdadosSPI ));
    xQueueUsart = xQueueCreate(30, sizeof( char ));
    xQueueUsartRec = xQueueCreate(30, sizeof( char ));
    xQueueSingleSPI = xQueueCreate(30, sizeof( int8_t ));
    
    prvSetupNVIC();
    DRAW_Init();
	
	
    if( xQueueSingleSPI == 0)
    {
       /* Queue was not created and must not be used */
    
    }
    else
    {
       /* Queue created successfully */
 
    }
	/* Start the tasks */
    xTaskCreate( prvDisplayMessageLCD, "Lcd", configMINIMAL_STACK_SIZE + 200, NULL, mainLCD_TASK_PRIORITY, &HandleTask1); 	
    xTaskCreate( prvFlashTask, "Flash", configMINIMAL_STACK_SIZE , NULL , mainFLASH_TASK_PRIORITY, &HandleTask2 );
    xTaskCreate( prvAccelTask, "Accel", configMINIMAL_STACK_SIZE + 200, NULL, mainACCEL_TASK_PRIORITY, &HandleTask3 );
    xTaskCreate( prvSendDataTask, "SendDadosUsart", configMINIMAL_STACK_SIZE+200, NULL, mainUSART_TASK_PRIORITY, &HandleTask4 );
    xTaskCreate( prvRcvUsart, "ReceiveDadosUsart", configMINIMAL_STACK_SIZE+200, NULL, mainRCV_USART_TASK_PRIORITY, &HandleTask5 );
	/* Start the scheduler. */
	vTaskStartScheduler();
	
	/* Will only get here if there was not enough heap space to create the idle task. */
	return 0;
}


/********************* INTERRUPTIONS *********************/
/* Handler da interrupçao da USART*/

void USART2_IRQHandler(void)
{
    if(USART_GetITStatus(USART2, USART_IT_TXE) == SET)
    {

        BaseType_t pxHigherPriorityTaskWoken;
        char carater;
        if(xQueueUsart != 0)
        {
            if (xQueueReceiveFromISR(xQueueUsart, &carater, &pxHigherPriorityTaskWoken) != pdPASS);
                {
                    USART_ITConfig(USART2, USART_IT_TXE, DISABLE); 
                }
        }
        
        USART_SendData(USART2, carater);
        USART_ClearITPendingBit(USART2, USART_IT_TXE);

    }
    else if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
    {
        
        char cRxData;
        BaseType_t pxHigherPriorityTaskWoken;
    
        //Ler os dados da USART
        cRxData = USART_ReceiveData(USART2);
        xQueueSendToBackFromISR( xQueueUsartRec, &cRxData, &pxHigherPriorityTaskWoken);
        
        if( pxHigherPriorityTaskWoken == pdTRUE )
        //forces the context change
        taskYIELD();   
        //Limpar a flag
        USART_ClearITPendingBit(USART2,USART_IT_RXNE);
    }
      
}

/* Handler da interrupçao da SPI*/



void SPI2_IRQHandler(void)
{
    int singleSPI;
 
    if(SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_RXNE) == SET)
    {
    SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, DISABLE); 
    BaseType_t pxHigherPriorityTaskWoken;
    if(xQueueSingleSPI != 0)
        {
        singleSPI = SPI_I2S_ReceiveData(SPI2);;//Ler o valor spi
        xQueueSendToBackFromISR(xQueueSingleSPI,&singleSPI, &pxHigherPriorityTaskWoken);
           if( pxHigherPriorityTaskWoken == pdTRUE )
            //forces the context change
            taskYIELD();   
        }
            //Limpar a flag
       SPI_I2S_ClearITPendingBit(SPI2, SPI_I2S_IT_RXNE);
    } 
 
     if(SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_TXE) == SET){
        SPI_I2S_SendData(SPI2, 0xE8);                  //primeiro conjunto de dados ( primeiro bit a 1 (Leitura), segundo bit a 1 (Autotincremento), 0x28 x_low)
        SPI_I2S_ClearITPendingBit(SPI2, SPI_I2S_IT_TXE);
     }
}
    


/********************* TASKS *********************/

/* Example task to present characteres in ther display. */
static void prvDisplayMessageLCD( void *pvParameters )
{
    /*The LCD must be initialized first with function DRAW_Init();
    Use only 10 numbers in the var 'line_number' (0 to 9)*/
    char buffer1 [25];
    char buffer2 [25];
    char buffer3 [25];

    xdadosSPI dadosSPI ;
 
    TickType_t xLastExecutionTime;

    xLastExecutionTime = xTaskGetTickCount();

    for( ;; )
	{
        uint8_t noQueueSpiItems = uxQueueMessagesWaiting( xQueueDadosSPI );
        uint8_t noQueueUsartItems = uxQueueMessagesWaiting( xQueueUsart );
        uint8_t noTaskKernel = uxTaskGetNumberOfTasks();
    
        if( xQueueDadosSPI != 0 )
            {
                 if(xQueuePeek(xQueueDadosSPI , &dadosSPI, (TickType_t) 10) == pdTRUE)
                     {
                        sprintf (buffer3 ,"read time:%d ", dadosSPI.xTickDiff );
                        DRAW_DisplayString(5, 90, buffer3 , strlen( buffer3) );
                     }
                
            }
        

        sprintf (buffer1 ,"NoSPI:%d UartIt:%d ", noQueueSpiItems, noQueueUsartItems );
        sprintf (buffer2 ,"no Tasks:%d ", noTaskKernel );
        
        
        /*Clear the lcd*/

        vTaskDelayUntil( &xLastExecutionTime, mainFLASH_DELAY1000 );        
        LCD_FillRect(0,0,127,127,0xFFFF);
		/*write the message in the display*/
        DRAW_DisplayString(5, 50, buffer1 , strlen( buffer1) );
        DRAW_DisplayString(5, 70, buffer2 , strlen( buffer2) );
        
        
    
    
    }
}
/*-----------------------------------------------------------*/

/* Debug Task - LED blink. */

static void prvFlashTask( void *pvParameters )
{
    TickType_t xLastExecutionTime;

    xLastExecutionTime = xTaskGetTickCount();
    for( ;; )
	{
        vTaskDelayUntil( &xLastExecutionTime, mainFLASH_DELAY1000 );
        GPIO_WriteBit(GPIOB, GPIO_Pin_9, (1-GPIO_ReadOutputDataBit(GPIOB, GPIO_Pin_9)));
        
	}
}
static void prvRcvUsart( void *pvParameters )
{
    char cRxData;
    
    
	for( ;; )
	{   
        xQueueReceive(xQueueUsartRec, &cRxData, (TickType_t) 10);
    
        if (cRxData == 's' || cRxData == 'S'){
        
               vTaskSuspend ( HandleTask4 );
        
        }
        if (cRxData == 'r' || cRxData == 'R'){
        
               vTaskResume( HandleTask4 );
        
        }
	}
}
/*-----------------------------------------------------------*/

/* Acceleration task -  to read the SPI and get the x y z values. */

static void prvAccelTask( void *pvParameters )
{
    xdadosSPI dadosSPI ;

    TickType_t xLastExecutionTime;
    
    int8_t singleSPI[7];

    int cont=0;


for(;;){

        
        GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_RESET);  //Chip Select enable

        for(cont=0; cont<7; cont++){
            SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, ENABLE);
            xQueueReceive(xQueueSingleSPI, &singleSPI[cont], (TickType_t) portMAX_DELAY); 
        }
      
        GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_SET);                          //Chip Select disable
        
        dadosSPI.x = (singleSPI[2] << 8) + singleSPI[1];
        dadosSPI.y = (singleSPI[4] << 8) + singleSPI[3];
        dadosSPI.z = (singleSPI[6] << 8) + singleSPI[5];
    
        dadosSPI.xTickDiff = dadosSPI.xTickCount;
        dadosSPI.xTickCount = xTaskGetTickCount();
        dadosSPI.xTickDiff = dadosSPI.xTickCount - dadosSPI.xTickDiff;
        
        xQueueSendToBack(xQueueDadosSPI, (void *) &dadosSPI, (TickType_t) portMAX_DELAY);
	}

    
}
/*-----------------------------------------------------------*/

static void prvSendDataTask( void *pvParameters )
{
    xdadosSPI dadosSPI;
    char buffer [30];
    int i;
    for( ;; )
	{   
    if(xQueueReceive(xQueueDadosSPI, &dadosSPI, (TickType_t) portMAX_DELAY)== pdTRUE)
        {
            sprintf (buffer ,"x:%d y:%d z:%d ticks:%d\r\n", dadosSPI.x, dadosSPI.y, dadosSPI.z, (uint32_t) dadosSPI.xTickCount );
            for(i=0; i<strlen(buffer);i++)
                {
                    USARTPutChar(buffer [i]); 
                }
        }
    }
}


/*********************   FUNCTIONS   *********************/
static void USARTPutChar ( char put_char )
{
    xQueueSendToBack(xQueueUsart, (void *) &put_char, (TickType_t) portMAX_DELAY);
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);           // Ativar a flag TXE da interrupção
    
}

/*********************     SETUP     *********************/

static void prvSetupRCC( void )
{
    /* RCC configuration - 72 MHz */
 
    ErrorStatus HSEStartUpStatus;

    RCC_DeInit();
    /*Enable the HSE*/
    RCC_HSEConfig(RCC_HSE_ON); 
    /* Wait untill HSE is ready or time out */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
    if(HSEStartUpStatus == SUCCESS)
    {
        /* Enable The Prefetch Buffer */
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
        /* 72 MHZ - 2 wait states */
        FLASH_SetLatency(FLASH_Latency_2);
    
        /* No division HCLK = SYSCLK */
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        /* PCLK1 = HCLK/2 (36MHz) */
        RCC_PCLK1Config(RCC_HCLK_Div2);
        /* PCLK2 = HCLK (72MHz)*/
        RCC_PCLK2Config(RCC_HCLK_Div1);

        /* Use PLL with HSE=12MHz */
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_6);
        /* Enable the PLL */
        RCC_PLLCmd(ENABLE);
        /* Wait for PLL ready */
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET );
        
        /* Select the PLL as system clock source */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        /* Wait until PLL is used as system clock */
        while( RCC_GetSYSCLKSource() != 0x08 );
    }
    else 
    {
        while(1);
    }
}
/*-----------------------------------------------------------*/


static void prvSetupGPIO( void )
{
    /* GPIO configuration - GREEN LED*/

    GPIO_InitTypeDef GPIO_InitStructure;

    /* Enable GPIOB clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB , ENABLE );

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOB, &GPIO_InitStructure);	
}

/*-----------------------------------------------------------*/

//SPI Configuração
static void prvSetupSPI(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
  
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);       // Enable do clock do periferico
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);        // Enable do SPI

    // SPI
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13 | GPIO_Pin_15;           // Configura o SCK MISO MOSI
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;                         //  SPI é alternate function
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;                         //  SPI é alternate function
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;                               // Chip Select
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure); 

    SPI_InitTypeDef SPI_InitStructure;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;      // 9 MHz
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    
    SPI_Init(SPI2, &SPI_InitStructure); //Initializes SPI peripheral with Struct values
    SPI_Cmd(SPI2, ENABLE); //Enable SPI peripheral
    
    
 }   
/*-----------------------------------------------------------*/


static void prvSetupAccel( void )
{
    GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_RESET);
    SPI_I2S_SendData(SPI2, 0x60);                                                 //Primeiro bit a 0 (Escrita), segundo bit a 1 (Autoincremento), 0x20 Control Register 1
	while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI2, 0xC7);                                                 //Ligar o Accel e ativar os 3 eixos
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
    SPI_I2S_SendData(SPI2, 0x44);                                                 //Full Scale(6g), 12 bits right justified, little endian
    while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_TXE) == RESET);
	GPIO_WriteBit(GPIOD, GPIO_Pin_2, Bit_SET);
}
/*-----------------------------------------------------------*/



void prvSetupUSART2( void )
{
USART_InitTypeDef USART_InitStructure;
GPIO_InitTypeDef GPIO_InitStructure;

    /* USART2 is configured as follow:
        - BaudRate = 115200 baud  
        - Word Length = 8 Bits
        - 1 Stop Bit
        - No parity
        - Hardware flow control disabled (RTS and CTS signals)
        - Receive and transmit enabled */

    /* Enable GPIOA clock */
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA , ENABLE );

    /* USART Periph clock enable */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);


    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    
    /* Configure the USART2 */ 
    USART_Init(USART2, &USART_InitStructure);
    /* Enable the USART2 */
    USART_Cmd(USART2, ENABLE);
 }
/*-----------------------------------------------------------*/


static void prvSetupNVIC (void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel= USART2_IRQChannel;  // Interrupção da USART2
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; // Prioridade 0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;        // Sub prioridade 0
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;           // Ativação da interrupção

    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel= SPI2_IRQChannel;  // Interrupção da SPI2
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0; // Prioridade 0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority=0;        // Sub prioridade 0
    NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;           // Ativação da interrupção
    NVIC_Init(&NVIC_InitStructure); 
    
    // Inicialização da USART
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);          // Ativar a flag RXNE da interrupção
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);           // Ativar a flag TXE da interrupção
    SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);          // Ativar a flag TXE da interrupção
    //SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_TXE, ENABLE);          // Ativar a flag RXNE da interrupção

    
}
/*-----------------------------------------------------------*/

/**************** OTHER FUNCTIONS and TASKS *************/

/*
static void prvLcdTask( void *pvParameters )
{
    char cRxData;
    
    int y=0;
    LCD_FillRect( 0, 0, 127, 127, 0xFFFF);
	for( ;; )
	{   
        xQueueReceive(xQueueUsartRec, &cRxData, (TickType_t) portMAX_DELAY);
        //DRAW_DisplayString (32, 0 , cRxData, sizeof(char));
        LCD_DisplayChar( 32, 0, cRxData, 0x0000, 0xFFFF, 10);
        vTaskDelay( ( TickType_t ) 300 / portTICK_RATE_MS);
	}
}
/*-----------------------------------------------------------

static void prvSendMessageUSART2(char *message)
{
uint16_t cont_aux=0;
    
    while(cont_aux != strlen(message))
    {
        USART_SendData(USART2, (uint8_t) message[cont_aux]);
        while(USART_GetFlagStatus(USART2, USART_FLAG_TXE) == RESET)
        {
        }
        cont_aux++;
    }
}
/*-----------------------------------------------------------*/
>>>>>>> f31ef0c40d65824725b864cdc9276e0e9db6c20b
