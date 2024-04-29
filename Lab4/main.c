#include "stm32l476xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <stdio.h>
#define STACK_SIZE 128
#define SCALE_FACTOR 10


//GPIO A5 LED
//GPIO C13 Switch

#define A3_PSC 15
#define A3_ARR 70
#define B3_PSC 10
#define B3_ARR 91
#define C4_PSC 4
#define C4_ARR 190
#define D4_PSC 11
#define D4_ARR 70
#define E4_PSC 10
#define E4_ARR 68
#define F4_PSC 3
#define F4_ARR 178
#define G4_PSC 10
#define G4_ARR 57
#define A4_PSC 7
#define A4_ARR 70

void clockConfig(void);
void gpioConfig(void);
void tim4Config(void);
void usartConfig(void);
void TIM4_IRQHandler(void);
void dacConfig(void);
void led_task( void *pvParameters );
void but_task( void *pvParameters );
void us100_task( void *pvParameters);

void tim4Reconfig( uint32_t psc, uint32_t arr );

void USART_Init( USART_TypeDef * USARTx );
//void USART2_IRQHandler ( void );
void USART_Read(USART_TypeDef* USARTx, uint8_t *buffer, uint32_t nBytes);
void USART_Write(USART_TypeDef* USARTx, uint8_t *buffer, uint32_t nBytes);

static const uint16_t sineLookupTable[] = {
0x200, 0x232, 0x263, 0x294, 0x2c3, 0x2f1, 0x31c, 0x344,
0x369, 0x38b, 0x3a9, 0x3c3, 0x3d8, 0x3e9, 0x3f5, 0x3fd,
0x3ff, 0x3fd, 0x3f5, 0x3e9, 0x3d8, 0x3c3, 0x3a9, 0x38b,
0x369, 0x344, 0x31c, 0x2f1, 0x2c3, 0x294, 0x263, 0x232,
0x200, 0x1cd, 0x19c, 0x16b, 0x13c, 0x10e, 0xe3, 0xbb,
0x96, 0x74, 0x56, 0x3c, 0x27, 0x16, 0x0a, 0x02,
0x00, 0x02, 0x0a, 0x16, 0x27, 0x3c, 0x56, 0x74,
0x96, 0xbb, 0xe3, 0x10e, 0x13c, 0x16b, 0x19c, 0x1cd};

static volatile QueueHandle_t LEDQueue;
static volatile QueueHandle_t LEDQueue2;
static volatile QueueHandle_t LetterQueue;
static int sineValue;

int main( void )
{
	sineValue = 0;
	clockConfig();
	gpioConfig();
	dacConfig();

	//USART_Init(USART2);
	

	LetterQueue = xQueueCreate( 1, sizeof( uint8_t ) );
	LEDQueue = xQueueCreate( 1, sizeof( uint8_t ) );
	LEDQueue2 = xQueueCreate( 1, sizeof( uint8_t ) );
	


	if(LEDQueue != NULL && LetterQueue != NULL){
			BaseType_t ledHand;
			BaseType_t butHand;
			BaseType_t US100Hand;

			tim4Config();
			usartConfig();
			
			
			ledHand = xTaskCreate(
											led_task,       /* Function that implements the task. */
											"LED is either on or off",          /* Text name for the task. */
											STACK_SIZE,      /* Stack size in words, not bytes. */
											NULL,    /* Parameter passed into the task. */
											2,/* Priority at which the task is created. */
											NULL );      /* Used to pass out the created task's handle. */
			butHand = xTaskCreate(
									but_task,       /* Function that implements the task. */
									"Button is clicked or not",          /* Text name for the task. */
									STACK_SIZE,      /* Stack size in words, not bytes. */
									(void * ) LEDQueue,    /* Parameter passed into the task. */
									2,/* Priority at which the task is created. */
									NULL );      /* Used to pass out the created task's handle. */

			US100Hand = xTaskCreate(
									us100_task,       /* Function that implements the task. */
									"Button is clicked or not",          /* Text name for the task. */
									STACK_SIZE,      /* Stack size in words, not bytes. */
									NULL,    /* Parameter passed into the task. */
									2,/* Priority at which the task is created. */
									NULL );      /* Used to pass out the created task's handle. */


			if( ledHand == pdPASS && butHand == pdPASS && US100Hand)
			{
					/* The task was created.  Use the task's handle to delete the task. */
					//vTaskDelete( xHandle );
					
			}

	/* Create the task, storing the handle. */
		}
		vTaskStartScheduler();
		while(1);

}

void usartConfig(void){
		//UART Config
	//USART4 Init for the HS100
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	GPIOA->MODER &= ~(0xF << (2*2));
	GPIOA->MODER |= 0xA << (2*2);
	GPIOA->AFR[0] |= 0x77 << (4*2);
	GPIOA->OSPEEDR |= 0xF<<(2*2);
	GPIOA->PUPDR &= ~(0xF<<(2*2));
	GPIOA->PUPDR |= 0x5<<(2*2);
	GPIOA->OTYPER &= ~(0x3<<2);
	
	USART_Init(USART2);	

	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;
	GPIOB->MODER &= ~(0xF << (2*6));
	GPIOB->MODER |= 0xA << (2*6);

	GPIOB->AFR[0] |= 0x77 << (4*6);
	GPIOB->OSPEEDR |= 0xF<<(2*6);

	GPIOB->PUPDR &= ~(0xF<<(2*6));
	GPIOB->PUPDR |= 0x5<<(2*6);

	GPIOB->OTYPER &= ~(0x3<<6);

	USART_Init(USART1);

	//GPIOA->MODER |= 0x3;
	//GPIOA->ASCR |= 0x1;
	//This is for PB 6 & 7
}

void USART_Init( USART_TypeDef * USARTx ) {
	// Enable the peripheral clock of GPIO Port
	
	// Enable the peripheral clock of USART
	if( USARTx == USART1 ) {
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
	} else if( USARTx == USART2 ) {
		RCC->APB1ENR1 |= RCC_APB1ENR1_USART2EN;
		RCC->CCIPR &= ~(RCC_CCIPR_USART2SEL);
		RCC->CCIPR |= RCC_CCIPR_USART2SEL_0;
	} else if( USARTx == USART3 ) {
		RCC->APB1ENR1 |= RCC_APB1ENR1_USART3EN;
	} else if( USARTx == UART4 ) {
		RCC->APB1ENR1 |= RCC_APB1ENR1_UART4EN;
	} else if( USARTx == UART5 ) {
		RCC->APB1ENR1 |= RCC_APB1ENR1_UART5EN;
	}
	

	// Disable USART
	USARTx->CR1 &= ~USART_CR1_UE;

	// Set data length to 8 bits
	// 00 = 8 bits, 01 = 9 bits, 10 = 7 bits
	USARTx->CR1 &= ~USART_CR1_M; // 0

	// Select 1 stop bit
	// 00 = 1 stop bit, 01 = 0.5 stop bit, 
	// 10 = 2 stop bits, 11 = 1.5 stop bits
	USARTx->CR2 &= ~USART_CR2_STOP; // 00

	// Select no parity
	// 0 = no parity, 1 = parity enabled
	USARTx->CR1 &= ~USART_CR1_PCE; // 0

	// Oversampling by 16
	// 0 = oversampling by 16, 1 = oversampling by 8
	USARTx->CR1 &= ~USART_CR1_OVER8; // 0

	// Select AF mode (10) on PA9 and PA10
	//GPIOA->MODER &= ~(GPIO_MODER_MODE2 | GPIO_MODER_MODE3);
	//GPIOA->MODER |= (GPIO_MODER_MODE2_1 | GPIO_MODER_MODE3_1);
	// AF7 for USART1 and USART2
	

	
//	if( USARTx == USART1 || USARTx == USART2 ) {
//		GPIOA->AFR[1] |= 0x00000770;
//	}
//	// AF7 for USART3, UART4, and UART5
//	else {
//		GPIOA->AFR[1] |= 0x07700000;
//	}
	// Baud rate = fCK / (8 * (2 - OVER8) * USARTDIV)
	// fCK = 16 MHz, OVER8 = 0, Baud rate = 9600
	USARTx->BRR = 0x683; // 1667

	if(USARTx == USART2){
		USARTx->CR1 = USART_CR1_RXNEIE;
		NVIC_EnableIRQ(USART2_IRQn);
		NVIC_SetPriority(USART2_IRQn, 6);
		GPIOA->MODER |= 0x3;
		GPIOA->ASCR |= 0x1;
	}
	// Enable RX, TX, and USART
	USARTx->CR1 |= (USART_CR1_UE | USART_CR1_TE | USART_CR1_RE);
	
	while((USARTx->ISR & USART_ISR_TEACK) == 0);
	
	while ((USARTx->ISR & USART_ISR_REACK) == 0);
	


}

void USART_Read(USART_TypeDef* USARTx, uint8_t *buffer, uint32_t nBytes) {
    for (uint32_t i = 0; i < nBytes; i++) {
        // Wait until data is received
        while((USARTx->ISR & USART_ISR_RXNE) == 0);

        // Read data from USART
        buffer[i] = USARTx->RDR;
    }
}

void USART_Write (USART_TypeDef *USARTx, uint8_t *buffer, uint32_t nBytes) {
	int i;
	for (i = 0; i < nBytes; i++) {
	while (!(USARTx->ISR & USART_ISR_TXE)); // Wait until hardware sets TXE
	USARTx->TDR = buffer[i] & 0xFF; // Writing to TOR clears TXE flag
	}
	// Wait until TC bit is set. TC is set by hardware and cleared by software.
	while (!(USARTx->ISR & USART_ISR_TC)); // TC: Transmission complete flag
	// Writing 1 to the TCCF bit in ICR clears the TC bit in ISR
	USARTx->ICR |= USART_ICR_TCCF; // TCCF: Transmission complete clear flag
}

void USART2_IRQHandler ( void ){
	uint8_t note;
	USART_Read(USART2, &note, 1);
	
	switch(note){
		case 'a':
			tim4Reconfig(A3_PSC, A3_ARR);
			break;
		case 'b':
			tim4Reconfig(B3_PSC, B3_ARR);
			break;
		case 'c':
			tim4Reconfig(C4_PSC, C4_ARR);
			break;
		case 'd':
			tim4Reconfig(D4_PSC, D4_ARR);
			break;
		case 'e':
			tim4Reconfig(E4_PSC, E4_ARR);
			break;
		case 'f':
			tim4Reconfig(F4_PSC, F4_ARR);
			break;
		case 'g':
			tim4Reconfig(G4_PSC, G4_ARR);
			break;
		case 'h':
			tim4Reconfig(A4_PSC, A4_ARR);
			break;
		case 't':
			//USART_Write(USART2, &note, 1);
			xQueueSendToBackFromISR( LetterQueue, &note, NULL );
			break;
		case 'p':
			//USART_Write(USART2, &note, 1);
			xQueueSendToBackFromISR( LetterQueue, &note, NULL );
			break;
		default:
			break;
	}
}


void clockConfig(void){
	RCC->CR |= RCC_CR_HSION;								//Turn on the HSION clock for the project
	while((RCC->CR & RCC_CR_HSIRDY) == 0);

	RCC->CFGR &= ~RCC_CFGR_SW;							//Set HSION as System Clock
	RCC->CFGR |= RCC_CFGR_SW_HSI;

	//SystemCoreClockUpdate();
}

void gpioConfig(void){					
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN;			//Turn on the Register Clocks
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	GPIOC->MODER &= 0xF3FFFFFF;  //Pin C15		//Configure as input
	GPIOA->MODER &= 0xFFFFF3FF;  //Pin A5			//Configure as output
	GPIOA->MODER |= 0x400;	



}

//We need to make the interrput system so like TIM4_IQR handler to control the interrupt

void TIM4_IRQHandler(void){
	uint8_t LEDState;
	BaseType_t xStatus;

	xStatus = xQueuePeekFromISR( LEDQueue, &LEDState );
	
	
	if((TIM4->SR & TIM_SR_UIF)!=0){
		if(LEDState == 1){	
			unsigned int output = sineLookupTable[sineValue];
			sineValue += 1;
			if(sineValue == 64){
				sineValue = 0;
			}
			
			DAC->DHR12R1 = output/SCALE_FACTOR;
		}
		else{
			DAC->DHR12R1 = 0;
		}
		TIM4->SR &= ~TIM_SR_CC1IF;
		
	}

	if ((TIM4->SR & TIM_SR_UIF)!=0) {
		TIM4->SR &= ~TIM_SR_UIF;
	}
	
} //This is the code that will happen on interrupt.

void tim4Config(void){

	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM4EN;        // Enable TIM4 clock
	
	TIM4->CR1 &= ~TIM_CR1_CMS;    // Edge-aligned mode
	TIM4->CR1 &= ~TIM_CR1_DIR;    // Up-counting
	
	TIM4->CR2 &= ~TIM_CR2_MMS;    // Select master mode
	TIM4->CR2 |= TIM_CR2_MMS_2;    // 100 = OC1REF as TRGO
	
	TIM4->DIER |= TIM_DIER_TIE;    // Trigger interrupt enable
	TIM4->DIER |= TIM_DIER_UIE;    // Update interrupt enable
	
	TIM4->CCMR1 &= ~TIM_CCMR1_OC1M;
	TIM4->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2);  // 0110 = PWM mode 1
	
	TIM4->PSC = 7;            // 16 MHz / (7+1) = 2 MHz timer ticks
	TIM4->ARR = 70;        // 2 MHz / (70+1) = 28.169 kHz interrupt rate; 64 entry look-up table = 440.14 Hz sine wave
	TIM4->CCR1 = 35;        // 50% duty cycle
	TIM4->CCER |= TIM_CCER_CC1E;
	
	NVIC_EnableIRQ(TIM4_IRQn);
	NVIC_SetPriority(TIM4_IRQn, 6);
	
	TIM4->CR1 |= TIM_CR1_CEN; // Enable timer

	
	//Edge align mode bits 5:6 00
	//DIR down counter
	//URS set to 1
	//MMS set to 100 signal to trigger output
	//TIE set bit 6 to 1
	//UIE set bit 0 to a 1
	//CC1S set as 00
	//CC1NP set bit 3 to 0
	//CC1P set bit 1 to 0
	//CC1E set bit 0 to 1
	//Prescaler 2^16/28160 = at least 2 (Adjust ARR to a value to make 400Hz)
	//Load CCR with PA5 value to know when LED is on
	//CEN counter enable do this last
	

	
}

void tim4Reconfig( uint32_t psc, uint32_t arr ) {
	TIM4->CR1 &= ~TIM_CR1_CEN; // Disable timer
	TIM4->PSC = psc; // Set prescaler
	TIM4->ARR = arr; // Set auto-reload
	TIM4->CR1 |= TIM_CR1_CEN; // Enable timer
}

void dacConfig(void){					//Analog Out found on Pin A4
	
	RCC->APB1ENR1 |= RCC_APB1ENR1_DAC1EN;
	
	DAC->CR &= ~(DAC_CR_EN1);
	
	GPIOA->MODER &= ~(GPIO_MODER_MODE4);
	GPIOA->MODER |= 0X00000300;

	DAC->CR |= DAC_CR_TEN1;
	DAC->CR &= ~(DAC_CR_TSEL1);
	DAC->CR |= DAC_CR_TSEL1_0;
	DAC->CR |= DAC_CR_TSEL1_2;
	
	DAC1->MCR &= ~(DAC_MCR_MODE1_0);
	DAC1->MCR |= DAC_MCR_MODE1_1;
	DAC1->MCR &= ~(DAC_MCR_MODE1_2);

	/* Enable DAC Channel 1 */
	DAC->CR |= DAC_CR_EN1;
	
}

void led_task( void *pvParameters )
{
    /* The parameter value is expected to be 1 as 1 is passed in the
    pvParameters value in the call to xTaskCreate() below. */

		uint8_t LEDState;
		uint8_t DACOn;
		BaseType_t xStatus;
		BaseType_t xStatus2;
	
		DACOn = ( uint8_t) pvParameters;
		
    for( ;; )
    {
        /* Task code goes here. */
			xStatus = xQueuePeek( LEDQueue, &LEDState, 0 );

			if(LEDState == 1){
				GPIOA->ODR |= 1<<5;
				DACOn = 1;
				xStatus2 = xQueueOverwrite( LEDQueue2, &DACOn );
			}
			else{
				GPIOA->ODR &= ~(1<<5);
				DACOn = 0;
				xStatus2 = xQueueOverwrite( LEDQueue2, &DACOn );
			}
			
		}
}

void but_task( void *pvParameters )
{
		uint8_t LEDState;
		BaseType_t xStatus;
		BaseType_t xStatus2;

		LEDState = ( uint8_t ) pvParameters;
    for( ;; )
    {
        /* Task code goes here. */
			unsigned int idr = GPIOC->IDR;
			if(idr == 0){
				while(idr == 0){
					idr = GPIOC->IDR;
				}
				if(LEDState == 1){
					LEDState = 0;
					
				}
				else{
					LEDState = 1;
				}
			}
			xStatus = xQueueOverwrite( LEDQueue, &LEDState );
			
		}	
}

void us100_task( void *pvParameters ){
	uint8_t letter;
	BaseType_t xStatus;
	int8_t pass[30];
	int count;

	for( ;; ){
		xStatus = xQueueReceive( LetterQueue, &letter, 0 );
		if(xStatus == pdPASS){
			if(letter == 't'){
				//code of 0x50 must be sent to the sensor
				int temp;
				letter = 0x50;
				USART_Write(USART1, &letter, 1);
				USART_Read(USART1, &temp, 1);
				count = snprintf(pass, sizeof(pass), "%d deg F\n\r", ((uint8_t)(temp-45)*9/5+32));
				USART_Write(USART2, &pass, count);
			}
			else if(letter == 'p'){
				//code of 0x55 must be sent to the sensor
				int dist1;
				int dist2;
				letter = 0x55;
				USART_Write(USART1, &letter, 1);
				USART_Read(USART1, &dist1, 1);
				USART_Read(USART1, &dist2, 1);
				int distance = (256*(uint8_t)dist1+(uint8_t)dist2)*.03937;
				count = snprintf(pass, sizeof(pass), "%d inches\n\r", distance);
				USART_Write(USART2, &pass, count);
			}
		}
	}
}