
/********************************************
*			STM32F439 Main (C Startup File)  			*
*			Developed for the STM32								*
*			Author: Dr. Glenn Matthews						*
*			Source File														*
********************************************/

#include <stdint.h>
#include "boardSupport.h"
#include "main.h"
#include "stm32f439xx.h"
#include "gpioControl.h"

//Function Declarations ********************************************************//
void init(void);
void configureGPIOPins(void);
void configureUSART(void);
void configureADC1(void);

void count1Second(void);

void incrementClock(void);
void transmitCharacter(uint8_t character);

void triggerADCConversion(void);
int percentageHumidity(void);

void buttonCONTROL(void);
void configureTIM7(void);
void buttonCHECK(void);
void setTIM7(void);
void latchFAN(void);
void latchLIGHT(void);


// Global Variables ************************************************************//
int fanStatus = 0;
int lightStatus = 0;

int16_t buttonState = -1;

int8_t FanPressed = -1;
int8_t LightPressed = -1;
int8_t BothPressed = -1;
int8_t countVALUE = 0;



//******************************************************************************//
// Function: main()
// Input : None
// Return : None
// Description : Entry point into the application.
// *****************************************************************************//
int main(void)
{
	// Bring up the GPIO for the power regulators.
	boardSupport_init();
	
	init();
	
	count1Second();
	
  while (1)
  {
		//Always Checks Buttons
		buttonCONTROL();
		
		if((TIM6->SR & TIM_SR_UIF) == 1){
			
			//Transmit Data 
			transmitCharacter(0x21);
			transmitCharacter(percentageHumidity());
			transmitCharacter((3<<lightStatus)|(2<<fanStatus));
			// Restart timer
			TIM6->SR &= ~(TIM_SR_UIF);
			TIM6->CR1 |= TIM_CR1_CEN;
		} 
		incrementClock();
  }
} 

//******************************************************************************//
// Function: percentageHumidity()
// Input : None
// Return : None
// Description : triggers an adc conversion and converts the result into percentage.
// *****************************************************************************//
int percentageHumidity(void){
	triggerADCConversion();
	while((ADC3->SR & ADC_SR_EOC) == 0x00);
	return (ADC3->DR / 255) * 100;
}

//******************************************************************************//
// Function: triggerADCConversion()
// Input : None
// Return : None
// Description : Triggers the ADC to sample.
// *****************************************************************************//
void triggerADCConversion(void) {
	ADC3->CR2 |= ADC_CR2_SWSTART;
}

//******************************************************************************//
// Function: transmitCharacter()
// Input : None
// Return : None
// Description : Transmits a character.
// *****************************************************************************//
void transmitCharacter(uint8_t character){
	while((USART3->SR & USART_SR_TXE) == 0x00);
	USART3->DR = character;
	while((USART3->SR & USART_SR_TC) == 0x00);
}

//******************************************************************************//
// Function: count1Second()
// Input : None
// Return : None
// Description : Counts 1 second.
// *****************************************************************************//
void count1Second() {
	TIM6->CR1 &= ~TIM_CR1_CEN;
	TIM6->PSC &= ~(TIM_PSC_PSC_Msk);
	TIM6->PSC |= 4199; 
	
	TIM6->ARR &= ~(TIM_ARR_ARR_Msk);
	TIM6->ARR |= 10000;
	TIM6->CR1 |= TIM_CR1_OPM;
	TIM6->CR1 |= TIM_CR1_CEN;
}

void incrementClock(void){
	if (TIM6->CNT <= TIM6->ARR)
	{
		TIM6->CNT++;
	} else {
		TIM6->SR |= TIM_SR_UIF;
		TIM6->CNT = 0;
	}
}

//******************************************************************************//
// Function: buttonCONTROL()
// Input : None
// Return : None
// Description : Checks the status of the buttons and checks if they have been pushed for 1 second
// *****************************************************************************//
void buttonCONTROL()
{
	//Read Button Inputs (only PA8,PA9)
	buttonState = (GPIOA->IDR & 0x300); 
		
	//If Light Button Is Pressed
	if(buttonState == 256)
	{LightPressed = 1;}
	else {LightPressed = 0;}
	
	//If Fan Button Is Pressed
	if(buttonState == 512)
	{FanPressed = 1;}	
	else {FanPressed = 0;}
	
	//If Both Buttons ARE Pressed
	if(buttonState == 0)
	{BothPressed = 1;}
	else {BothPressed = 0;}
	
	buttonCHECK();
}


//******************************************************************************//
// Function: buttonCHECK()
// Input : None
// Return : None
// Description : Latches the FAN button
// *****************************************************************************//
void buttonCHECK()
{
	//Checks to see if a button has been pressed
	if((FanPressed || LightPressed || BothPressed) == 1)
	{
		//Starts 10ms timer
		setTIM7();
		//Increases countValue
		countVALUE = countVALUE + 1;
		//If the timer has expired and the countVALUE = 100 (counter occured 100 times (ie 1 second) then set outputs)
		//This means it checks every 10ms to see if the button has stopped being pressed
		if(((TIM7->SR & TIM_SR_UIF) == 1) && countVALUE == 100)
		{
			//Determines which output needs to be set
			
			//Fan needs to be set
			if(FanPressed == 1 || BothPressed == 1)
			{
				latchFAN();
			}
			
			
			//Light needs to be set
			if(LightPressed == 1 || BothPressed == 1)
			{
				latchLIGHT();
			}
			
		}
	}
	
	//Else if buttons not pressed then reset countVALUE
	if((FanPressed || LightPressed || BothPressed) == 0 || countVALUE == 100)
	{
	countVALUE = 0;
	}
}


//******************************************************************************//
// Function: latchFAN()
// Input : None
// Return : None
// Description : Latches the FAN button
// *****************************************************************************//
void latchFAN()
{
	if(fanStatus == 1) {fanStatus = 0;}
	else if(fanStatus == 0) {fanStatus = 1;}
}


//******************************************************************************//
// Function: latchLIGHT()
// Input : None
// Return : None
// Description : Latches the FAN button
// *****************************************************************************//
void latchLIGHT()
{
	if(lightStatus == 1) {lightStatus = 0;}
	else if(lightStatus == 0) {lightStatus = 1;}
}

//******************************************************************************//
// Function: setTIM7()
// Input : None
// Return : None
// Description : Resets the timer to 10ms and starts it
// *****************************************************************************//
void setTIM7()
{
	//Clear the reload register
	TIM7->ARR &= ~(TIM_ARR_ARR_Msk);
	
	//Set Count value
	TIM7->ARR |= 0x79E0; //Counts to this value to give 10ms //should be countValue
	//This is where I'm gonna change the duty cycle (<2^16-1)
	
	//Clear Status Flag
	TIM7->SR &= ~(TIM_SR_UIF);
	
	//Start Timer
	TIM7->CR1 |= TIM_CR1_CEN;	
		
}


//*********************************************************************************************************************************************//
//*********************************************************************************************************************************************//

//CONFIG SECTION//


//******************************************************************************//
// Function: configureUSART()
// Input : None
// Return : None
// Description : Configures the alternate function register and sets up Usart.
// *****************************************************************************//
void configureUSART(void) {
	//==Setup Alternate Function==================================================//
	GPIOB->AFR[1] &= ~(GPIO_AFRH_AFSEL11_Msk | GPIO_AFRH_AFSEL10_Msk);
	GPIOB->AFR[1] |= (0x07 << GPIO_AFRH_AFSEL11_Pos) | (0x07 << GPIO_AFRH_AFSEL10_Pos);
	
	USART3->CR1 &= ~(USART_CR1_OVER8);
	USART3->BRR &= 0xFFFF0000;
	
	//Set Baud Rate 9600
	USART3->BRR |= (0x16 << USART_BRR_DIV_Mantissa_Pos) | (0x4E << USART_BRR_DIV_Fraction_Pos);

	USART3->CR1 &= ~(USART_CR1_M);
	
	USART3->CR2 &= ~(USART_CR2_STOP_Msk);
	USART3->CR2 |= (0x00 << USART_CR2_STOP_Pos);
	
	USART3->CR1 &= ~(USART_CR1_PCE);
	
	USART3->CR2 &= ~(USART_CR2_CLKEN | USART_CR2_CPOL | USART_CR2_CPHA);
	USART3->CR2 &= ~(USART_CR3_CTSE | USART_CR3_RTSE);
	
	USART3->CR1 |= (USART_CR1_TE | USART_CR1_UE | USART_CR1_RE);
}

//******************************************************************************//
// Function: configureGPIOPorts()
// Input : None
// Return : None
// Description : Set up the GPIO Port Pins.
// *****************************************************************************//
void configureGPIOPins(void) {
	
	//PB0, PA10, PA9, PA8, PA3, PF10
	
	GPIO_Config portConfig;
	
	portConfig.port = GPIOB;
	portConfig.pin = Pin0;
	portConfig.mode = GPIO_Output;
	portConfig.pullUpDown = GPIO_No_Pull;
	portConfig.outputType = GPIO_Output_PushPull;
	portConfig.speed = GPIO_2MHz;
	
	// PB0
	gpio_configureGPIO(&portConfig);

	// PA10
	portConfig.port = GPIOA;
	portConfig.pin = Pin10;
	gpio_configureGPIO(&portConfig);
	
	// PA9
	portConfig.pin = Pin9;
	portConfig.mode = GPIO_Input;
	gpio_configureGPIO(&portConfig);
	
	// PA8
	portConfig.pin = Pin8;
	gpio_configureGPIO(&portConfig);

	// PA3
	portConfig.pin = Pin3;
	gpio_configureGPIO(&portConfig);
	
	// PF10
	portConfig.port = GPIOF;
	portConfig.pin = Pin10;
	portConfig.mode = GPIO_Analog;
	gpio_configureGPIO(&portConfig);
}

//******************************************************************************//
// Function: configureADC1()
// Input : None
// Return : None
// Description : Configure the ADC.
// *****************************************************************************//
void configureADC1(void){
	ADC123_COMMON->CCR &= ~(ADC_CCR_VBATE);
	ADC123_COMMON->CCR &= ~(ADC_CCR_TSVREFE);
	ADC3->CR1 &= ~((ADC_CR1_SCAN) | (0x02 << ADC_CR1_RES_Pos));
	ADC3->CR2 &= ~(ADC_CR2_CONT | ADC_CR2_ALIGN | ADC_CR2_SWSTART);
	
	ADC3->SQR3 &= ~(ADC_SQR3_SQ1_Msk);
	ADC3->SQR3 |= 0x08;
	
	ADC3->SMPR2 &= ~(ADC_SMPR2_SMP0_Msk);
	ADC3->SMPR2 |= 0x03 << (ADC_SMPR2_SMP0_Pos);
	
	ADC3->CR2 |= ADC_CR2_ADON;
}


//******************************************************************************//
// Function: configureTIM7()
// Input : None
// Return : None
// Description : Configures the timer 7 for button press checking
// *****************************************************************************//
void configureTIM7()
{
//Turning off timer
	TIM7->CR1 &= ~(TIM_CR1_CEN);
	
	//Clear Prescaler
	TIM7->PSC &= ~(TIM_PSC_PSC_Msk);
	
	//Set Prescaler to 25+1 to make a 10ms Timer
	TIM7->PSC |= 26; //make 26
	
	//Clear the reload register
	TIM7->ARR &= ~(TIM_ARR_ARR_Msk);
	
	//Set Count value
	TIM7->ARR |= 0x79E0; //Counts to this value to give 10ms
	
	//Set Timer to count be poled after each count is reached
	TIM7->CR1 |= TIM_CR1_OPM;
	
	//Start Timer
	TIM7->CR1 |= TIM_CR1_CEN;	
		
}

//******************************************************************************//
// Function: initRCC()
// Input : None
// Return : None
// Description : Configure the RCC for USART3, TIM6, ADC1 and GPIO Ports.
// *****************************************************************************//
void init(void){
	//TIM6&USART3 RCC
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN | RCC_APB1ENR_USART3EN;
	RCC->APB1RSTR |= RCC_APB1RSTR_TIM6RST | RCC_APB1RSTR_USART3RST;
	__ASM("NOP");
	__ASM("NOP");
	RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM6RST) & ~(RCC_APB1RSTR_USART3RST);
	__ASM("NOP");
	__ASM("NOP");
	
	//TIM7 RCC
	RCC->APB1RSTR |= RCC_APB1RSTR_TIM6RST;
	__asm("NOP");
	__asm("NOP");	
	RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM7RST);
	__asm("NOP");
	__asm("NOP");
	
	//GPIO RCC
	RCC->AHB1ENR = RCC->AHB1ENR | RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | 
	RCC_AHB1ENR_GPIOFEN;
	RCC->AHB1RSTR = RCC->AHB1RSTR | RCC_AHB1RSTR_GPIOARST | RCC_AHB1RSTR_GPIOBRST | 
	RCC_AHB1RSTR_GPIOFRST;
	__ASM("NOP");
	__ASM("NOP");
	RCC->AHB1RSTR = RCC->AHB1RSTR & ~(RCC_AHB1RSTR_GPIOARST) 
	& ~(RCC_AHB1RSTR_GPIOBRST) & ~(RCC_AHB1RSTR_GPIOFRST);
	__ASM("NOP");
	__ASM("NOP");
	
	//ADC RCC
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	RCC->APB2RSTR |= RCC_APB2RSTR_ADCRST;
	__ASM("NOP");
	__ASM("NOP");
	RCC->APB2RSTR &= ~(RCC_APB2RSTR_ADCRST);
	__ASM("NOP");
	__ASM("NOP");
	
	configureGPIOPins();
	configureUSART();
	configureADC1();
	configureTIM7();
	
}
