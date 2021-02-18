/******************************************************************************
*******************************************************************************
Author      : Collin MacDicken
File        : STM32F429ZI_HAL.c
Description : This code provides the Hardware Abstraction Layer (HAL) for the
              kernel.  This HAL only supports the STM32F429ZI microcontroller. 

/******************************************************************************
*******************************************************************************
    Includes
*******************************************************************************
******************************************************************************/
#include        <stdint.h>
#include 	"STM32F429ZI_HAL.h"
#include 	"stm32f429xx.h"
#include        "CommandLine.h"
/******************************************************************************
*******************************************************************************
    Definitions
*******************************************************************************
******************************************************************************/

// Optional definitions not required to produce working code
#ifndef			MAX_WAIT
	#define		BON(X)			|=(X)
	#define		BOFF(X)			&=~(X)
	#define		BTOG(X)			^=(X)
	#define		MAX_WAIT		0xFFFF
#endif		//	MAX_WAIT

#define   OS_CRITICAL_BEGIN __disable_irq()
#define   OS_CRITICAL_END   __enable_irq()
#define   ONE_SEC  16000000
#define   ONE_MS   (ONE_SEC / 1000)
#define   TIM6_IRQ 54

// The RED LED on the Discovery Adapter Board
#define   REDPIN   GPIO_ODR_OD2_Msk        
// The GREED LED on the Discovery Adapter Board
#define   GREENPIN GPIO_ODR_OD3_Msk
// The BLUE LED on the Discovery Adapter Board
#define   BLUEPIN  GPIO_ODR_OD9_Msk

#define MAX_UARTBUF_LENGTH 50

#define DEFAULT_UART_EN 1
#define DEFAULT_UART_BAUD_FRACT 7
#define DEFAULT_UART_BITMODE 0
#define DEFAULT_UART_PARITYEN 0
#define DEFAULT_UART_PARITYMODE 0
#define DEFAULT_UART_STOPMODE 1
#define DEFAULT_UART_BAUD_MANT 0x45

UartConfig uart1Config;
uint8_t uartbuf[MAX_UARTBUF_LENGTH];
uint8_t uartbuf_offset = 0;

/******************************************************************************
    OSp_InitUART
		
      Initializes PA9/10 for USART1 at 14400 baud, sets up interrupts for when
      characters are received.
******************************************************************************/
void OSp_InitUART(void)
{
  //enable USART1
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  
  //enable GPIO Port A
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
  
  //set PA9/10 to Alternate Function mode
  GPIOA->MODER = (GPIOA->MODER & ~(GPIO_MODER_MODER9_Msk | GPIO_MODER_MODER10_Msk)) |
                 (0b10 << GPIO_MODER_MODER9_Pos) | (0b10 << GPIO_MODER_MODER10_Pos);
  
  //set alternate function of PA9/10 to USART1
  GPIOA->AFR[1] = (GPIOA->AFR[1] & ~(GPIO_AFRH_AFSEL9_Msk | GPIO_AFRH_AFSEL10_Msk)) |
                  (7 << GPIO_AFRH_AFSEL9_Pos) | (7 << GPIO_AFRH_AFSEL10_Pos);


  //set baud divisor to 69.4375 (baud rate of 14401 with 16MHz fck)
  USART1->BRR = (uart1Config.baud_mant << 4) + uart1Config.baud_fract;   //14401
  
  //enable receiver and interrupts when data is enabled
  USART1->CR1 |= (uart1Config.bitMode << USART_CR1_M_Pos) | (uart1Config.parityEn << USART_CR1_PCE_Pos) | 
                 (uart1Config.parityMode << USART_CR1_PS_Pos) | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
  
  USART1->CR2 |= (uart1Config.stopMode << USART_CR2_STOP_Pos);

  //setup UART interrupts in NVIC
  NVIC_SetPriority(USART1_IRQn, 0);
  NVIC_EnableIRQ(USART1_IRQn);

  if(uart1Config.en)
  {
    //enable UART
    USART1->CR1 |= USART_CR1_UE;

    USART1->DR = '>';
  }
  else
  {
    USART1->CR1 &= ~USART_CR1_UE;
  }
}

/******************************************************************************
    OS_SendString_UART
		
      Sends the given number of characters starting from the given address 
      over USART1.
******************************************************************************/
void OS_SendString_UART(uint8_t *buf, uint8_t length)
{
  if(length)
  {
    //send each character
    while(length--)
    {
      USART1->DR = *(buf++);
      while(!(USART1->SR & USART_SR_TXE)){}
    }
  }
  else
  {
    while(*buf)
    {
      USART1->DR = *(buf++);
      while(!(USART1->SR & USART_SR_TXE)){}
    }
  }
  
  //send a newline
  USART1->DR = '\n';
  while(!(USART1->SR & USART_SR_TC)){}

  USART1->DR = '\r';
  while(!(USART1->SR & USART_SR_TC)){}
}

/******************************************************************************
    USART1_IRQHandler
		
      Interrupt handler for USART1, inserts recieved character into buffer,
      transmits the whole buffer over USART1 if a CR is received, or if the
      buffer is full.
******************************************************************************/
void USART1_IRQHandler(void)
{
  if(inCL)
  {
    //backspace key was pressed
    if(USART1->DR == 0x7F && uartbuf_offset)
    {
      USART1->DR = 0x7F;
      --uartbuf_offset;
    }
    else if(USART1->DR != 0x7F && uartbuf_offset < MAX_UARTBUF_LENGTH)
    {
      //insert character into buffer
      uartbuf[uartbuf_offset++] = USART1->DR;

      //Enter key was pressed
      if(uartbuf[uartbuf_offset - 1] == 0x0D)
      {
        OS_SendString_UART("\r", 0);
        Command_CL(uartbuf, uartbuf_offset);
        USART1->DR =  '>';
        uartbuf_offset = 0;
      }
      else
        USART1->DR =  uartbuf[uartbuf_offset - 1];
    }
  }
}

/******************************************************************************
    OSp_GetLEDPins
		
      Uses the abstracted LED values defined in Kernel HAL.h to get the
      actual pin values on the STM32f429.
******************************************************************************/
unsigned OSp_GetLEDPins(unsigned colors)
{
    unsigned int pins = 0;

    //add any pin set in colors
    if(colors & RED)
    {
        pins |= REDPIN;
    }
    if(colors & GREEN)
    {
        pins |= GREENPIN;
    }
    if(colors & BLUE)
    {
        pins |= BLUEPIN;
    }
    return pins;
}


/******************************************************************************
    OSp_InitGPIOG
		
      The clock to the PORTG module is enabled and the two pins 
    attached to the Red/Green LEDs have their digital outputs enabled. 
******************************************************************************/
void OSp_InitGPIOG(void) 
{
    volatile unsigned int wait = MAX_WAIT;	

    // Enable the system clock for the PORTG peripheral
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;

    // Wait for the system clock to stabilize
    for (wait = 0x00; wait < MAX_WAIT; ) {
      ++wait;
    } 

    // Configure PG2/3/9 as output
    GPIOG->MODER = (GPIOG->MODER & ~(GPIO_MODER_MODER2_Msk | GPIO_MODER_MODER3_Msk | GPIO_MODER_MODER9_Msk)) | 
                   ((0b01 << GPIO_MODER_MODER2_Pos) | (0b01 << GPIO_MODER_MODER3_Pos) | (0b01 << GPIO_MODER_MODER9_Pos));

    // Set the output type of PG2/3/9 to open drain
    GPIOG->OTYPER |= GPIO_OTYPER_OT2 | GPIO_OTYPER_OT3 | GPIO_OTYPER_OT9;

    // Set the speed of PG2/3/9 to medium speed
    GPIOG->OSPEEDR = (GPIOG->OSPEEDR & ~(GPIO_OSPEEDR_OSPEED2_Msk | GPIO_OSPEEDR_OSPEED3_Msk | GPIO_OSPEEDR_OSPEED9_Msk)) | 
                     ((0b01 << GPIO_OSPEEDR_OSPEED2_Pos) | (0b01 << GPIO_OSPEEDR_OSPEED3_Pos) | (0b01 << GPIO_OSPEEDR_OSPEED9_Pos));

    // Disable pull up/down on PG2/3/9
    GPIOG->PUPDR &= ~(GPIO_PUPDR_PUPD2_Msk | GPIO_PUPDR_PUPD3_Msk | GPIO_PUPDR_PUPD9_Msk);
    OS_SetLEDs(RED | GREEN | BLUE);
}


/******************************************************************************
    OSp_InitGPIOA
		
      The clock to the PORTA module is enabled and the pin
    attached to the Blue pushbutton has its digital input enabled. 
******************************************************************************/
void OSp_InitGPIOA(void) 
{
    volatile unsigned int wait = MAX_WAIT;	

    // Enable the system clock for the PORTA peripheral
     RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Wait for the system clock to stabilize
    for (wait = 0x00; wait < MAX_WAIT; ) {
      ++wait;
    } 

    // Configure PA0 as input
    GPIOA->MODER &= ~GPIO_MODER_MODER0_Msk;

    // Set the speed of PA0 to medium
    GPIOA->OSPEEDR = (GPIOA->OSPEEDR & ~GPIO_OSPEEDR_OSPEED0_Msk) | (0b01 << GPIO_OSPEEDR_OSPEED0_Pos); //Medium speed

    // Disable pull up/down on PA0
    GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD0_Msk;
}



/******************************************************************************
    OSp_InitTIM6
		
      The clock to the TIM6 module is enabled and configured to run at a 1ms
    cycle, causing an interrupt if global interrupts are enabled. 
******************************************************************************/
void OSp_InitTIM6(void) 
{
    volatile unsigned int wait = MAX_WAIT;	

    // Enable the system clock for the TIM6 peripheral
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

    // Wait for the system clock to stabilize
    for (wait = 0x00; wait < MAX_WAIT; ) {
      ++wait;
    } 

    // Enable auto-reload, only over/underflow causes interrupts
    // Ensure counter is free-running (does not stop counting), interrupts enabled
    TIM6->CR1 = (TIM6->CR1 & ~(TIM_CR1_OPM_Msk | TIM_CR1_UDIS_Msk)) | TIM_CR1_ARPE | TIM_CR1_URS;
    
    // Interrupt on over/underflow enabled
    TIM6->DIER |= TIM_DIER_UIE;

    // No prescaler used
    TIM6->PSC = 0;

    // Value for auto-reload register (sets the timer duration)
    TIM6->ARR = ONE_MS;

    // Set the timer to the lowest-numbered (best-possible) priority
    // [4] pp.208, 214, core_cm4.h in (Proj. Dir.)/CMSIS_4/CMSIS/Include
    NVIC_SetPriority(TIM6_IRQ, 0);

    // Timer is ON
    //TIM6->CR1 |= TIM_CR1_CEN;

    // Enable IRQ for TIM6 in the NVIC
    NVIC_EnableIRQ(TIM6_IRQ);
}

void OS_StartTimer(void)
{
  TIM6->CR1 |= TIM_CR1_CEN;
}

void OS_EnableIRQ(void)
{
  OS_CRITICAL_END;
}


void OS_DisableIRQ(void)
{
  OS_CRITICAL_BEGIN;
}
/******************************************************************************
    OS_SetLEDs
		
      Parameter is the bitwise OR of all the colors that should be set
    to ON.  This does not turn any LEDs OFF (requires OS_ClearLEDs).
******************************************************************************/
unsigned OS_SetLEDs(unsigned LEDs) 
{
    unsigned pins;
    pins = OSp_GetLEDPins(LEDs);
    if(pins == 0)
      return 0;

    OS_CRITICAL_BEGIN;
    GPIOG->ODR &= ~pins;
    OS_CRITICAL_END;
    return 1;
}


/******************************************************************************
    OS_ClearLEDs
		
      Parameter is the bitwise OR of all the colors that should be set
    to OFF.  This does not turn any LEDs ON (requires OS_SetLEDs).
******************************************************************************/
unsigned OS_ClearLEDs(unsigned LEDs) 
{
    unsigned pins;
    pins = OSp_GetLEDPins(LEDs);
    if(pins == 0)
      return 0;

    OS_CRITICAL_BEGIN;
    GPIOG->ODR |= pins;
    OS_CRITICAL_END;
    return 1;
}

/******************************************************************************
    OS_GetButton
		
      Returns nonzero if the button is pushed, otherwise returns zero. 
******************************************************************************/
unsigned OS_GetButton(void) 
{
    //return value of PA0
    return (GPIOA->IDR & GPIO_IDR_ID0_Msk) >> GPIO_IDR_ID0_Pos;
}


void OSp_InitConfig(void)
{
  uart1Config.en = DEFAULT_UART_EN;
  uart1Config.baud_fract = DEFAULT_UART_BAUD_FRACT;
  uart1Config.baud_mant = DEFAULT_UART_BAUD_MANT;
  uart1Config.bitMode = DEFAULT_UART_BITMODE;
  uart1Config.parityEn = DEFAULT_UART_PARITYEN;
  uart1Config.parityMode = DEFAULT_UART_PARITYMODE;
  uart1Config.stopMode = DEFAULT_UART_STOPMODE;
}


/******************************************************************************
    OS_InitKernelHAL
		
      Prepares the system hardware for use.
******************************************************************************/
unsigned OS_InitKernelHAL(void) 
{
  OSp_InitConfig();
  OSp_InitGPIOG();
  OSp_InitGPIOA();
  OSp_InitTIM6();
  OSp_InitUART();
  while(inCL){}
  OSp_InitUART();
  return 1; //not really much to go wrong here short of hardware errors
}
