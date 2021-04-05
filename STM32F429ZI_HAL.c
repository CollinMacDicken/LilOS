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

/*
SPI Pins
PB12    NSS
PB13    SCK
PB14    MISO
PB15    MOSI
*/


/*
SPI Pins
PE15    NSS
PE12    SCK
PE13    MISO
PE14    MOSI
*/


// Optional definitions not required to produce working code
#ifndef			MAX_WAIT
	#define		BON(X)			|=(X)
	#define		BOFF(X)			&=~(X)
	#define		BTOG(X)			^=(X)
	#define		MAX_WAIT		0xFFFF
#endif		//	MAX_WAIT

#define   OS_CRITICAL_BEGIN __disable_irq()
#define   OS_CRITICAL_END   __enable_irq()
#define   TIM6_IRQ 54

#define SD_AF 5

#define TIMER_PSC 1600

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

#define DEFAULT_GPIO_EN 0
#define DEFAULT_GPIO_DIR 1   //(0: input, 1: output)
#define DEFAULT_GPIO_OTYPE 0 //(0: push-pull, 1: OD)
#define DEFAULT_GPIO_PULL 0  //(0: none, 1: PU, 2: PD)

UartConfig uart1Config;
GpioConfig gpioCConfig;
uint8_t uartbuf[MAX_UARTBUF_LENGTH];
uint8_t uartbuf_offset = 0;
uint16_t userVar = 0;
uint16_t timeout = 1000;

void OS_SendString_UART(uint8_t *buf, uint8_t length);

/******************************************************************************
*******************************************************************************
                                 TASK FUNCTIONS
*******************************************************************************
******************************************************************************/
void OS_SetGPIO(uint16_t val)
{
  switch(val >> 8)
  {
    case GPIO_A:
      GPIOA->BSRR = 1 << (val & 0xFF);
      break;
    
    case GPIO_B:
      GPIOB->BSRR = 1 << (val & 0xFF);
      break;

    case GPIO_C:
      GPIOC->BSRR = 1 << (val & 0xFF);
      break;
    
    case GPIO_D:
      GPIOD->BSRR = 1 << (val & 0xFF);
      break;
    
    case GPIO_E:
      GPIOE->BSRR = 1 << (val & 0xFF);
      break;
    
    case GPIO_F:
      GPIOF->BSRR = 1 << (val & 0xFF);
      break;
    
    case GPIO_G:
      GPIOG->BSRR = 1 << (val & 0xFF);
      break;
    
    case GPIO_H:
      GPIOH->BSRR = 1 << (val & 0xFF);
      break;

    case GPIO_I:
      GPIOI->BSRR = 1 << (val & 0xFF);
      break;
    
    case GPIO_J:
      GPIOJ->BSRR = 1 << (val & 0xFF);
      break;

    case GPIO_K:
      GPIOK->BSRR = 1 << (val & 0xFF);
      break;
  }
}

void OS_ClearGPIO(uint16_t val)
{
  val += NUM_GPIOS;
  switch(val >> 8)
  {
    case GPIO_A:
      GPIOA->BSRR = 1 << (val & 0xFF);
      break;
    
    case GPIO_B:
      GPIOB->BSRR = 1 << (val & 0xFF);
      break;

    case GPIO_C:
      GPIOC->BSRR = 1 << (val & 0xFF);
      break;
    
    case GPIO_D:
      GPIOD->BSRR = 1 << (val & 0xFF);
      break;
    
    case GPIO_E:
      GPIOE->BSRR = 1 << (val & 0xFF);
      break;
    
    case GPIO_F:
      GPIOF->BSRR = 1 << (val & 0xFF);
      break;
    
    case GPIO_G:
      GPIOG->BSRR = 1 << (val & 0xFF);
      break;
    
    case GPIO_H:
      GPIOH->BSRR = 1 << (val & 0xFF);
      break;

    case GPIO_I:
      GPIOI->BSRR = 1 << (val & 0xFF);
      break;
    
    case GPIO_J:
      GPIOJ->BSRR = 1 << (val & 0xFF);
      break;

    case GPIO_K:
      GPIOK->BSRR = 1 << (val & 0xFF);
      break;
  }
}

void OS_SetVar(uint16_t val)
{
  userVar = val;
}

uint16_t OS_GetVar()
{
  return userVar;
}

void OS_SsHighSPI(void)
{
  GPIOE->ODR |= 1 << 15;
  //GPIOE->ODR &= ~(1 << 15);
}

void OS_SsLowSPI(void)
{
  //GPIOE->ODR |= 1 << 15;
  GPIOE->ODR &= ~(1 << 15);
}

/******************************************************************************
    OSp_InitSPI
		
      Initializes PA9/10 for USART1 at 14400 baud, sets up interrupts for when
      characters are received.
******************************************************************************/
void OSp_InitSPI(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_SPI4EN;

  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;

  GPIOE->MODER = (GPIOE->MODER & ~(GPIO_MODER_MODER15_Msk | GPIO_MODER_MODER12_Msk | GPIO_MODER_MODER13_Msk | GPIO_MODER_MODER14_Msk)) |
                 (0b01 << GPIO_MODER_MODER15_Pos) | (0b10 << GPIO_MODER_MODER12_Pos) | (0b10 << GPIO_MODER_MODER13_Pos) | (0b10 << GPIO_MODER_MODER14_Pos);

  GPIOE->AFR[1] = (GPIOE->AFR[1] & ~(GPIO_AFRH_AFSEL12_Msk | GPIO_AFRH_AFSEL13_Msk | GPIO_AFRH_AFSEL14_Msk)) |
                  (SD_AF << GPIO_AFRH_AFSEL12_Pos) | (SD_AF << GPIO_AFRH_AFSEL13_Pos) | (SD_AF << GPIO_AFRH_AFSEL14_Pos);
  

  GPIOE->OSPEEDR |= 0x2A800000;

  SPI4->CR2 |= SPI_CR2_SSOE;

  SPI4->CR1 |= SPI_CR1_MSTR | SPI_CR1_SSM | 0b101000 | SPI_CR1_SPE;

  //OS_SetTimer(1000);
  //while(!OS_TimerDone()){}
  //SPI4->CR1 |= SPI_CR1_SPE;

  OS_SsHighSPI();
}

void OS_WriteSPI(uint8_t *txBuf, uint16_t txLength)
{
  uint16_t count = 0;
  uint8_t dummy;
  while(count < txLength)
  {
    SPI4->DR = txBuf[count++];
    while(!(SPI4->SR & SPI_SR_TXE)){}
    dummy = SPI4->DR;
  }
}

void OS_ReadSPI(uint8_t *rxBuf, uint16_t rxLength)
{
  uint16_t count = 0;
  uint8_t dummy;
  while(count < rxLength)
  {
    SPI4->DR = 0xFF;
    while(!(SPI4->SR & SPI_SR_TXE)){}
    if(rxBuf)
      rxBuf[count] = SPI4->DR;
    else
      dummy = SPI4->DR;
    ++count;
  }

}

void OS_CommSPI(uint8_t *txBuf, uint8_t *rxBuf, uint16_t length)
{
  uint16_t count = 0;
  SPI4->CR1 |= SPI_CR1_SPE;
  while(count < length)
  {
    SPI4->DR = txBuf[count];
    while(!(SPI4->SR & SPI_SR_TXE)){}
    rxBuf[count++] = SPI4->DR;
  }
}

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


void OSp_InitGPIOC(void)
{
  uint16_t wait;
  uint8_t i;

  RCC->AHB1ENR |= gpioCConfig.en << RCC_AHB1ENR_GPIOCEN_Pos;
  for (wait = 0x00; wait < MAX_WAIT; ++wait){}
  
  GPIOC->MODER = 0x00000000;
  GPIOC->OTYPER &= 0xFFFF0000;
  GPIOC->PUPDR = 0x00000000;
  for(i = 0; i < NUM_GPIOS; ++i)
  {
    GPIOC->MODER |= (gpioCConfig.dir)[i] << (i*2);
    GPIOC->OTYPER |= (gpioCConfig.oType)[i] << i;
    GPIOC->PUPDR |= (gpioCConfig.pull)[i] << (i*2);
  }


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
void OSp_InitTIM6(uint16_t timeval) 
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
    TIM6->ARR = timeval;

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

void OS_SetTimer(uint16_t hundred_us)
{
  RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
  TIM7->PSC = TIMER_PSC;
  
  TIM7->DIER &= ~TIM_DIER_UIE;

  TIM7->CNT = 0;

  TIM7->ARR = hundred_us - 1;

  TIM7->CR1 |= 1;
}

uint8_t OS_TimerDone()
{
  if(TIM7->SR & TIM_SR_UIF)
  {
    TIM7->CR1 &= ~TIM_CR1_CEN;
    TIM7->SR &= ~TIM_SR_UIF;
    return 1;
  }

  return 0;
}

void OS_Wait_ms(uint16_t ms)
{
  OS_SetTimer(ms*10);
  while(!OS_TimerDone()){};
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

/******************************************************************************
    OSp_InitConfig
		
      Fills configuration structs with default values.
******************************************************************************/
void OSp_InitConfig(void)
{
  uint8_t i;

  uart1Config.en = DEFAULT_UART_EN;
  uart1Config.baud_fract = DEFAULT_UART_BAUD_FRACT;
  uart1Config.baud_mant = DEFAULT_UART_BAUD_MANT;
  uart1Config.bitMode = DEFAULT_UART_BITMODE;
  uart1Config.parityEn = DEFAULT_UART_PARITYEN;
  uart1Config.parityMode = DEFAULT_UART_PARITYMODE;
  uart1Config.stopMode = DEFAULT_UART_STOPMODE;
  
  gpioCConfig.en = DEFAULT_GPIO_EN;
  for(i = 0; i < NUM_GPIOS; ++i)
  {
    (gpioCConfig.dir)[i] = DEFAULT_GPIO_DIR;
    (gpioCConfig.oType)[i] = DEFAULT_GPIO_OTYPE;
    (gpioCConfig.pull)[i] = DEFAULT_GPIO_PULL;
  }
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
  OSp_InitUART();
  //OSp_InitSPI();
  //inCL = 0;
  while(inCL){}
  OSp_InitUART();
  OSp_InitGPIOC();
  OSp_InitTIM6(timeout * ONE_US);
  return 1; //not really much to go wrong here short of hardware errors
}
