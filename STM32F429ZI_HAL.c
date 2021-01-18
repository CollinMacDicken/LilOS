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
#include 	"STM32F429ZI_HAL.h"
#include 	"stm32f429xx.h"

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
/******************************************************************************
*******************************************************************************
    Prototypes
*******************************************************************************
******************************************************************************/

/******************************************************************************
*******************************************************************************
    Declarations & Types
*******************************************************************************
******************************************************************************/

/******************************************************************************
*******************************************************************************
    Helper Functions
*******************************************************************************
******************************************************************************/

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


/******************************************************************************
    OS_InitKernelHAL
		
      Prepares the system hardware for use.
******************************************************************************/
unsigned OS_InitKernelHAL(void) 
{
    OSp_InitGPIOG();
    OSp_InitGPIOA();
    OSp_InitTIM6();
    return 1; //not really much to go wrong here short of hardware errors
}
