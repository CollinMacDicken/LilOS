/******************************************************************************
*******************************************************************************
Author      : Collin MacDicken
File        : STM32F429ZI_HAL.h
Description : Header for STM32F429ZI HAL

*******************************************************************************
******************************************************************************/

#ifndef		__KERNEL_HAL__H__
#define		__KERNEL_HAL__H__

#include <stdint.h>

// The RED LED on the Discovery Adapter Board
#define   RED   0b001     
// The GREED LED on the Discovery Adapter Board
#define   GREEN 0b010
// The BLUE LED on the Discovery Adapter Board
#define   BLUE  0b100                   

typedef struct {
  uint8_t en;
  uint8_t baud_fract;
  uint8_t bitMode;
  uint8_t parityEn;
  uint8_t parityMode;
  uint8_t stopMode;
  uint16_t baud_mant;


} UartConfig;

extern UartConfig uart1Config;

/******************************************************************************
    OS_EnableIRQ
		
      Enables interrupts systemwide
******************************************************************************/
void OS_EnableIRQ(void);


/******************************************************************************
    OS_DisableIRQ
		
      Disables interrupts systemwide
******************************************************************************/
void OS_DisableIRQ(void);


/******************************************************************************
    OS_SetLEDs
		
      Parameter is the bitwise OR of all the colors that should be set
    to ON.  This does not turn any LEDs OFF (requires OS_ClearLEDs).
******************************************************************************/
unsigned OS_SetLEDs(unsigned);


/******************************************************************************
    OS_ClearLEDs
		
      Parameter is the bitwise OR of all the colors that should be set
    to OFF.  This does not turn any LEDs ON (requires OS_SetLEDs).
******************************************************************************/
unsigned OS_ClearLEDs(unsigned);


/******************************************************************************
    OS_GetButton
		
      Returns nonzero if the button is pushed, otherwise returns zero. 
******************************************************************************/
unsigned OS_GetButton(void);


/******************************************************************************
    OS_InitKernelHAL
		
      Prepares the system hardware for use.
******************************************************************************/
unsigned OS_InitKernelHAL(void);


/******************************************************************************
    OS_StartTimer
		
      Enables the system timer to start the OS
******************************************************************************/
void OS_StartTimer(void);

/******************************************************************************
    OS_SendString_UART
		
      Sends the given number of characters starting from the given address 
      over USART1.
******************************************************************************/
void OS_SendString_UART(uint8_t *buf, uint8_t length);

#endif	//	__KERNEL_HAL__H__
