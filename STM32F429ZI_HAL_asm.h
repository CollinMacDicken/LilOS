/******************************************************************************
*******************************************************************************
Author      : Collin MacDicken
File        : STM32F429ZI_HAL_asm.h
Description : Contains gcc symbol definitions that must be shared between both
              the assembly and C source files.
              STM32F429ZI_HAL
              
******************************************************************************/

/******************************************************************************
*******************************************************************************
    Definitions
*******************************************************************************
******************************************************************************/

#define   OS_READY            0x00 // task is ready to be run
#define   OS_RUNNING          0x01 // task is actively running
#define   OS_SATISFIED        0x02 // task has been completed before current deadline
#define   OS_BLOCKED          0x03 // task is waiting for a resource to become available

#define   BYTES_PER_WORD      4    //32 bit CPU
#define   TCB_STATE_OFFSET    (BYTES_PER_WORD*2)
#define   TIMER6_SR           0x40001010
#define   TIMER_IF            1
