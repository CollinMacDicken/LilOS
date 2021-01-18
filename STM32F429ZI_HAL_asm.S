/******************************************************************************
******************************************************************************
Name        : Collin MacDicken
File        : Kernel HAL_asm.s
Description : Assembly HAL for STM32F429ZI, supports context switching.

*****************************************************************************
****************************************************************************/
   #include "STM32F429ZI_HAL_asm.h"
  .syntax unified                   // Required to execute Thumb-2 code
  .global OS_Start                  // Required to expose function to .c files
  //.global TIM6_DAC_IRQHandler
  .extern OS_TaskRUNNING            // Defined in Kernel.c
 
  .section .text                    // Where to locate this code
  .align 0

  .thumb_func
OS_Start:
    //Code from Valvano text [5]
    LDR R0, =OS_TaskRUNNING //currently running thread
    LDR R1, [R0]            //R1 = value of OS_TaskRUNNING
    LDR SP, [R1]            //new thread SP; SP = OS_TaskRUNNING->sp
    POP {R4-R11}            //restore regs R4-11
    POP {R0-R3}             //restore regs R0-3
    POP {R12}
    ADD SP, SP, #4          //discard LR from initial stack
    POP {LR}                //start location
    ADD SP, SP, #4          //discard PSR
    CPSIE I                 //enable intrrupts at processor level
    PUSH {LR}                      
    BL OS_StartTimer
    POP {LR}
    BX LR                   //start first thread
    //End code from valvano text

  .weak TIM6_DAC_IRQHandler
  .thumb_func
TIM6_DAC_IRQHandler:
    CPSID I                        //disable global interrupts
    PUSH {R4-R11}                  //save values of R4-R11
    LDR R0, =TIMER6_SR             //Clear IF bit of TIM6->SR
    LDR R1, [R0]                   
    BIC R1, R1, TIMER_IF
    STR R1, [R0]                   
    ClearLoop:                     //Loop until IF bit in TIM6->SR is cleared
      LDR R1, [R0]
      CMP R1, TIMER_IF
      BEQ ClearLoop
    
    //Code fromm Valvano text [5]
    LDR R0, =OS_TaskRUNNING        //OS_TaskRUNNING->sp = SP
    LDR R1, [R0]                   
    STR SP, [R1]            
    //end code from valvano text

    PUSH {LR}                      
    BL OSp_ScheduleTask
    POP {LR}
    CMP R0, #1                     //check return value and skip context switch if its not necessary
    BNE NoSwitch
    LDR R0, =OS_TaskRUNNING
    LDR R1, [R0]
    LDR SP, [R1]
    
    NoSwitch:
    POP {R4-R11}
    CPSIE I                        //enable global interrupts
    BX LR


// EOF    Kernel HAL_asm.s
// Note: Some IDEs generate warnings if a file doesn't end in whitespace,
//  but Embedded Studio doesn't seem to be one of them.



