/******************************************************************************
*******************************************************************************
Author      : Collin MacDicken
File        : LilOS.h
Description : Kernel header that user tasks will interact with.

*******************************************************************************
******************************************************************************/

#ifndef		__KERNEL__H__
#define		__KERNEL__H__

/******************************************************************************
*******************************************************************************
    Includes
*******************************************************************************
******************************************************************************/
#include  "STM32F429ZI_HAL.h"
#include  "STM32F429ZI_HAL_asm.h"
/******************************************************************************
*******************************************************************************
    Definitions
*******************************************************************************
******************************************************************************/
#define   OS_MAX_STACK_SIZE   0x40
#define   OS_MIN_STACK_SIZE   17
#define   OS_MAX_TASKS        5
#define   OS_MAX_SEMS         10
#define   OS_STACK_MARKER     0xDEADBEEF
#define   OS_HEAP_SIZE        512
#define   OS_HEAP_BLOCK_ALIGN 2
#define   OS_MAX_STEPS_PER_TASK 10
#define   OS_MAX_BITASKS        5
/******************************************************************************
*******************************************************************************
    Public Prototypes
*******************************************************************************
******************************************************************************/
typedef enum {NO_ERROR, UNDEFINED_ERROR, ALLOC_ERROR, FREE_ERROR, TASK_STATE_MISMATCH,
             INVALID_SEM_ERROR, MAX_SEM_ERROR, SEM_ACQ_ERROR} kernelErrors;
typedef enum {MUTEX, COUNTING} kernelObjects;

typedef enum
{
  SCHED_EDF,
  SCHED_RR,
  SCHED_NUM
} schedule_t;

typedef struct
{
  char en;
  char numsteps;
  char funcString[300];
  unsigned priority;
  unsigned runtime;
  unsigned deadline;
} BI_TaskStruct;

extern schedule_t OS_Scheduler;
extern uint16_t OS_sched_timeout;
extern BI_TaskStruct BI_tasks[OS_MAX_BITASKS];

void OS_InitBITasks();

/******************************************************************************
    OS_InitKernel
		
      Prepares the Kernel for use, but does not start any services.  No OS_
    function should be called until after this one has executed.
******************************************************************************/    
unsigned OS_InitKernel(unsigned numTasks, unsigned stackSize);

/******************************************************************************
    OS_CreateTask
		
      Takes the assigned function pointer and uses it to create a kernel task
    that is ready for execution.
******************************************************************************/
unsigned OS_CreateTask(char *funcString, unsigned priority, unsigned runtime, unsigned deadline);


/******************************************************************************
    OS_SemCreate
		
      Creates a semaphore with the passed values.
******************************************************************************/
unsigned OS_SemCreate(unsigned type, unsigned startTokens, unsigned maxTokens);


/******************************************************************************
    OS_SemAcquire
		
      Acquires the semaphore with the given ID, this blocks until the semaphore
    is available.
******************************************************************************/
unsigned OS_SemAcquire(unsigned ID);

/******************************************************************************
    OS_SemRelease
		
     Releases the semaphore with the given ID
******************************************************************************/
unsigned OS_SemRelease(unsigned ID);


/******************************************************************************
    OS_Malloc
		
      Selects a fitting memory block for the given blockSize, marks that block
     as in use and returns a pointer to the beginning of it.
******************************************************************************/
void *OS_Malloc(unsigned blockSize);



/******************************************************************************
    OS_Free
		
      Marks the memory block pointer to by the given pointer as free and
    joins memory blocks if able.
******************************************************************************/
unsigned OS_Free(void *ptr);


/******************************************************************************
    OS_GetError
		
      Returns currently reported kernel error.
******************************************************************************/
kernelErrors OS_GetError(void);

/******************************************************************************
    OS_Start
		
      Begins kernel services and starts execution of the highest priority task.
******************************************************************************/
extern void OS_Start(void);

#endif	//	__KERNEL__H__
