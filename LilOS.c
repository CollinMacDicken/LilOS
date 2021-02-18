/******************************************************************************
*******************************************************************************
Name        : Collin MacDicken
File        : LilOS.c
Description : Contains implementation of RTOS functions.

/******************************************************************************
*******************************************************************************
    Includes
*******************************************************************************
******************************************************************************/
#include "LilOS.h"
#include <stdlib.h>

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

#define PARENT_INDEX(i)      ((i%2)?(i-1)/2:(i/2)-1)
#define LEFT_CHILD_INDEX(i)  ((i*2)+1)
#define RIGHT_CHILD_INDEX(i) ((i+1)*2)
/******************************************************************************
*******************************************************************************
    Declarations & Types
*******************************************************************************
******************************************************************************/
typedef	void (* OS_TaskAddress)(void);	

typedef enum {READY = OS_READY, RUNNING = OS_RUNNING, SATISFIED = OS_SATISFIED, BLOCKED = OS_BLOCKED} taskState_t;

//task Control Block
typedef struct 
{
  unsigned        *sp;          //task stack pointer
  unsigned        taskID;       
  unsigned        taskPriority;
  unsigned        taskRuntime;  //expected number of clock ticks to complete task
  unsigned        taskDeadline; //periodic deadline
  unsigned        taskTime;     //number of clock ticks that the task has been run so far
  taskState_t     taskState;    
  OS_TaskAddress  taskAddress;
} TCB;

//task node for task lists
typedef struct
{
  TCB  tcb;
  void *next;
} taskNode;

//memory node to keep track of memory allocations
typedef struct
{
  unsigned size;
  unsigned allocTableIndex;
} memNode;

//Semaphore Control Block
typedef struct
{
  unsigned ID;
  unsigned ownerList[OS_MAX_TASKS];
  unsigned type;
  unsigned tokens;
  unsigned maxTokens;
} SCB;

// Globals used by the system
kernelErrors    currentError = NO_ERROR;
unsigned char 	newTaskID = 0;
unsigned char   newSemID = 1;
unsigned        OS_numTasks;
unsigned        OS_stackSize;
unsigned        OS_systemTick;
unsigned        OS_blockSize;
unsigned        heap[OS_HEAP_SIZE];
unsigned        stacks[OS_MAX_TASKS][OS_MAX_STACK_SIZE];
unsigned        numFreeBlocks = 0;
int             allocTable[OS_HEAP_SIZE / OS_HEAP_BLOCK_ALIGN]; //Maps memory based on location
taskNode        *taskRunningHead = 0;                           //linked list of running tasks
taskNode        *taskReadyHead = 0;                             //linked list of ready tasks
taskNode        *taskBlockedHead = 0;                           //linked list of blocked tasks
taskNode        *taskSatisfiedHead = 0;                         //linked list of satisfied tasks
taskNode        tasks[OS_MAX_TASKS];                            //array where task data is stored
memNode         freeBlocks[OS_HEAP_SIZE / OS_HEAP_BLOCK_ALIGN]; //Maps memory based on size
SCB             semaphores[OS_MAX_SEMS];
TCB             *OS_TaskRUNNING;

/******************************************************************************
*******************************************************************************
    Prototypes
*******************************************************************************
******************************************************************************/
void OSp_IdleTask(void);
void OSp_UpdateSatisfiedList(void);
void OSp_UpdateBlockedList(unsigned semID);
char OSp_ScheduleTask(void);
void OSp_SetError(kernelErrors error);
taskNode *OSp_CompareNodePrios(taskNode *A, taskNode *B);
void OSp_InsertNode(taskState_t state, taskNode *node);
void OSp_RemoveNode(taskState_t state, taskNode *node);
void OSp_ChangeTaskState(taskState_t currentState, taskState_t newState, taskNode *node);
unsigned OSp_AllocateInit(unsigned blockSize);
void OSp_InsertFreeHeap(unsigned size, unsigned allocIndex);
void OSp_RemoveFreeHeap(unsigned size, unsigned allocIndex);
void OSp_SwapMemNodes(memNode *A, memNode *B);
/******************************************************************************
*******************************************************************************
    Helper Functions
*******************************************************************************
******************************************************************************/

/******************************************************************************
    OS_InitKernel
		
      Prepares the Kernel for use, but does not start any services.  No OS_
    function should be called until after this one has executed.
******************************************************************************/    
unsigned OS_InitKernel(const unsigned numTasks, const unsigned stackSize) 
{
    //set error state if passed parameters go beyond hard OS bounds
    if(numTasks > OS_MAX_TASKS || stackSize < OS_MIN_STACK_SIZE || stackSize > OS_MAX_STACK_SIZE)
    {
      OSp_SetError(ALLOC_ERROR);
      return 0;
    }
    
    //set OS parameters
    OS_numTasks = numTasks;
    OS_stackSize = stackSize;
    OS_systemTick = 0;
    
    //Create Idle task and set it to running state
    if(OS_CreateTask(OSp_IdleTask, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF) == 0)
      return 0;
    OSp_ChangeTaskState(READY, RUNNING, taskReadyHead);
    
    if(OSp_AllocateInit(2) == 0)
      return 0;

    //set up hardware
    return 1;//OS_InitKernelHAL();
}

/******************************************************************************
    OS_TaskCreate
		
      Takes the assigned function pointer and uses it to create a kernel task
    that is ready for execution.
******************************************************************************/
unsigned OS_CreateTask(void (* newTask)(void), unsigned priority, unsigned runtime, unsigned deadline) 
{
    //set error state if max tasks supported by the OS has been exceeded
    if(newTaskID > OS_numTasks || newTask == 0)
    {
      OSp_SetError(ALLOC_ERROR);
      return 0;
    }

    //set up TCB
    tasks[newTaskID].tcb.taskAddress = newTask;
    tasks[newTaskID].tcb.taskID = newTaskID;
    tasks[newTaskID].tcb.taskState = READY;
    tasks[newTaskID].tcb.taskPriority = priority;
    tasks[newTaskID].tcb.taskRuntime = runtime;
    tasks[newTaskID].tcb.taskDeadline = deadline;
    tasks[newTaskID].tcb.taskTime = 0;
    
    //set up Stack
    stacks[newTaskID][OS_stackSize - 1] = OS_STACK_MARKER;     //Border Marker
    stacks[newTaskID][OS_stackSize - 3] = (unsigned) newTask;  //Link Register

    //Code from Valvano text
    stacks[newTaskID][OS_stackSize - 2] = 0x01000000;          //Thumb bit
    stacks[newTaskID][OS_stackSize - 4] = 0x14141414;          //R14
    stacks[newTaskID][OS_stackSize - 5] = 0x12121212;          //R12
    stacks[newTaskID][OS_stackSize - 6] = 0x03030303;          //R3
    stacks[newTaskID][OS_stackSize - 7] = 0x02020202;          //R2
    stacks[newTaskID][OS_stackSize - 8] = 0x01010101;          //R1
    stacks[newTaskID][OS_stackSize - 9] = 0x00000000;          //R0
    stacks[newTaskID][OS_stackSize - 10] = 0x11111111;         //R11
    stacks[newTaskID][OS_stackSize - 11] = 0x10101010;         //R10
    stacks[newTaskID][OS_stackSize - 12] = 0x09090909;         //R9
    stacks[newTaskID][OS_stackSize - 13] = 0x08080808;         //R8
    stacks[newTaskID][OS_stackSize - 14] = 0x07070707;         //R7
    stacks[newTaskID][OS_stackSize - 15] = 0x06060606;         //R6
    stacks[newTaskID][OS_stackSize - 16] = 0x05050505;         //R5
    stacks[newTaskID][OS_stackSize - 17] = 0x04040404;         //R4
    //end code from valvano text
  
    //set TCB stack pointer to top of valid stack data
    tasks[newTaskID].tcb.sp = &(stacks[newTaskID][OS_stackSize - 17]);
    
    OSp_InsertNode(READY, &(tasks[newTaskID]));
    ++newTaskID;
    return 1;
}

/******************************************************************************
    OS_SemCreate
		
      Creates a semaphore with the passed values.
      ## StartTokens should equal maxTokens right??? ##
******************************************************************************/
unsigned OS_SemCreate(unsigned type, unsigned startTokens, unsigned maxTokens)
{   unsigned numOwners = (type == MUTEX)? 1 : maxTokens;
    unsigned i;
    
    //set error state if invalid semaphore parameters are given
    if((type != MUTEX && type != COUNTING) || maxTokens == 0 || startTokens > maxTokens)
    {
       OSp_SetError(INVALID_SEM_ERROR);
        return 0;
    }

    //set error state if max number of semaphores supported by the OS has been exceeded
    if(newSemID > OS_MAX_SEMS)
    {
        OSp_SetError(MAX_SEM_ERROR);
        return 0;
    }
    
    //set up SCB
    for(i = 0; i < OS_MAX_TASKS; ++i)
      semaphores[newSemID-1].ownerList[i] = 0;
    semaphores[newSemID-1].ID = newSemID;        
    semaphores[newSemID-1].type = type;
    semaphores[newSemID-1].tokens = startTokens;
    semaphores[newSemID-1].maxTokens = maxTokens;

    return newSemID++;
}

/******************************************************************************
    OS_SemAcquire
		
      Acquires the semaphore with the given ID, this blocks until the semaphore
    is available.
******************************************************************************/
unsigned OS_SemAcquire(unsigned ID)
{
    //return an error state if an invalid semaphore was given
    if(ID >= newSemID)
    {
        OSp_SetError(INVALID_SEM_ERROR);
        return 0;
    }

    if(semaphores[ID-1].type == MUTEX)
    {
        OS_DisableIRQ();

        //if the mutex is owned by the currently running task, or is unowned,
        if(semaphores[ID-1].ownerList[OS_TaskRUNNING->taskID] || semaphores[ID-1].tokens == semaphores[ID-1].maxTokens)
        {
            //if there are still tokens remaining in the mutex
            if(semaphores[ID-1].tokens > 0)
            {
                //take a token from the mutex and give it to the currently running task
                ++(semaphores[ID-1].ownerList[OS_TaskRUNNING->taskID]);
                --(semaphores[ID-1].tokens);
                OS_EnableIRQ();
                return 1;
            }
            //if the mutex is owned by this task, but there are no tokens remaing, set an error state
            else
            {
                OS_EnableIRQ();
                OSp_SetError(SEM_ACQ_ERROR);
                return 0;
            }
        }

        //if the mutex is owned by another task, block until it becomes available
        else
        {
            OS_TaskRUNNING->taskState = BLOCKED;
            semaphores[ID-1].ownerList[OS_TaskRUNNING->taskID] = semaphores[ID-1].maxTokens+1;
            OS_EnableIRQ();
            while(semaphores[ID-1].tokens != semaphores[ID-1].maxTokens){}
            OS_DisableIRQ();
            semaphores[ID-1].ownerList[OS_TaskRUNNING->taskID] = 1;
            --(semaphores[ID-1].tokens);
            OS_EnableIRQ();
            return 1;
        }
    }
    else if(semaphores[ID-1].type == COUNTING)
    {
        OS_DisableIRQ();

        //if the counting semaphore still has tokens, take one
        if(semaphores[ID-1].tokens > 0)
        {
            --(semaphores[ID-1].tokens);
            ++(semaphores[ID-1].ownerList[OS_TaskRUNNING->taskID]);
            OS_EnableIRQ();
            return 1;
        }

        //if the counting semaphore is unavailable, wait until it is
        else
        {
            OS_TaskRUNNING->taskState = BLOCKED;
            semaphores[ID-1].ownerList[OS_TaskRUNNING->taskID] = (semaphores[ID-1].ownerList[OS_TaskRUNNING->taskID] + 2) * semaphores[ID-1].maxTokens;
            OS_EnableIRQ();
            while(semaphores[ID-1].tokens == 0){}
            OS_DisableIRQ();
             --(semaphores[ID-1].tokens);
             semaphores[ID-1].ownerList[OS_TaskRUNNING->taskID] = (semaphores[ID-1].ownerList[OS_TaskRUNNING->taskID] / semaphores[ID-1].maxTokens) - 2;
            OS_EnableIRQ();
            return 1;
        }
    }
    
    //this semaphore has an invalid type
    OSp_SetError(INVALID_SEM_ERROR);
    return 0;
}

/******************************************************************************
    OS_SemRelease
		
     Releases the semaphore with the given ID
******************************************************************************/
unsigned OS_SemRelease(unsigned ID)
{
  //set an error state if an invalid semaphore was given
  if(ID >= newSemID)
  {
    OSp_SetError(INVALID_SEM_ERROR);
    return 0;
  }
  
  if(semaphores[ID-1].type == MUTEX || semaphores[ID-1].type == COUNTING)
  {
    OS_DisableIRQ();
    
    //if the currently running task owns a token for this semaphore, give 1 back
    if(semaphores[ID-1].ownerList[OS_TaskRUNNING->taskID])
    {
      ++(semaphores[ID-1].tokens);
      --(semaphores[ID-1].ownerList[OS_TaskRUNNING->taskID]);

      //if this is a counting semaphore, or the final token was given back to a mutex, unblock any tasks blocked by this semaphore
      if(semaphores[ID-1].type == COUNTING || semaphores[ID-1].ownerList[OS_TaskRUNNING->taskID] == 0)
        OSp_UpdateBlockedList(ID);

      OS_EnableIRQ();
      return 1;
    }

    //if currently running task doesn't own this semaphore, set an error state
    else
    {
      OS_EnableIRQ();
      OSp_SetError(SEM_ACQ_ERROR);
      return 0;
    }
  }
  
  //this semaphore has an invalid type
  OSp_SetError(INVALID_SEM_ERROR);
  return 0;
}

/******************************************************************************
    OS_GetError
		
      Returns currently reported kernel error.
******************************************************************************/
kernelErrors OS_GetError(void)
{
  return currentError;
}


/******************************************************************************
    OSp_UpdateSatisfiedList
		
      Moves tasks from the satisfied list to the ready list if their deadline
    has passed
******************************************************************************/
void OSp_UpdateSatisfiedList(void)
{
  taskNode *node = taskSatisfiedHead;
  while(node)
  {
    if(OS_systemTick % node->tcb.taskDeadline == 0)
    {
      taskNode *temp = node->next;
      OSp_ChangeTaskState(SATISFIED, READY, node);
      node = temp;
    }
    else
      node = node->next;
  }
}

/******************************************************************************
    OSp_UpdateBlockedList
		
      Moves tasks from the blocked list to the ready list if they were blocked
    waiting for the passed semaphore
******************************************************************************/
void OSp_UpdateBlockedList(unsigned semID)
{
  taskNode *node = taskBlockedHead;
  while(node)
  {
    if(semaphores[semID-1].ownerList[node->tcb.taskID] > semaphores[semID-1].maxTokens)
    {
      taskNode *temp = node->next;
      OSp_ChangeTaskState(BLOCKED, READY, node);
      node = temp;
    }
    else
      node = node->next;
  }
}
/******************************************************************************
    OSp_ScheduleTask
		
      Updates OS_TaskRUNNING with current highest priority task.
******************************************************************************/
char OSp_ScheduleTask(void)
{
  //make sure that any tasks that have passed their deadlines and need to be run again are put back on the ready list
  OSp_UpdateSatisfiedList();

  //if the current task has been blocked, place it on the blocked list and switch to the next ready task
  if(OS_TaskRUNNING->taskState == BLOCKED)
  {
     OSp_ChangeTaskState(RUNNING, BLOCKED, taskRunningHead);
     OSp_ChangeTaskState(READY, RUNNING, taskReadyHead);
     ++OS_systemTick;
     return 1;
  }

  //if the current task has met its runtime, place it on the satisfied list and switch to the next ready task
  else if(++(OS_TaskRUNNING->taskTime) == OS_TaskRUNNING->taskRuntime)
  {
    OS_TaskRUNNING->taskTime = 0;
    OSp_ChangeTaskState(RUNNING, SATISFIED, taskRunningHead);
    OSp_UpdateSatisfiedList();
    OSp_ChangeTaskState(READY, RUNNING, taskReadyHead);
    ++OS_systemTick;
    return 1;
  }

  //if the next ready task is higher priority than the currently running task, place the currently running task on the ready list and switch to the next ready task
  else if(OSp_CompareNodePrios(taskReadyHead, taskRunningHead) == taskReadyHead)
  {
    OSp_ChangeTaskState(RUNNING, READY, taskRunningHead);
    OSp_ChangeTaskState(READY, RUNNING, taskReadyHead);
    ++OS_systemTick;
    return 1;
  }

  //no context switch
  ++OS_systemTick;
  return 0;
}


/******************************************************************************
    OSp_CompareNodePrios
		
      Checks whether Node A has higher priority than Node B, using EDF
******************************************************************************/
taskNode *OSp_CompareNodePrios(taskNode *A, taskNode *B)
{
  if(A && !B)
    return A;
  
  if(B && !A)
    return B;
  
  //if A has an earlier deadline than B, or the same deadline but higher priority, A is the winner
  if(A->tcb.taskDeadline < B->tcb.taskDeadline ||
    (A->tcb.taskDeadline == B->tcb.taskDeadline && 
     A->tcb.taskPriority <= B->tcb.taskPriority))
    return A;

  return B;
}


/******************************************************************************
    OSp_InsertNode
		
      Inserts a task node into the given list at a priority based position
******************************************************************************/
void OSp_InsertNode(taskState_t state, taskNode *node)
{
  node->tcb.taskState = state;
  taskNode **listHead;

  //get the correct list head based on passed state
  if(state == READY)
    listHead = &taskReadyHead;
  else if(state == RUNNING)
  {
    listHead = &taskRunningHead;
    OS_TaskRUNNING = &(node->tcb);
  }
  else if(state == SATISFIED)
    listHead = &taskSatisfiedHead;
  else if(state == BLOCKED)
    listHead = &taskBlockedHead;
  else
    return;
  
  //if the list is not empty and the head is higher priority than the new node,
  if(*listHead && OSp_CompareNodePrios(*listHead, node) == *listHead)
  {
    //loop until the end of the list or until the next node is of lower priority than the new node
    taskNode *tempNode = *listHead;
    while(tempNode->next && OSp_CompareNodePrios(tempNode->next, node) == tempNode->next)
      tempNode = tempNode->next;
    
    //insert the node
    node->next = tempNode->next;
    tempNode->next = node;
  }

  //if the new node belongs at the head, insert at the head
  else
  {
    node->next = *listHead;
    *listHead = node;
  }
}


/******************************************************************************
    OSp_RemoveNode
		
      Removes a node from the given list
******************************************************************************/
void OSp_RemoveNode(taskState_t state, taskNode *node)
{
  taskNode **listHead = 0;

  //get the correct list head based on the passed state.
  if(state == READY)
    listHead = &taskReadyHead;
  else if(state == RUNNING)
  {
    listHead = &taskRunningHead;
    OS_TaskRUNNING = 0;
  }
  else if(state == SATISFIED)
    listHead = &taskSatisfiedHead;
  else if(state == BLOCKED)
    listHead = &taskBlockedHead;
  else
    return;
  
  //if the list is not empty
  if(*listHead)
  {
    //if the head is not the node we're looking to remove
    if(*listHead != node)
    {
      //loop until the end of the list or until we find the node
      taskNode *tempNode = *listHead;
      while(tempNode->next)
      {
        if(tempNode->next == node)
        {
          //remove the node
          tempNode->next = node->next;
          node->next = 0;
          return;
        }
        tempNode = tempNode->next;
      }
    }
    //if the head of the list is the node, remove the head of the list
    else
    {
      *listHead = node->next;
      node->next = 0;
    }
  }
}


/******************************************************************************
    OSp_ChangeTaskState
		
      Moves the given node from the list it is in to the given list
******************************************************************************/
void OSp_ChangeTaskState(taskState_t currentState, taskState_t newState, taskNode *node)
{   
    OSp_RemoveNode(currentState, node);
    OSp_InsertNode(newState, node);
}

/******************************************************************************
    OSp_AllocateInit
		
      Sets up the allocation table for the passed block size.
******************************************************************************/
unsigned OSp_AllocateInit(unsigned blockSize)
{
  unsigned i;

  //Check to make sure blockSize is valid
  if(blockSize == 0 ||  OS_HEAP_SIZE % blockSize != 0 || 
     blockSize % OS_HEAP_BLOCK_ALIGN != 0 || blockSize > OS_HEAP_SIZE/2)
  {
    OSp_SetError(ALLOC_ERROR);
    return 0;
  }
  
  OS_blockSize = blockSize;

  //zero out the allocation table
  for(i = 1; i < OS_HEAP_SIZE / OS_HEAP_BLOCK_ALIGN; ++i)
    allocTable[i] = 0;

  //set the first memory chunk (size of heap)
  allocTable[0] = OS_HEAP_SIZE / blockSize;
  allocTable[(OS_HEAP_SIZE / blockSize) - 1] = OS_HEAP_SIZE / blockSize;
  OSp_InsertFreeHeap(OS_HEAP_SIZE / blockSize, 0);

  //return number of available minimum sized blocks
  return OS_HEAP_SIZE / blockSize;
}


/******************************************************************************
fds    OS_Malloc
		
      Selects a fitting memory block for the given blockSize (4-byte units)
    marks that block as in use and returns a pointer to the beginning of it.
******************************************************************************/
void *OS_Malloc(unsigned blockSize)
{
  //if there is no memory available or memory requested was of size 0, return an error
  if(numFreeBlocks == 0 || blockSize == 0)
  {
    OSp_SetError(ALLOC_ERROR);
    return 0;
  }

  //if the requested amount of memory doesn't align with the OS block size, add the difference
  if(blockSize % OS_blockSize != 0)
    blockSize += (OS_blockSize - (blockSize % OS_blockSize));

  //set blockSize in units of OS blocks
  blockSize /= OS_blockSize;

  //we're touching memory that will be shared amongst tasks, make sure we're not interrupted
  OS_DisableIRQ();

  //if there is enough free memory to give, mark the memory as in use
  if(blockSize <= freeBlocks[0].size)
  {
    unsigned size = freeBlocks[0].size;
    unsigned ind = freeBlocks[0].allocTableIndex;
    OSp_RemoveFreeHeap(size, ind);
    allocTable[ind + size - 1] = 0;
    allocTable[ind + size - blockSize] = blockSize * -1;

    //if the free block is bigger than what was requested, add the leftover memory back to free memory
    if(blockSize != size)
    {
      allocTable[ind] -= blockSize;
      allocTable[ind + size - 1 - blockSize] = size - blockSize;
      OSp_InsertFreeHeap(size - blockSize, ind);
    }
    OS_EnableIRQ();
    return &(heap[(ind + size - blockSize) * OS_blockSize]);
  }
  
  //Not enough memory to support the request, return an error
  else
  {
    OS_EnableIRQ();
    OSp_SetError(ALLOC_ERROR);
    return 0;
  }
}

/******************************************************************************
    OS_Free
		
      Marks the memory block pointer to by the given pointer as free and
    joins memory blocks if able.
******************************************************************************/
unsigned OS_Free(void *ptr)
{
  //get the heap index from the given pointer
  unsigned heapInd = ((unsigned) ptr - (unsigned) &(heap[0])) / sizeof(unsigned);

  //if the pointer is in the heap and aligned propery,
  if(heapInd < OS_HEAP_SIZE && heapInd % OS_blockSize == 0)
  {
    unsigned allocTableInd = heapInd / OS_blockSize;

    //touching memory shared amongst tasks now, don't let interrupts happen
    OS_DisableIRQ();

    //if the given pointer points to memory that is in use, mark the memory as free
    if(allocTable[allocTableInd] < 0)
    {
      allocTable[allocTableInd] *= -1;
      unsigned memSize = allocTable[allocTableInd];

      //if there is a free block above the block being freed, merge the two
      if(allocTableInd > 0 && allocTable[allocTableInd - 1] > 0)
      {
        unsigned joinedMemSize = allocTable[allocTableInd - 1];
        allocTable[allocTableInd] = 0;
        allocTable[allocTableInd - 1] = 0;
        allocTable[allocTableInd + memSize - 1] = memSize + joinedMemSize; 
        allocTable[allocTableInd - joinedMemSize] = memSize + joinedMemSize;
        OSp_RemoveFreeHeap(joinedMemSize, allocTableInd - joinedMemSize);
        OSp_InsertFreeHeap(memSize + joinedMemSize, allocTableInd - joinedMemSize);
      }
      //if there is a free block below the block being freed, merge the two
      else if(allocTableInd + memSize < (OS_HEAP_SIZE / OS_blockSize) && allocTable[allocTableInd + memSize] > 0)
      {
        unsigned joinedMemSize = allocTable[allocTableInd + memSize];
        allocTable[allocTableInd + memSize - 1] = 0;
        allocTable[allocTableInd + memSize] = 0;
        allocTable[allocTableInd] = memSize + joinedMemSize;
        allocTable[allocTableInd + memSize + joinedMemSize - 1] = memSize + joinedMemSize;
        OSp_RemoveFreeHeap(joinedMemSize, allocTableInd + memSize);
        OSp_InsertFreeHeap(memSize + joinedMemSize, allocTableInd);
      }

      //if there are no free blocks adjacent to the block being free, just free the block.
      else
      {
        allocTable[allocTableInd + memSize - 1] = memSize;
        OSp_InsertFreeHeap(memSize, allocTableInd);
      }
      OS_EnableIRQ();
      return (unsigned) ptr;
    }
  }
  //if the pointer is not on the heap or does not point to the start of a block in use, return an error
  OS_EnableIRQ();
  OSp_SetError(FREE_ERROR);
  return 0;
}

/******************************************************************************
    OSp_InsertFreeHeap
		
      Add a node to the free memory heap, then sort the heap.
******************************************************************************/
void OSp_InsertFreeHeap(unsigned size, unsigned allocIndex)
{
  if(size > 0)
  {
    //add the new free block node to the end of the heap
    unsigned pos = numFreeBlocks;
    freeBlocks[pos].size = size;
    freeBlocks[pos].allocTableIndex = allocIndex;

    //bubble the new node up through its parents until the heap is sorted properly
    while(pos != 0 && freeBlocks[pos].size > freeBlocks[PARENT_INDEX(pos)].size)
    {
      OSp_SwapMemNodes(&(freeBlocks[pos]), &(freeBlocks[PARENT_INDEX(pos)]));
      pos = PARENT_INDEX(pos);
    }
    ++numFreeBlocks;
  }
}


/******************************************************************************
    OSp_RemoveFreeHeap
		
      Remove a node from the free memory heap, then sort the heap.
******************************************************************************/
void OSp_RemoveFreeHeap(unsigned size, unsigned allocIndex)
{
  if(numFreeBlocks)
  {
    //loop until we find the position of the correct node, or hit the end of the list
    unsigned pos = 0;
    while(pos < numFreeBlocks-1 && freeBlocks[pos].allocTableIndex != allocIndex)
      ++pos;
    
    //ensure that the size and allocation table index match
    if(freeBlocks[pos].size == size && freeBlocks[pos].allocTableIndex == allocIndex)
    {
      //swap the values of the last node with the node to remove
      OSp_SwapMemNodes(&(freeBlocks[pos]), &(freeBlocks[numFreeBlocks-1]));

      //drop the last node off of the heap
      --numFreeBlocks;

      //trickle the swapped node down through its children until the heap is sorted again
      while((LEFT_CHILD_INDEX(pos) < numFreeBlocks && freeBlocks[pos].size < freeBlocks[LEFT_CHILD_INDEX(pos)].size) ||
            (RIGHT_CHILD_INDEX(pos) < numFreeBlocks && freeBlocks[pos].size < freeBlocks[RIGHT_CHILD_INDEX(pos)].size))
      {
        if(RIGHT_CHILD_INDEX(pos) >= numFreeBlocks || freeBlocks[LEFT_CHILD_INDEX(pos)].size > freeBlocks[RIGHT_CHILD_INDEX(pos)].size)
        {
          OSp_SwapMemNodes(&(freeBlocks[pos]), &(freeBlocks[LEFT_CHILD_INDEX(pos)]));
          pos = LEFT_CHILD_INDEX(pos);
        }
        else
        {
          OSp_SwapMemNodes(&(freeBlocks[pos]), &(freeBlocks[RIGHT_CHILD_INDEX(pos)]));
          pos = RIGHT_CHILD_INDEX(pos);
        }
      }
    }
  }
}


/******************************************************************************
    OSp_SwapMemNodes
		
      Swap the values between two given memory nodes
******************************************************************************/
void OSp_SwapMemNodes(memNode *A, memNode *B)
{
  memNode temp;
  temp.size = (*A).size;
  temp.allocTableIndex = (*A).allocTableIndex;
  (*A).size = (*B).size;
  (*A).allocTableIndex = (*B).allocTableIndex;
  (*B).size = temp.size;
  (*B).allocTableIndex = temp.allocTableIndex;
}

/******************************************************************************
    OSp_SetError
		
      Sets the current error state of the OS to the passed value.
******************************************************************************/
void OSp_SetError(kernelErrors error)
{
  currentError = error;
}

/******************************************************************************
    OSp_IdleTask
		
      This task should always be created, have the lowest possible (worst) 
    priority, and never be prevented from running.
******************************************************************************/
void OSp_IdleTask(void) 
{
    while (0x01) 
    {
      ;
    }

}
