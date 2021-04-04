/******************************************************************************
*******************************************************************************
Author      : Collin MacDicken
File        : CommandLine.c
Description : Implementation of CommandLine Functions

*******************************************************************************
******************************************************************************/
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "STM32F429ZI_HAL.h"
#include "LilOS.h"

#define MIN(X, Y) ((X) < (Y) ? (X) : (Y))

uint8_t inCL = 1;
uint8_t edit = 0;


char Compare(char *com, char *buf)
{
  return !strncmp(com, buf, strlen(com));
}

void Query_UARTenable(char* msg)
{
  sprintf(msg, "%d: %s", uart1Config.en, uart1Config.en? "enabled" : "disabled");
}

void Command_UARTenable(char* msg, uint8_t param)
{
  if(param > 1)
    strcpy(msg, "Invalid parameter, use the following:\r\n0: disable UART1\r\n1: enable UART1\n");
  else
  {
    uart1Config.en = param;
    strcpy(msg, "OK\n");
  }
}

void Command_help(char* msg)
{
  strcpy(msg, "helpmsg\n");
}

void Query_UART9bit(char* msg)
{
  sprintf(msg, "%u: %u data bits\n", uart1Config.bitMode, uart1Config.bitMode+8);
}

void Command_UART9bit(char* msg, uint8_t param)
{
  if(param > 1)
    strcpy(msg, "Invalid parameter, use the following:\r\n0: 8 bit mode\r\n1: 9 bit mode\n");
  else
  {
    uart1Config.bitMode = param;
    strcpy(msg, "OK\n");
  }
}

void Query_UARTparity(char* msg)
{
  sprintf(msg, "%d: parity %s\n", uart1Config.parityEn, uart1Config.parityEn? "enabled" : "disabled");
}

void Command_UARTparity(char* msg, uint8_t param)
{
  if(param > 1)
    strcpy(msg, "Invalid parameter, use the following:\r\n0: disable parity\r\n1: enable parity\n");
  else
  {
    uart1Config.parityEn = param;
    strcpy(msg, "OK\n");
  }
}

void Query_UARTparityOdd(char* msg)
{
  sprintf(msg, "%d: %s parity\n", uart1Config.parityMode, uart1Config.parityMode? "odd" : "even");
}

void Command_UARTparityOdd(char* msg, uint8_t param)
{
  if(param > 1)
    strcpy(msg, "Invalid parameter, use the following:\r\n0: even parity\r\n1: odd parity\n");
  else
  {
    uart1Config.parityMode = param;
    strcpy(msg, "OK\n");
  }
}

void Query_UARTstop(char* msg)
{
  sprintf(msg, "%d: %d%s stop bit(s)\n", uart1Config.stopMode, (uart1Config.stopMode+1)/2, uart1Config.stopMode%2? "" : ".5" );
}

void Command_UARTstop(char* msg, uint8_t param)
{
  if(param > 3)
    strcpy(msg, "Invalid parameter, use the following:\r\n0: 0.5 stop bits\r\n1: 1 stop bit\r\n2: 1.5 stop bits\r\n3: 2 stop bits\n");
  else
  {
    uart1Config.stopMode = param;
    strcpy(msg, "OK\n");
  }
}

void Query_UARTmantissa(char* msg)
{
  sprintf(msg, "%d\n", uart1Config.baud_mant);
}

void Command_UARTmantissa(char* msg, uint16_t param)
{
  if(param > 2047)
    strcpy(msg, "Invalid parameter, use the following:\r\n[0-2047]\n");
  else
  {
    uart1Config.baud_mant = param;
    strcpy(msg, "OK\n");
  }
}

void Query_UARTfraction(char* msg)
{
  sprintf(msg, "%d\n", uart1Config.baud_fract);
}

void Command_UARTfraction(char* msg, uint8_t param)
{
  if(param > 7)
    strcpy(msg, "Invalid parameter, use the following:\r\n[0-7]\n");
  else
  {
    uart1Config.baud_fract = param;
    strcpy(msg, "OK\n");
  }
}

void Query_GPIOenable(char *msg)
{
  sprintf(msg, "%d: %s\n", gpioCConfig.en, gpioCConfig.en? "GPIOC enabled" : "GPIOC disabled");
}

void Command_GPIOenable(char *msg, uint8_t val)
{
  if(val > 1)
    strcpy(msg, "invalid parameter, use the following:\r\n0: disable GPIO\r\n1: enable GPIO");
  else
  {
    gpioCConfig.en = val;
    strcpy(msg, "OK\n");
  }
}

void Query_GPIOtype(char *msg)
{
  uint8_t i;
  char temp[30];
  strcpy(msg, "");
  for(i = 0; i < NUM_GPIOS; ++i)
  {
    sprintf(temp, "Pin %d - %d: %s\r\n", i, (gpioCConfig.dir)[i], (gpioCConfig.dir)[i]? "Output" : "Input");
    strcat(msg, temp);
  }
}

void Command_GPIOtype(char *msg, uint8_t val, uint8_t pin)
{
  if(val > 1 || pin >= NUM_GPIOS)
  {
    strcpy(msg, "invalid parameter, use the following:\r\nParam 1 - setting:\r\n0: input\r\n1: output\r\nParam 2 - pin number:\r\n[0-15]");
  }
  else
  {
    (gpioCConfig.dir)[pin] = val;
    strcpy(msg, "OK\n");
  }
}

void Query_GPIOoutType(char *msg)
{
  uint8_t i;
  char temp[30];
  strcpy(msg, "");
  for(i = 0; i < NUM_GPIOS; ++i)
  {
    sprintf(temp, "Pin %d - %d: %s\r\n", i, (gpioCConfig.oType)[i], (gpioCConfig.oType)[i]? "Push-Pull" : "Open-Drain");
    strcat(msg, temp);
  }
}

void Command_GPIOoutType(char *msg, uint8_t val, uint8_t pin)
{
  if(val > 1 || pin >= NUM_GPIOS)
  {
    strcpy(msg, "invalid parameter, use the following:\r\nParam 1 - setting:\r\n0: Push-Pull\r\n1: Open-Drain\r\nParam 2 - pin number:\r\n[0-15]");
  }
  else
  {
    (gpioCConfig.oType)[pin] = val;
    strcpy(msg, "OK\n");
  }
}

void Query_GPIOpull(char *msg)
{
  uint8_t i;
  char temp[30];
  strcpy(msg, "");
  for(i = 0; i < NUM_GPIOS; ++i)
  {
    sprintf(temp, "Pin %d - %d: %s\r\n", i, (gpioCConfig.pull)[i], (gpioCConfig.pull)[i]? ((gpioCConfig.pull)[i]==1? "Pull-up" : "Pull-down") : "No Pull-up/down");
    strcat(msg, temp);
  }
}

void Command_GPIOpull(char *msg, uint8_t val, uint8_t pin)
{
  if(val > 2 || pin >= NUM_GPIOS)
  {
    strcpy(msg, "invalid parameter, use the following:\r\nParam 1 - setting:\r\n0: No Pull-up/down\r\n1: Pull-up\r\n2: Pull-down\r\nParam 2 - pin number:\r\n[0-15]");
  }
  else
  {
    (gpioCConfig.pull)[pin] = val;
    strcpy(msg, "OK\n");
  }
}

void DumpTaskInfo(char *msg)
{
  char temp[100];
  if(edit > 0 && edit < OS_MAX_BITASKS)
  {
    sprintf(temp, "Task %d: %s\r\nPriority: %d   Runtime: %d   Deadline: %d\r\n", edit, BI_tasks[edit-1].en? "Enabled" : "Disabled",
            BI_tasks[edit-1].priority, BI_tasks[edit-1].runtime, BI_tasks[edit-1].deadline);
    strcat(msg, temp);
    strcat(msg, BI_tasks[edit-1].funcString);
    strcat(msg, "\r\n\n\n");
  }
  else
  {
    sprintf(temp, "Invalid task index, use 1-%d.\n", OS_MAX_BITASKS);
    strcat(msg, temp);
  }
}

void Add_SetGPIO(char *msg, char port, uint8_t pin)
{
  char temp[30];
  if(BI_tasks[edit-1].numsteps < OS_MAX_STEPS_PER_TASK)
  {
    if(port >= 'A' && port <= 'K' && pin >=0 && pin <=15)
    {
      sprintf(temp, "  SetGPIO%c(%d)\r\n", port, pin);
      strcat(BI_tasks[edit-1].funcString, temp);
      ++(BI_tasks[edit-1].numsteps);
      strcpy(msg, "");
      DumpTaskInfo(msg);
    }
    else
    {
      strcpy(msg, "invalid port or pin number, use the following:\r\nPorts: A-K\r\nPins:0-15");
    }
  }
  else
  {
    strcpy(msg, "Max number of steps reached, try \"clear\" to start over.\n");
  }
}

void Add_ClearGPIO(char *msg, char port, uint8_t pin)
{
  char temp[30];
  if(BI_tasks[edit-1].numsteps < OS_MAX_STEPS_PER_TASK)
  {
    if(port >= 'A' && port <= 'K' && pin >=0 && pin <=15)
    {
      sprintf(temp, "  ClearGPIO%c(%d)\r\n", port, pin);
      strcat(BI_tasks[edit-1].funcString, temp);
      ++(BI_tasks[edit-1].numsteps);
      strcpy(msg, "");
      DumpTaskInfo(msg);
    }
    else
    {
      strcpy(msg, "invalid port or pin number, use the following:\r\nPorts: A-K\r\nPins:0-15");
    }
  }
  else
  {
    strcpy(msg, "Max number of steps reached, try \"clear\" to start over.\n");
  }
}

void Add_Wait(char *msg, uint16_t ms)
{
  char temp[30];
  if(BI_tasks[edit-1].numsteps < OS_MAX_STEPS_PER_TASK)
  {
    sprintf(temp, "  Wait_ms(%d)\r\n", ms);
    strcat(BI_tasks[edit-1].funcString, temp);
    ++(BI_tasks[edit-1].numsteps);
    strcpy(msg, "");
    DumpTaskInfo(msg);
  }
  else
  {
    strcpy(msg, "Max number of steps reached, try \"clear\" to start over.\n");
  }
}

void Add_SetVar(char *msg, uint16_t val)
{
  char temp[30];
  if(BI_tasks[edit-1].numsteps < OS_MAX_STEPS_PER_TASK)
  {
    sprintf(temp, "  SetVar(%d)\r\n", val);
    strcat(BI_tasks[edit-1].funcString, temp);
    ++(BI_tasks[edit-1].numsteps);
    strcpy(msg, "");
    DumpTaskInfo(msg);
  }
  else
  {
    strcpy(msg, "Max number of steps reached, try \"clear\" to start over.\n");
  }
}

void Add_AddVar(char *msg, uint16_t val)
{
  char temp[30];
  if(BI_tasks[edit-1].numsteps < OS_MAX_STEPS_PER_TASK)
  {
    sprintf(temp, "  AddVar(%d)\r\n", val);
    strcat(BI_tasks[edit-1].funcString, temp);
    ++(BI_tasks[edit-1].numsteps);
    strcpy(msg, "");
    DumpTaskInfo(msg);
  }
  else
  {
    strcpy(msg, "Max number of steps reached, try \"clear\" to start over.\n");
  }
}

void Add_SubVar(char *msg, uint16_t val)
{
  char temp[30];
  if(BI_tasks[edit-1].numsteps < OS_MAX_STEPS_PER_TASK)
  {
    sprintf(temp, "  SubVar(%d)\r\n", val);
    strcat(BI_tasks[edit-1].funcString, temp);
    ++(BI_tasks[edit-1].numsteps);
    strcpy(msg, "");
    DumpTaskInfo(msg);
  }
  else
  {
    strcpy(msg, "Max number of steps reached, try \"clear\" to start over.\n");
  }
}

void DisableTask(char *msg)
{
  BI_tasks[edit-1].en = 0;
  strcpy(msg, "");
  DumpTaskInfo(msg);
}

void EnableTask(char *msg)
{
  BI_tasks[edit-1].en = 1;
  strcpy(msg, "");
  DumpTaskInfo(msg);
}

void ClearTask(char *msg)
{
  BI_tasks[edit-1].numsteps = 0;
  strcpy(BI_tasks[edit-1].funcString, "");
  strcpy(msg, "");
  DumpTaskInfo(msg);
}

void SetPrio(char *msg, uint16_t val)
{
  BI_tasks[edit-1].priority = val;
  strcpy(msg, "");
  DumpTaskInfo(msg);
}

void SetDead(char *msg, uint16_t val)
{
  BI_tasks[edit-1].deadline = val;
  strcpy(msg, "");
  DumpTaskInfo(msg);
}

void SetRunt(char *msg, uint16_t val)
{
  BI_tasks[edit-1].runtime = val;
  strcpy(msg, "");
  DumpTaskInfo(msg);
}

void Command_CL(char *buf, uint8_t size)
{
  //TODO: make command recognition smarter
  //TODO: fix compare so that appended strings don't trigger their shorter versions
  char msg[500];
  if(edit == 0)
  {
    if(Compare("help", buf))
    {
      Command_help(msg);
    }
  
    else if(Compare("UART1-enable?", buf))
    {
      Query_UARTenable(msg);
    }
    else if(Compare("UART1-enable", buf))
    {
      Command_UARTenable(msg, atoi(buf+strlen("UART1-enable")));
    }

    else if(Compare("UART1-9bit?", buf))
    {
      Query_UART9bit(msg);
    }
    else if(Compare("UART1-9bit", buf))
    {
      Command_UART9bit(msg, atoi(buf+strlen("UART1-9bit")));
    }

    else if(Compare("UART1-parity-odd?", buf))
    {
      Query_UARTparityOdd(msg);
    }
     else if(Compare("UART1-parity-odd", buf))
    {
      Command_UARTparityOdd(msg, atoi(buf+strlen("UART1-parity-odd")));
    }

    else if(Compare("UART1-parity-en?", buf))
    {
      Query_UARTparity(msg);
    }
    else if(Compare("UART1-parity-en", buf))
    {
      Command_UARTparity(msg, atoi(buf+strlen("UART1-parity-en")));
    }
  
    else if(Compare("UART1-stop?", buf))
    {
      Query_UARTstop(msg);
    }
     else if(Compare("UART1-stop", buf))
    {
      Command_UARTstop(msg, atoi(buf+strlen("UART1-stop")));
    }
  
    else if(Compare("UART1-baud-mantissa?", buf))
    {
      Query_UARTmantissa(msg);
    }
     else if(Compare("UART1-baud-mantissa", buf))
    {
      Command_UARTmantissa(msg, atoi(buf+strlen("UART1-baud-mantissa")));
    }
  
    else if(Compare("UART1-baud-fraction?", buf))
    {
      Query_UARTfraction(msg);
    }
     else if(Compare("UART1-baud-fraction", buf))
    {
      Command_UARTfraction(msg, atoi(buf+strlen("UART1-baud-fraction")));
    }
  
    else if(Compare("GPIOC-enable?", buf))
    {
      Query_GPIOenable(msg);
    }
    else if(Compare("GPIOC-enable", buf))
    {
      Command_GPIOenable(msg, atoi(buf+strlen("GPIOC-enable")));
    }

    else if(Compare("GPIOC-type?", buf))
    {
      Query_GPIOtype(msg);
    }
    else if(Compare("GPIOC-type", buf))
    {
      Command_GPIOtype(msg, atoi(buf+strlen("GPIOC-type")), atoi(buf+strlen("GPIOC-type x")));
    }
  
    else if(Compare("GPIOC-out-type?", buf))
    {
      Query_GPIOoutType(msg);
    }
    else if(Compare("GPIOC-out-type", buf))
    {
      Command_GPIOoutType(msg, atoi(buf+strlen("GPIOC-out-type")), atoi(buf+strlen("GPIOC-out-type x")));
    }

    else if(Compare("GPIOC-pullr?", buf))
    {
      Query_GPIOpull(msg);
    }
    else if(Compare("GPIOC-pullr", buf))
    {
      Command_GPIOpull(msg, atoi(buf+strlen("GPIOC-pullr")), atoi(buf+strlen("GPIOC-pullr x")));
    }
    else if(Compare("edit_task", buf))
    {
      edit = atoi(buf+strlen("task_edit"));
      edit = edit < 6? edit : 0;
      strcpy(msg, "");
      DumpTaskInfo(msg);
    }
    else if(Compare("startOS", buf))
    {
      uint8_t i;
      for(i = 0; i < OS_MAX_BITASKS; ++i)
      {
        strcat(BI_tasks[i].funcString, ";");
      }
      strcpy(msg, "OK\n");
      inCL = 0;
    }
    else 
     strcpy(msg, "Unrecognized Command, try \"help\".");
  }
  else
  {
    if(Compare("exit", buf))
    {
      strcpy(msg, "Task procedure saved. Returning to HW Configuration mode.");
      edit = 0;
    }
    else if(Compare("enable", buf))
    {
      EnableTask(msg);
    }
    else if(Compare("disable", buf))
    {
      DisableTask(msg);
    }
    else if(Compare("clear", buf))
    {
      ClearTask(msg);
    }
    else if(Compare("priority ", buf))
    {
      SetPrio(msg, atoi(buf+strlen("priority")));
    }
    else if(Compare("deadline ", buf))
    {
      SetDead(msg, atoi(buf+strlen("deadline")));
    }
    else if(Compare("runtime ", buf))
    {
      SetRunt(msg, atoi(buf+strlen("runtime")));
    }
    else if(Compare("SetGPIO", buf))
    {
      Add_SetGPIO(msg, buf[strlen("SetGPIO")], atoi(buf+strlen("SetGPIOA(")));
    }
    else if(Compare("ClearGPIO", buf))
    {
      Add_ClearGPIO(msg, buf[strlen("ClearGPIO")], atoi(buf+strlen("ClearGPIOA(")));
    }
    else if(Compare("Wait_ms", buf))
    {
      Add_Wait(msg, atoi(buf+strlen("Wait_ms(")));
    }
    else if(Compare("SetVar", buf))
    {
      Add_SetVar(msg, atoi(buf + strlen("SetVar(")));
    }
     else if(Compare("AddVar", buf))
    {
      Add_AddVar(msg, atoi(buf + strlen("AddVar(")));
    }
     else if(Compare("SubVar", buf))
    {
      Add_SubVar(msg, atoi(buf + strlen("SubVar(")));
    }
  }
  OS_SendString_UART(msg, 0);
}
