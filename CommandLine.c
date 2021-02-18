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

#define MIN(X, Y) ((X) < (Y) ? (X) : (Y))

uint8_t inCL = 1;

char Compare(char *com, char *buf)
{
  return !strncmp(com, buf, strlen(com));
}

void Query_enable(char* msg)
{
  sprintf(msg, "%d: %s", uart1Config.en, uart1Config.en? "enabled" : "disabled");
}

void Command_enable(char* msg, uint8_t param)
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

void Query_9bit(char* msg)
{
  sprintf(msg, "%u: %u data bits\n", uart1Config.bitMode, uart1Config.bitMode+8);
}

void Command_9bit(char* msg, uint8_t param)
{
  if(param > 1)
    strcpy(msg, "Invalid parameter, use the following:\r\n0: 8 bit mode\r\n1: 9 bit mode\n");
  else
  {
    uart1Config.bitMode = param;
    strcpy(msg, "OK\n");
  }
}

void Query_parity(char* msg)
{
  sprintf(msg, "%d: parity %s\n", uart1Config.parityEn, uart1Config.parityEn? "enabled" : "disabled");
}

void Command_parity(char* msg, uint8_t param)
{
  if(param > 1)
    strcpy(msg, "Invalid parameter, use the following:\r\n0: disable parity\r\n1: enable parity\n");
  else
  {
    uart1Config.parityEn = param;
    strcpy(msg, "OK\n");
  }
}

void Query_parityOdd(char* msg)
{
  sprintf(msg, "%d: %s parity\n", uart1Config.parityMode, uart1Config.parityMode? "odd" : "even");
}

void Command_parityOdd(char* msg, uint8_t param)
{
  if(param > 1)
    strcpy(msg, "Invalid parameter, use the following:\r\n0: even parity\r\n1: odd parity\n");
  else
  {
    uart1Config.parityMode = param;
    strcpy(msg, "OK\n");
  }
}

void Query_stop(char* msg)
{
  sprintf(msg, "%d: %d%s stop bit(s)\n", uart1Config.stopMode, (uart1Config.stopMode+1)/2, uart1Config.stopMode%2? "" : ".5" );
}

void Command_stop(char* msg, uint8_t param)
{
  if(param > 3)
    strcpy(msg, "Invalid parameter, use the following:\r\n0: 0.5 stop bits\r\n1: 1 stop bit\r\n2: 1.5 stop bits\r\n3: 2 stop bits\n");
  else
  {
    uart1Config.stopMode = param;
    strcpy(msg, "OK\n");
  }
}

void Query_mantissa(char* msg)
{
  sprintf(msg, "%d\n", uart1Config.baud_mant);
}

void Command_mantissa(char* msg, uint16_t param)
{
  if(param > 2047)
    strcpy(msg, "Invalid parameter, use the following:\r\n[0-2047]\n");
  else
  {
    uart1Config.baud_mant = param;
    strcpy(msg, "OK\n");
  }
}

void Query_fraction(char* msg)
{
  sprintf(msg, "%d\n", uart1Config.baud_fract);
}

void Command_fraction(char* msg, uint8_t param)
{
  if(param > 7)
    strcpy(msg, "Invalid parameter, use the following:\r\n[0-7]\n");
  else
  {
    uart1Config.baud_fract = param;
    strcpy(msg, "OK\n");
  }
}

void Command_CL(char *buf, uint8_t size)
{
  //TODO: make command recognition smarter
  //TODO: fix compare so that appended strings don't trigger their shorter versions
  char msg[200];
  if(Compare("help", buf))
  {
    Command_help(msg);
  }
  
  else if(Compare("UART1-enable?", buf))
  {
    Query_enable(msg);
  }
  else if(Compare("UART1-enable", buf))
  {
    Command_enable(msg, atoi(buf+strlen("UART1-enable")));
  }

  else if(Compare("UART1-9bit?", buf))
  {
    Query_9bit(msg);
  }
  else if(Compare("UART1-9bit", buf))
  {
    Command_9bit(msg, atoi(buf+strlen("UART1-9bit")));
  }

  else if(Compare("UART1-parity-odd?", buf))
  {
    Query_parityOdd(msg);
  }
   else if(Compare("UART1-parity-odd", buf))
  {
    Command_parityOdd(msg, atoi(buf+strlen("UART1-parity-odd")));
  }

  else if(Compare("UART1-parity-en?", buf))
  {
    Query_parity(msg);
  }
  else if(Compare("UART1-parity-en", buf))
  {
    Command_parity(msg, atoi(buf+strlen("UART1-parity-en")));
  }
  
  else if(Compare("UART1-stop?", buf))
  {
    Query_stop(msg);
  }
   else if(Compare("UART1-stop", buf))
  {
    Command_stop(msg, atoi(buf+strlen("UART1-stop")));
  }
  
  else if(Compare("UART1-baud-mantissa?", buf))
  {
    Query_mantissa(msg);
  }
   else if(Compare("UART1-baud-mantissa", buf))
  {
    Command_mantissa(msg, atoi(buf+strlen("UART1-baud-mantissa")));
  }
  
  else if(Compare("UART1-baud-fraction?", buf))
  {
    Query_fraction(msg);
  }
   else if(Compare("UART1-baud-fraction", buf))
  {
    Command_fraction(msg, atoi(buf+strlen("UART1-baud-fraction")));
  }

  else if(Compare("startOS", buf))
  {
    strcpy(msg, "OK\n");
    inCL = 0;
  }
  else 
   strcpy(msg, "Unrecognized Command, try \"help\".");

  OS_SendString_UART(msg, 0);
}
