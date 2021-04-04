/******************************************************************************
*******************************************************************************
Author      : Collin MacDicken
File        : CommandLine.h
Description : Header for CommandLine

*******************************************************************************
******************************************************************************/
#ifndef		__COMMANDLINE__H__
#define		__COMMANDLINE__H__

extern uint8_t inCL;



void CommandCL(uint8_t *buf, uint8_t size);

char Compare(char *com, char *buf);


#endif