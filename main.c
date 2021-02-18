/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------

File    : main.c
Purpose : Generic application start

*/

#include <stdio.h>
#include <stdlib.h>
#include "STM32F429ZI_HAL.h"
#include "LilOS.h"
/*********************************************************************
*
*       main()
*
*  Function description
*   Application entry point.
*/
int main(void) {
  OS_InitKernelHAL();
  OS_InitKernel(1, 40);
  OS_Start();
  return 0;
}

/*************************** End of file ****************************/
