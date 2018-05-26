#include "fput_debug.h"

#ifdef DEBUF_VIEWER_PRINTF_USE

struct __FILE { int handle; };
FILE __stdout;
FILE __stdin;

int fputc(int ch, FILE *f) 
{
	ITM_SendChar( ch );
//   if (DEMCR & TRCENA) 
// 	{
//     while (ITM_Port32(0) == 0);
//     ITM_Port8(0) = ch;
// 		
//   }
  return(ch);
}

#endif


