#ifndef _DBGUART_H_
#define _DBGUART_H_

#include "embARC.h"

extern void dbgInit();
extern void UartInit(void);
extern void UartGetLine(char *line);
extern int putStr(const char *format, ...);

#endif /* _DBGUART_H_ */