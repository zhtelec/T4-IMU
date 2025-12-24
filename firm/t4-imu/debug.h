#ifndef _DEBUG_H_
#define _DEBUG_H_


void            DebugInit(void);
void            DebugLoop(void);


#ifdef  _DEBUG_CPP_
static void     DebugUartAs6668Init(void);
static void     DebugUartAs6668Loop(void);
#endif

#endif
