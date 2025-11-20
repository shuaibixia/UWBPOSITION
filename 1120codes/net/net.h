
#ifndef __net_h__
#define __net_h__

#include "../lib/rcmIf.h"

void    netMain();

void    netInit();

void    netFree();

void    netCmd  (char* cmd, unsigned int len);

void    netRecv (unsigned char* buf, unsigned int& len);

void    netParse(unsigned char* buf, unsigned int len );

void    HumaticNet();

void    HumaParse (unsigned char* buf, unsigned int len );

#endif
