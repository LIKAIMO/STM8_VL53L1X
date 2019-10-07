
#ifndef _BSP_H
#define _BSP_H

#include "stm8s.h"

typedef struct s_dataPackage
{
    u8 status;
    u8 data[10];
}t_dataPackage;

extern t_dataPackage *pDataPackage;

void sysInit(void);
void dataHandle(void);

#endif
