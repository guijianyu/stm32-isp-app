#ifndef __IWDG_model_H
#define	__IWDG_model_H

#include "stm32f0xx.h"
void bsp_InitIwdg(uint32_t _ulIWDGTime);
void IWDG_Feed(void);

#endif /*__IWDG_model_H*/
