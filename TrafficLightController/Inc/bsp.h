#ifndef __BSP_H
#define __BSP_H

#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx_hal.h"

//Exported Global Variables
extern int Button_Status;

//Function Prototypes
void BSP_Init(void);
void Signal_Pass(void);
void Signal_Block(void);
void Signal_Flash(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_H */