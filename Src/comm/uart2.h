
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART2_H
#define __UART2_H

#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"

void uart2_Init(void);
void uart2_DeInit(void);
void uart2_Transmit(uint8_t *pData, uint16_t Size);


#endif /* __UART2_H */

/* Includes ------------------------------------------------------------------*/
