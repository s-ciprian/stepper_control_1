#ifndef __DIO_TYPEDEF_H__
#define __DIO_TYPEDEF_H__

#include "stm32f4xx_hal.h"
#include "stm32f4xx_nucleo.h"

#ifdef __cplusplus
extern "C"
{
#endif

	
typedef uint16_t      pin_t;
typedef GPIO_TypeDef* port_t;
typedef uint32_t      pull_t;
typedef uint32_t      pmode_t;

typedef struct
{
	const pmode_t  pmode;
	const port_t   port;
	const pin_t    pin;
	const pull_t   pull;
} digital_io_t;

typedef digital_io_t* p_digital_io_t;

	
#ifdef __cplusplus
}
#endif

#endif // __DIO_TYPEDEF_H__
		