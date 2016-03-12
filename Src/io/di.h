#ifndef __DI_H__
#define __DI_H__

#include "dio_typedef.h"

#ifdef __cplusplus
extern "C"
{
#endif


/* Exported typedef ----------------------------------------------------------*/	
typedef struct
{
	const digital_io_t dio;
} digital_input_t;
	
typedef digital_input_t* ptr_digital_input_t;	

	
/* Exported functions --------------------------------------------------------*/
void DigitalInput_Init(ptr_digital_input_t di);
GPIO_PinState DigitalInput_ReadPin(ptr_digital_input_t di);
	

/* Exported variables ---------------------------------------------------------*/
extern ptr_digital_input_t pOnboard_Btn;
	
#ifdef __cplusplus
}
#endif

#endif // __DI_H__
