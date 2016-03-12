#ifndef __DO_H__
#define __DO_H__

#include "dio_typedef.h"

#ifdef __cplusplus
extern "C"
{
#endif

/* Exported typedef ----------------------------------------------------------*/	
typedef struct
{
	const digital_io_t dio;
} digital_output_t;

typedef digital_output_t* ptr_digital_output_t;

/* Exported functions --------------------------------------------------------*/
void DigitalOutput_Init(ptr_digital_output_t dq);	
void DigitalOutput_Toggle(ptr_digital_output_t dq);
void DigitalOutput_SetHigh(ptr_digital_output_t dq);
void DigitalOutput_SetLow(ptr_digital_output_t dq);

/* Exported variables ---------------------------------------------------------*/
extern ptr_digital_output_t pAlarm_LED;
	

#ifdef __cplusplus
}
#endif

#endif // __DO_H__
