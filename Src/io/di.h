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
	const uint16_t time;  // Debounce time for given input
	uint16_t maximum;     // Used in task code to compute the value used with integrator (maximum = time/recurrence)
	uint16_t fl_input;      // Input value after filtering
	uint16_t integrator;  // For filter implementation
} debounce_t;
	
typedef struct
{
	const digital_io_t dio;
	        debounce_t debounce;
} digital_input_t;
	
typedef digital_input_t* ptr_digital_input_t;	

	
/* Exported functions --------------------------------------------------------*/
void DigitalInput_Init(ptr_digital_input_t di);
GPIO_PinState DigitalInput_ReadPin(ptr_digital_input_t di);
void DigitalInput_DebouncePin(ptr_digital_input_t di);


/* Exported variables ---------------------------------------------------------*/
extern ptr_digital_input_t pOnboard_Btn;
extern ptr_digital_input_t pUsr_Btn_1;
extern ptr_digital_input_t pUsr_Btn_2;
	
#ifdef __cplusplus
}
#endif

#endif // __DI_H__
