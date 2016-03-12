/* Includes ------------------------------------------------------------------*/
#include "di.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
digital_input_t Onboard_Btn =
{
	.dio.pmode = GPIO_MODE_INPUT,
	.dio.pull = GPIO_PULLDOWN,
	.dio.port = GPIOC,
	.dio.pin = GPIO_PIN_13,
	// debounce cfg
	.debounce.time = 100,
	.debounce.maximum = 10, //updated later
	.debounce.fl_input = 0,
	.debounce.integrator = 0
};

digital_input_t Usr_Btn_1 =
{
	.dio.pmode = GPIO_MODE_INPUT,
	.dio.pull = GPIO_PULLDOWN,
	.dio.port = GPIOA,
	.dio.pin = GPIO_PIN_1,
	// debounce cfg
	.debounce.time = 100,
	.debounce.maximum = 10, //updated later
	.debounce.fl_input = 0,
	.debounce.integrator = 0		
};

digital_input_t Usr_Btn_2 =
{
	.dio.pmode = GPIO_MODE_INPUT,
	.dio.pull = GPIO_PULLDOWN,
	.dio.port = GPIOB,
	.dio.pin = GPIO_PIN_0,
	// debounce cfg
	.debounce.time = 100,
	.debounce.maximum = 10, //updated later
	.debounce.fl_input = 0,
	.debounce.integrator = 0		
};


/* Exported variables ---------------------------------------------------------*/
ptr_digital_input_t pOnboard_Btn = &Onboard_Btn;
ptr_digital_input_t pUsr_Btn_1 = &Usr_Btn_1;
ptr_digital_input_t pUsr_Btn_2 = &Usr_Btn_2;

	
/* Exported functions --------------------------------------------------------*/
/**
  *
  */
void DigitalInput_Init(ptr_digital_input_t di)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.Pin   = di->dio.pin;
	GPIO_InitStructure.Mode  = di->dio.pmode;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pull  = di->dio.pull;

	if (di->dio.port == GPIOA)
	{
		__GPIOA_CLK_ENABLE();
	}
	else if (di->dio.port == GPIOB)
	{
		__GPIOB_CLK_ENABLE();
	}
	else if (di->dio.port == GPIOC)
	{
		__GPIOC_CLK_ENABLE();
	}
	else if (di->dio.port == GPIOD)
	{
		__GPIOD_CLK_ENABLE();
	}
	else if (di->dio.port == GPIOE)
	{
		__GPIOE_CLK_ENABLE();
	}
	else if (di->dio.port == GPIOH)
	{
		__GPIOH_CLK_ENABLE();
	}
	else
	{
	}
	
	HAL_GPIO_Init(di->dio.port, &GPIO_InitStructure);
}

/**
  * Read raw value of the digital input (pin)
  */
GPIO_PinState DigitalInput_ReadPin(ptr_digital_input_t di)
{
	return HAL_GPIO_ReadPin(di->dio.port, di->dio.pin);
}

/**
  * Debounce given input (pin)
  */
void DigitalInput_DebouncePin(ptr_digital_input_t di)
{
	//debounce algorithm is based on code written by Kenneth A.Kuhn (http://www.kennethkuhn.com/electronics/debounce.c)
	
	/* Step 1: Update the integrator based on the input signal. 
       Integrator follows the input, decreasing or increasing towards the limits as 
       determined by the input state (0 or 1). */
	if (DigitalInput_ReadPin(di) == 0)
	{
		if (di->debounce.integrator > 0)
		{
			di->debounce.integrator = di->debounce.integrator - 1;
		}
		// else - don't substract, sice the integrator is at minimum
	}
	else
	{
		if (di->debounce.integrator < di->debounce.maximum)
		{
			di->debounce.integrator = di->debounce.integrator + 1;
		}
		// else - integrator is at maximum, don't add more
	}
	
	/* Step 2: Update the output state based on the integrator.
       The output (fl_input) will only change states if the integrator has reached a limit, either
       0 or MAXIMUM. */
	if (di->debounce.integrator == 0)
	{
		di->debounce.fl_input = 0;
	}
	else if (di->debounce.integrator >= di->debounce.maximum)
	{
		di->debounce.fl_input = 1;
		di->debounce.integrator = di->debounce.maximum;
	}
}
