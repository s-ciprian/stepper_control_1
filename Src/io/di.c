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
	.dio.pin = GPIO_PIN_13	
};

/* Exported variables ---------------------------------------------------------*/
ptr_digital_input_t pOnboard_Btn = &Onboard_Btn;

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
  *
  */
GPIO_PinState DigitalInput_ReadPin(ptr_digital_input_t di)
{
	return HAL_GPIO_ReadPin(di->dio.port, di->dio.pin);
}

