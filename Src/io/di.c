#include "di.h"


digital_io_t LED_1 =
{
	.pmode = GPIO_MODE_OUTPUT_PP,
	.pull = GPIO_NOPULL,
	.port = GPIOA,
	.pin = GPIO_PIN_0	
};

p_digital_io_t pLED_1 = &LED_1;


void di_Init(digital_io_t * dio)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.Pin   = dio->pin;
	GPIO_InitStructure.Mode  = dio->pmode;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pull  = dio->pull;

	if (dio->port == GPIOA)
	{
		__GPIOA_CLK_ENABLE();
	}
	else if (dio->port == GPIOB)
	{
		__GPIOB_CLK_ENABLE();
	}
	else if (dio->port == GPIOC)
	{
		__GPIOC_CLK_ENABLE();
	}
	else if (dio->port == GPIOD)
	{
		__GPIOD_CLK_ENABLE();
	}
	else if (dio->port == GPIOE)
	{
		__GPIOE_CLK_ENABLE();
	}
	else if (dio->port == GPIOH)
	{
		__GPIOH_CLK_ENABLE();
	}
	else
	{
	}
	
	HAL_GPIO_Init(dio->port, &GPIO_InitStructure);
}

void di_Toggle_DigitalOutput(digital_io_t * dio)
{
	HAL_GPIO_TogglePin(dio->port, dio->pin);
}

