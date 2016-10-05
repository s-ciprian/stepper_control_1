/* Includes ------------------------------------------------------------------*/
#include "do.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/


/* Private variables ---------------------------------------------------------*/
digital_output_t Alarm_LED =
{
	.dio.pmode = GPIO_MODE_OUTPUT_PP,
	.dio.pull  = GPIO_NOPULL,
	.dio.port  = GPIOA,
	.dio.pin   = GPIO_PIN_0	
};

digital_output_t Usr_Out_1 =
{
	.dio.pmode = GPIO_MODE_OUTPUT_PP,
	.dio.pull = GPIO_NOPULL,
	.dio.port = GPIOC,
	.dio.pin = GPIO_PIN_3
};


/* Exported variables ---------------------------------------------------------*/
ptr_digital_output_t pAlarm_LED = &Alarm_LED;
ptr_digital_output_t pUsr_Out_1 = &Usr_Out_1;

/* Exported functions --------------------------------------------------------*/
/**
  *
  */
void DigitalOutput_Init(ptr_digital_output_t dq)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.Pin   = dq->dio.pin;
	GPIO_InitStructure.Mode  = dq->dio.pmode;
	GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
	GPIO_InitStructure.Pull  = dq->dio.pull;

	if (dq->dio.port == GPIOA)
	{
		__GPIOA_CLK_ENABLE();
	}
	else if (dq->dio.port == GPIOB)
	{
		__GPIOB_CLK_ENABLE();
	}
	else if (dq->dio.port == GPIOC)
	{
		__GPIOC_CLK_ENABLE();
	}
	else if (dq->dio.port == GPIOD)
	{
		__GPIOD_CLK_ENABLE();
	}
	else if (dq->dio.port == GPIOE)
	{
		__GPIOE_CLK_ENABLE();
	}
	else if (dq->dio.port == GPIOH)
	{
		__GPIOH_CLK_ENABLE();
	}
	else
	{
	}
	
	HAL_GPIO_Init(dq->dio.port, &GPIO_InitStructure);
}

/**
  *
  */
void DigitalOutput_Toggle(ptr_digital_output_t dq)
{
	HAL_GPIO_TogglePin(dq->dio.port, dq->dio.pin);
}

/**
  *
  */
void DigitalOutput_SetHigh(ptr_digital_output_t dq)
{
	HAL_GPIO_WritePin(dq->dio.port, dq->dio.pin, GPIO_PIN_SET);
}

/**
  *
  */
void DigitalOutput_SetLow(ptr_digital_output_t dq)
{
	HAL_GPIO_WritePin(dq->dio.port, dq->dio.pin, GPIO_PIN_RESET);
}

/* Private functions ---------------------------------------------------------*/
/**
  *
  */
