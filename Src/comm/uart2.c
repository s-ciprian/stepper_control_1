/*! \file
 * \details This file  implements functions to control UART2 peripheral.
 */
 
/* Includes ------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/ 
#include "main.h"
#include "uart2.h"

/* Private typedef -----------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/* Definition for USART 2 Pins */    
#define USART2_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USART2_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define USART2_TX_PIN                    GPIO_PIN_2
#define USART2_TX_GPIO_PORT              GPIOA
#define USART2_TX_AF                     GPIO_AF7_USART2
#define USART2_RX_PIN                    GPIO_PIN_3
#define USART2_RX_GPIO_PORT              GPIOA
#define USART2_RX_AF                     GPIO_AF7_USART2


/* Private macro -------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/** \brief UART handler declaration */
 UART_HandleTypeDef hUart_2;

/* Private function prototypes -----------------------------------------------*/
/*----------------------------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/**
  * \brief Configure UART 2 peripheral
  * \param None
  * \retval None
  *
  * TODO: Add an API to retrive the address of hUart_2, then pass
  *       this address to this function. If changes are nedded to
  *       default configuration this can be done.
  */
void uart2_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	/*##-1- Enable peripherals and GPIO Clocks #################################*/
	/* Enable GPIO TX/RX clock */
	USART2_TX_GPIO_CLK_ENABLE();
	USART2_RX_GPIO_CLK_ENABLE();

	/* Enable USART 2 clock */
	__HAL_RCC_USART2_CLK_ENABLE();

	/*##-2- Configure peripheral GPIO ##########################################*/
	/* UART TX GPIO pin configuration  */
	GPIO_InitStruct.Pin       = USART2_TX_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_PULLUP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = USART2_TX_AF;

	HAL_GPIO_Init(USART2_TX_GPIO_PORT, &GPIO_InitStruct);

	/* UART RX GPIO pin configuration  */
	GPIO_InitStruct.Pin = USART2_RX_PIN;
	GPIO_InitStruct.Alternate = USART2_RX_AF;

	HAL_GPIO_Init(USART2_RX_GPIO_PORT, &GPIO_InitStruct);

	/* UART2 configured as follow:
	  - Word Length = 9 Bits
	  - Stop Bit = One Stop bit
	  - Parity = NO parity
	  - BaudRate = 9600 baud
	  - Hardware flow control disabled (RTS and CTS signals) */
	hUart_2.Instance          = USART2;

	hUart_2.Init.BaudRate     = 9600;
	hUart_2.Init.WordLength   = UART_WORDLENGTH_8B;
	hUart_2.Init.StopBits     = UART_STOPBITS_1;
	hUart_2.Init.Parity       = UART_PARITY_NONE;
	hUart_2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	hUart_2.Init.Mode         = UART_MODE_TX_RX;
	hUart_2.Init.OverSampling = UART_OVERSAMPLING_16;

	if(HAL_UART_Init(&hUart_2) != HAL_OK)
	{
		/* Initialization Error */
		Error_Handler(1002);
	}
}

/**
  * \brief Disable UART 2 peripheral
  * \param None
  * \retval None
  *
  *
  */
void uart2_DeInit(void)
{
	/*##-1- Reset peripherals ##################################################*/
	__HAL_RCC_USART2_FORCE_RESET();
	__HAL_RCC_USART2_RELEASE_RESET();

	/*##-2- Disable peripherals and GPIO Clocks #################################*/
	/* Configure UART Tx as alternate function  */
	HAL_GPIO_DeInit(USART2_TX_GPIO_PORT, USART2_TX_PIN);
	/* Configure UART Rx as alternate function  */
	HAL_GPIO_DeInit(USART2_RX_GPIO_PORT, USART2_RX_PIN);
}

/**
  * \brief UART 2 transmit function
  * \param pointer to buffer to be sent, size of the buffer
  * \retval None
  */
void uart2_Transmit(uint8_t *pData, uint16_t Size)
{
	HAL_UART_Transmit(&hUart_2, pData, Size, 0xFFFF);
}

/* Private functions ---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

