/**
  ******************************************************************************
  * @file    Multi/Examples/MotionControl/IHM01A1_ExampleFor1Motor/Src/main.c 
  * @author  IPC Rennes
  * @version V1.5.0
  * @date    November 12, 2014
  * @brief   This example shows how to use 1 IHM01A1 expansion board
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <../CMSIS_RTOS/cmsis_os.h>
#include "main.h"
#include "uart2.h"
#include "stepper_ctrl.h"
#include "cmd.h"
#include "io/do.h"
#include "io/di.h"

/** @defgroup IHM01A1_Example_for_1_motor_device
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define CMD_BUF_SIZE 32
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
 static volatile uint16_t gLastError;
 /* UART handler declaration */
 UART_HandleTypeDef UartHandle;

 // Reusable command buffer
 static char cmdBuf[CMD_BUF_SIZE];
 // Store actual time in system ticks
 uint32_t currentTime;
// Store actual position of the axis
 static int32_t actPos;


 static GPIO_PinState btnOldVal;

/* Private function prototypes -----------------------------------------------*/
static void MyFlagInterruptHandler(void);
void Init_User_GPIO(void);
static void LED_Thread1(void *argument);
static void Motor_Controller(void *argument);
static void Serial_Comm(void *argument);
static void DI_Scan(void *argument);

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	/* STM32xx HAL library initialization */
	HAL_Init();
  
	/* Configure the system clock */
	SystemClock_Config();

	DigitalOutput_Init(pAlarm_LED);

	////////////////////////////////////////
	// Thread creation
	xTaskCreate(LED_Thread1, "LED1", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(Motor_Controller, "MC", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(Serial_Comm, "SC", 256, NULL, 1, NULL);
	xTaskCreate(DI_Scan, "DIS", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	//printf("StartScheduler()\n");
	/* Start scheduler */
	vTaskStartScheduler();	
	///////////////////////////////////////

    // old code moved in function void originalTestCode(void)

//   volatile uint32_t param = BSP_MotorControl_CmdGetParam(0, L6474_TVAL);

    btnOldVal = BSP_PB_GetState(BUTTON_USER);

    /* Infinite loop */
    while(1)
    {
		// Get actual time (in miliseconds)
		currentTime = HAL_GetTick();

		////////////////////////////////////////////////////////////////////
		// Button (blue button on Nucleo401RE) - press detection and logics
		////////////////////////////////////////////////////////////////////
		static uint32_t btnCnt = 0;

		if ( (BSP_PB_GetState(BUTTON_USER) == GPIO_PIN_RESET) &&
			(btnOldVal == GPIO_PIN_SET) )
		{
			btnCnt++;
		}
		btnOldVal = BSP_PB_GetState(BUTTON_USER);
		////////////////////////////////////////////////////////////////////

		//////////////////////////////////////////////////////////////
		// Test sequence - motor control
		//////////////////////////////////////////////////////////////
		if (mcDriverReady == mcGetDriverStatus())
		{
			// Return motor Actual Position in variable "actPos", command is "mc_GetPos &actPos"
			snprintf(cmdBuf, CMD_BUF_SIZE, "mc_GetPos %p", &actPos);
			ExecuteCommand(cmdBuf);

			if(btnCnt == 1)
			{
				ExecuteCommand("mc_Run FW");
				//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
			}
		}
		else if (mcDriveJogging == mcGetDriverStatus())
		{
			if(btnCnt == 2)
			{
				ExecuteCommand("mc_Stop Soft");
				//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
			}
		}

		if (btnCnt >= 2) {btnCnt = 0;}
		//////////////////////////////////////////////////////////////

		//	   uart2_Transmit(string, sizeof(string));
	}
}

/**
  * @brief  This function is the User handler for the flag interrupt
  * @param  None
  * @retval None
  */
void MyFlagInterruptHandler(void)
{
	/* Get the value of the status register via the L6474 command GET_STATUS */
	uint16_t statusRegister = BSP_MotorControl_CmdGetStatus(0);
  
	/* Check HIZ flag: if set, power brigdes are disabled */
	if ((statusRegister & L6474_STATUS_HIZ) == L6474_STATUS_HIZ)
	{
	// HIZ state
	// Action to be customized            
	}

	/* Check direction bit */
	if ((statusRegister & L6474_STATUS_DIR) == L6474_STATUS_DIR)
	{
	// Forward direction is set
	// Action to be customized            
	}  
	else
	{
	// Backward direction is set
	// Action to be customized            
	}  

	/* Check NOTPERF_CMD flag: if set, the command received by SPI can't be performed */
	/* This often occures when a command is sent to the L6474 */
	/* while it is in HIZ state */
	if ((statusRegister & L6474_STATUS_NOTPERF_CMD) == L6474_STATUS_NOTPERF_CMD)
	{
		// Command received by SPI can't be performed
		// Action to be customized            
	}  

	/* Check WRONG_CMD flag: if set, the command does not exist */
	if ((statusRegister & L6474_STATUS_WRONG_CMD) == L6474_STATUS_WRONG_CMD)
	{
		//command received by SPI does not exist 
		// Action to be customized          
	}  

	/* Check UVLO flag: if not set, there is an undervoltage lock-out */
	if ((statusRegister & L6474_STATUS_UVLO) == 0)
	{
		//undervoltage lock-out 
		// Action to be customized          
	}  

	/* Check TH_WRN flag: if not set, the thermal warning threshold is reached */
	if ((statusRegister & L6474_STATUS_TH_WRN) == 0)
	{
	//thermal warning threshold is reached
	// Action to be customized          
	}    

	/* Check TH_SHD flag: if not set, the thermal shut down threshold is reached */
	if ((statusRegister & L6474_STATUS_TH_SD) == 0)
	{
	//thermal shut down threshold is reached 
	// Action to be customized          
	}    

	/* Check OCD  flag: if not set, there is an overcurrent detection */
	if ((statusRegister & L6474_STATUS_OCD) == 0)
	{
	//overcurrent detection 
	// Action to be customized          
	}      
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  error number of the error
  * @retval None
  */
void Error_Handler(uint16_t error)
{
  /* Backup error number */
  gLastError = error;
  
  /* Infinite loop */
  while(1)
  {
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  *
  */
void originalTestCode(void)
{
	int32_t pos;
	uint16_t mySpeed;
	//----- Move of 16000 steps in the FW direction

	  /* Move device 0 of 16000 steps in the FORWARD direction*/
	  BSP_MotorControl_Move(0, FORWARD, 16000);

	  /* Wait for the motor of device 0 ends moving */
	  BSP_MotorControl_WaitWhileActive(0);

	  /* Wait for 2 seconds */
	  HAL_Delay(2000);

	//----- Move of 16000 steps in the BW direction

	  /* Move device 0 of 16000 steps in the BACKWARD direction*/
	  BSP_MotorControl_Move(0, BACKWARD, 16000);

	  /* Wait for the motor of device 0 ends moving */
	  BSP_MotorControl_WaitWhileActive(0);

	   /* Set the current position of device 0 to be the Home position */
	  BSP_MotorControl_SetHome(0);

	  /* Wait for 2 seconds */
	  HAL_Delay(2000);

	//----- Go to position -6400

	  /* Request device 0 to go to position -6400 */
	  BSP_MotorControl_GoTo(0,-6400);

	  /* Wait for the motor ends moving */
	  BSP_MotorControl_WaitWhileActive(0);

	  /* Get current position of device 0*/
	  pos = BSP_MotorControl_GetPosition(0);

	  if (pos != -6400)
	  {
	    Error_Handler(11);
	  }

	  /* Set the current position of device 0 to be the Mark position */
	  BSP_MotorControl_SetMark(0);

	  /* Wait for 2 seconds */
	  HAL_Delay(2000);

	//----- Go Home

	  /* Request device 0 to go to Home */
	  BSP_MotorControl_GoHome(0);
	  BSP_MotorControl_WaitWhileActive(0);

	  /* Get current position of device 0 */
	  pos = BSP_MotorControl_GetPosition(0);

	  /* Wait for 2 seconds */
	  HAL_Delay(2000);

	//----- Go to position 6400

	  /* Request device 0 to go to position 6400 */
	  BSP_MotorControl_GoTo(0,6400);

	  /* Wait for the motor of device 0 ends moving */
	  BSP_MotorControl_WaitWhileActive(0);

	  /* Get current position of device 0*/
	  pos = BSP_MotorControl_GetPosition(0);

	  /* Wait for 2 seconds */
	  HAL_Delay(2000);

	//----- Go Mark which was set previously after go to -6400

	  /* Request device 0 to go to Mark position */
	  BSP_MotorControl_GoMark(0);

	  /* Wait for the motor of device 0 ends moving */
	  BSP_MotorControl_WaitWhileActive(0);

	  /* Get current position of device 0 */
	  pos = BSP_MotorControl_GetPosition(0);

	  /* Wait for 2 seconds */
	  HAL_Delay(2000);

	//----- Run the motor BACKWARD

	  /* Request device 0 to run BACKWARD */
	   BSP_MotorControl_Run(0,BACKWARD);
	   HAL_Delay(5000);

	   /* Get current speed of device 0 */
	   mySpeed = BSP_MotorControl_GetCurrentSpeed(0);

	//----- Increase the speed while running

	  /* Increase speed of device 0 to 2400 step/s */
	  BSP_MotorControl_SetMaxSpeed(0,2400);
	  HAL_Delay(5000);

	   /* Get current speed of device 0 */
	   mySpeed = BSP_MotorControl_GetCurrentSpeed(0);

	//----- Decrease the speed while running

	  /* Decrease speed of device 0 to 1200 step/s */
	  BSP_MotorControl_SetMaxSpeed(0,1200);
	  HAL_Delay(5000);

	  /* Get current speed */
	  mySpeed = BSP_MotorControl_GetCurrentSpeed(0);

	//----- Increase acceleration while running

	  /* Increase acceleration of device 0 to 480 step/s^2 */
	  BSP_MotorControl_SetAcceleration(0,480);
	  HAL_Delay(5000);

	  /* Increase speed of device 0 to 2400 step/s */
	  BSP_MotorControl_SetMaxSpeed(0,2400);
	  HAL_Delay(5000);

	  /* Get current speed of device 0 */
	  mySpeed = BSP_MotorControl_GetCurrentSpeed(0);

	  if (mySpeed != 2400)
	  {
	    Error_Handler(10);
	  }
	//----- Increase deceleration while running

	  /* Increase deceleration of device 0 to 480 step/s^2 */
	  BSP_MotorControl_SetDeceleration(0,480);
	  HAL_Delay(5000);

	  /* Decrease speed of device 0 to 1200 step/s */
	  BSP_MotorControl_SetMaxSpeed(0,1200);
	  HAL_Delay(5000);

	  /* Get current speed */
	  mySpeed = BSP_MotorControl_GetCurrentSpeed(0);

	//----- Soft stopped required while running

	  /* Request soft stop of device 0 */
	  BSP_MotorControl_SoftStop(0);

	  /* Wait for the motor of device 0 ends moving */
	 BSP_MotorControl_WaitWhileActive(0);

	  /* Wait for 2 seconds */
	  HAL_Delay(2000);

	//----- Run stopped by hardstop

	  /* Request device 0 to run in FORWARD direction */
	  BSP_MotorControl_Run(0,FORWARD);
	  HAL_Delay(5000);

	  /* Request device 0 to immediatly stop */
	  BSP_MotorControl_HardStop(0);
	  BSP_MotorControl_WaitWhileActive(0);

	  /* Wait for 2 seconds */
	  HAL_Delay(2000);

	//----- GOTO stopped by softstop

	 /* Request device 0 to go to position 20000  */
	  BSP_MotorControl_GoTo(0,20000);
	  HAL_Delay(5000);

	  /* Request device 0 to perform a soft stop */
	  BSP_MotorControl_SoftStop(0);
	  BSP_MotorControl_WaitWhileActive(0);

	  /* Wait for 2 seconds */
	  HAL_Delay(2000);

	  //----- Read inexistent register to test MyFlagInterruptHandler

	  /* Try to read an inexistent register */
	  /* the flag interrupt should be raised */
	  /* and the MyFlagInterruptHandler function called */
	  BSP_MotorControl_CmdGetParam(0,0x1F);
	  HAL_Delay(500);

	//----- Change step mode to full step mode

	  /* Select full step mode for device 0 */
	   BSP_MotorControl_SelectStepMode(0,STEP_MODE_FULL);

	  /* Set speed and acceleration to be consistent with full step mode */
	   BSP_MotorControl_SetMaxSpeed(0,100);
	   BSP_MotorControl_SetMinSpeed(0,50);
	   BSP_MotorControl_SetAcceleration(0,10);
	   BSP_MotorControl_SetDeceleration(0,10);

	  /* Request device 0 to go position 200 */
	   BSP_MotorControl_GoTo(0,200);

	  /* Wait for the motor of device 0 ends moving */
	   BSP_MotorControl_WaitWhileActive(0);

	  /* Get current position */
	  pos =  BSP_MotorControl_GetPosition(0);

	  /* Wait for 2 seconds */
	  HAL_Delay(2000);
}

/**
  * 
  */
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
	while (1)
		;
}

/**
  * @brief  Toggle LED1
  * @param  thread not used
  * @retval None
  */
static void LED_Thread1(void *argument)
{
	(void) argument;
	portTickType xLastWakeTime, dly;
	
	xLastWakeTime = xTaskGetTickCount();
	
	for (;;)
	{
		if( (pOnboard_Btn->debounce.fl_input == 0) ||
			(pUsr_Btn_1->debounce.fl_input == 0) ||
			(pUsr_Btn_2->debounce.fl_input == 0) )
		{
			dly = (500 / portTICK_RATE_MS);
		}
		else
		{
			dly = (1000 / portTICK_RATE_MS);
		}
		DigitalOutput_Toggle(pAlarm_LED);
		
		vTaskDelayUntil(&xLastWakeTime, dly);
	}
}

/**
  * Motor controll thread
  */
static void Motor_Controller(void *argument)
{
	portTickType xLastWakeTime;
	
    //----- Init of the Motor control library 
    /* Start the L6474 library to use 1 device */
    /* The L6474 registers are set with the predefined values */
    /* from file l6474_target_config.h*/
	BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_L6474, 1);
  
	/* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
	BSP_MotorControl_AttachFlagInterrupt(MyFlagInterruptHandler);

	  /* Attach the function Error_Handler (defined below) to the error Handler*/
	BSP_MotorControl_AttachErrorHandler(Error_Handler);	
	
    // Should be after BSP_MotorControl_Init() because reads data from motor driver
	mcInit();
	
	  /* Reset device 0 to 1/x microstepping mode */
	BSP_MotorControl_SelectStepMode(0, STEP_MODE_1_8);

	/* Update speed, acceleration, deceleration for 1/x microstepping mode*/
	BSP_MotorControl_SetMaxSpeed(0, 3600);
	BSP_MotorControl_SetMinSpeed(0, 100);
	BSP_MotorControl_SetAcceleration(0, 1600);
	BSP_MotorControl_SetDeceleration(0, 1600);
	
	ExecuteCommand("mc_Run FW");
	xLastWakeTime = xTaskGetTickCount();
	
	for (;;)
	{
		if (pUsr_Btn_1->debounce.fl_input == 0)
		{
			ExecuteCommand("mc_Run FW");
		}
		else if (pUsr_Btn_2->debounce.fl_input == 0)
		{
			ExecuteCommand("mc_Run BW");
		}
		else
		{
			ExecuteCommand("mc_Stop Soft");
		}
		
		mcRecurrentFnc(0);
		vTaskDelayUntil(&xLastWakeTime, (200 / portTICK_RATE_MS));
	}	
}

/**
  * Serial communication thread
  */
static void Serial_Comm(void *argument)
{
	portTickType xLastWakeTime = 0;
	int32_t act_pos = 0;
	int32_t cx = 0;
	char str[16] = {0};
	
	uart2_Init();
	
	xLastWakeTime = xTaskGetTickCount();
	
	for (;;)
	{
        act_pos = mc_Get_MotorPosition();
		
		memset(str, 0, sizeof(str));
		cx = snprintf(str, sizeof(str), "%i ", act_pos);
		uart2_Transmit((uint8_t *)str, cx);
		
		vTaskDelayUntil(&xLastWakeTime, (500 / portTICK_RATE_MS));
	}
}

/**
  * Digital inputs scanner
  */
static void DI_Scan(void *argument)
{
	portTickType xLastWakeTime = 0;
	portTickType scan_period = 10;  // Scan period in ms
	
	// Initialize inputs
	DigitalInput_Init(pOnboard_Btn);
	DigitalInput_Init(pUsr_Btn_1);
	DigitalInput_Init(pUsr_Btn_2);
	
	// Compute maximum value. This value will be compared with the integrator in filter code
	pOnboard_Btn->debounce.maximum = pOnboard_Btn->debounce.time / scan_period;
	pUsr_Btn_1->debounce.maximum = pUsr_Btn_1->debounce.time / scan_period;
	pUsr_Btn_2->debounce.maximum = pUsr_Btn_2->debounce.time / scan_period;
	
	xLastWakeTime = xTaskGetTickCount();
	
	for (;;)
	{

		DigitalInput_DebouncePin(pUsr_Btn_1);
		DigitalInput_DebouncePin(pUsr_Btn_2);
		DigitalInput_DebouncePin(pOnboard_Btn);
		vTaskDelayUntil(&xLastWakeTime, (scan_period / portTICK_RATE_MS));
	}
}

/**** END OF FILE ****/
