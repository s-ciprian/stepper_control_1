/* check if my teammate  see this
   hello test issue 2
 */
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
//#include "uart2.h"
#include "stepper_ctrl.h"
#include "cmd.h"
#include "io/do.h"
#include "io/di.h"
#include "hc_app/hca.h"
#include "lcd/nhd_0216.h"

///////////////////////////////////////////////////////////////////////////////
// Private typedef
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// Private define
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// Private macro
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// Private variables
///////////////////////////////////////////////////////////////////////////////
static volatile uint16_t gLastError;
/* UART handler declaration */
UART_HandleTypeDef UartHandle;


///////////////////////////////////////////////////////////////////////////////
// Private function prototypes
///////////////////////////////////////////////////////////////////////////////
// Tasks
static void LED_Thread1(void *argument);
static void Motor_Controller(void *argument);
static void Serial_Comm(void *argument);
static void DI_Scan(void *argument);
static void Heat_Controller_App(void *argument);
// Not used - To Be Deleted
void originalTestCode(void);


///////////////////////////////////////////////////////////////////////////////
// Public functions - implementation
///////////////////////////////////////////////////////////////////////////////

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
	DigitalOutput_Init(pUsr_Out_1);

	////////////////////////////////////////
	// Thread creation
    //xTaskCreate(DI_Scan, "DIS", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(LED_Thread1, "LED1", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(Motor_Controller, "MC", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(Serial_Comm, "SC", 256, NULL, 1, NULL);
    xTaskCreate(DI_Scan, "DIS", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
    xTaskCreate(Heat_Controller_App, "HCA", configMINIMAL_STACK_SIZE, NULL, 1, NULL); 

	//printf("StartScheduler()\n");
	/* Start scheduler */
	vTaskStartScheduler();	
	///////////////////////////////////////

    //   volatile uint32_t param = BSP_MotorControl_CmdGetParam(0, L6474_TVAL);
	//btnOldVal = BSP_PB_GetState(BUTTON_USER);

	    /* Infinite loop */
	while (1)
	{
		// How to return motor Actual Position in variable "actPos" using command "mc_GetPos &actPos"
	    // Position could be get directly using stepper_ctrl_Get_Actual_Position
		//// snprintf(cmdBuf, 32, "mc_GetPos %p", &actPos);  // actPos is signed 32 bits
		//// ExecuteCommand(cmdBuf);
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
	while (1)
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


void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
	while (1)
		;
}


///////////////////////////////////////////////////////////////////////////////
// Private functions - implementation
///////////////////////////////////////////////////////////////////////////////

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
	BSP_MotorControl_GoTo(0, -6400);

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
	BSP_MotorControl_GoTo(0, 6400);

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
	BSP_MotorControl_Run(0, BACKWARD);
	HAL_Delay(5000);

		   /* Get current speed of device 0 */
	mySpeed = BSP_MotorControl_GetCurrentSpeed(0);

		//----- Increase the speed while running

			  /* Increase speed of device 0 to 2400 step/s */
	BSP_MotorControl_SetMaxSpeed(0, 2400);
	HAL_Delay(5000);

		   /* Get current speed of device 0 */
	mySpeed = BSP_MotorControl_GetCurrentSpeed(0);

		//----- Decrease the speed while running

			  /* Decrease speed of device 0 to 1200 step/s */
	BSP_MotorControl_SetMaxSpeed(0, 1200);
	HAL_Delay(5000);

		  /* Get current speed */
	mySpeed = BSP_MotorControl_GetCurrentSpeed(0);

		//----- Increase acceleration while running

			  /* Increase acceleration of device 0 to 480 step/s^2 */
	BSP_MotorControl_SetAcceleration(0, 480);
	HAL_Delay(5000);

		  /* Increase speed of device 0 to 2400 step/s */
	BSP_MotorControl_SetMaxSpeed(0, 2400);
	HAL_Delay(5000);

		  /* Get current speed of device 0 */
	mySpeed = BSP_MotorControl_GetCurrentSpeed(0);

	if (mySpeed != 2400)
	{
		Error_Handler(10);
	}
  //----- Increase deceleration while running

  	  /* Increase deceleration of device 0 to 480 step/s^2 */
	BSP_MotorControl_SetDeceleration(0, 480);
	HAL_Delay(5000);

		  /* Decrease speed of device 0 to 1200 step/s */
	BSP_MotorControl_SetMaxSpeed(0, 1200);
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
	BSP_MotorControl_Run(0, FORWARD);
	HAL_Delay(5000);

		  /* Request device 0 to immediatly stop */
	BSP_MotorControl_HardStop(0);
	BSP_MotorControl_WaitWhileActive(0);

		  /* Wait for 2 seconds */
	HAL_Delay(2000);

		//----- GOTO stopped by softstop

			 /* Request device 0 to go to position 20000  */
	BSP_MotorControl_GoTo(0, 20000);
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
	BSP_MotorControl_CmdGetParam(0, 0x1F);
	HAL_Delay(500);

		//----- Change step mode to full step mode

			  /* Select full step mode for device 0 */
	BSP_MotorControl_SelectStepMode(0, STEP_MODE_FULL);

		  /* Set speed and acceleration to be consistent with full step mode */
	BSP_MotorControl_SetMaxSpeed(0, 100);
	BSP_MotorControl_SetMinSpeed(0, 50);
	BSP_MotorControl_SetAcceleration(0, 10);
	BSP_MotorControl_SetDeceleration(0, 10);

		  /* Request device 0 to go position 200 */
	BSP_MotorControl_GoTo(0, 200);

		  /* Wait for the motor of device 0 ends moving */
	BSP_MotorControl_WaitWhileActive(0);

		  /* Get current position */
	pos =  BSP_MotorControl_GetPosition(0);

		  /* Wait for 2 seconds */
	HAL_Delay(2000);
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
		dly = (1000 / portTICK_RATE_MS);
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

	mcInit();
	
	xLastWakeTime = xTaskGetTickCount();

	
	for (;;)
	{
        stepper_ctrl_Begin();
		stepper_ctrl_ProcessEvent();
        stepper_ctrl_End();

		vTaskDelayUntil(&xLastWakeTime, (20 / portTICK_RATE_MS));
	}	
}

/**
  * Serial communication thread
  */
static void Serial_Comm(void *argument)
{
	portTickType xLastWakeTime = 0;
    int16_t lcd_init_done_flag = 0;
	int32_t act_pos = 0;
	int32_t cx = 0;
	uint16_t statusRegister;
	char str[20] = { 0 };
	
//	uart2_Init();
	
	xLastWakeTime = xTaskGetTickCount();
	
	for (;;)
	{
        // Init the LCD
        if (!lcd_init_done_flag)
        {
            // Call LCD Init function, pass a pointer to the function used to send
            // data over serial.
            // LCD Init process can take some time, when done set the flag.
             if(lcd_Init() == 0)
             {
                lcd_init_done_flag = 1;
             }
        }

		act_pos = stepper_ctrl_Get_Actual_Position();
		/* Get the value of the status register of the L6474 */
		statusRegister = BSP_MotorControl_CmdGetStatus(0);
		
		// Clear string
		memset(str, 0, sizeof(str));
		// Format and send string containing status register of motor driver and actual motor position
		cx = snprintf(str, sizeof(str), "%i, %u\r\n", act_pos, statusRegister);
//        uart2_Transmit((uint8_t *)str, cx);

        if (lcd_init_done_flag) // Don't send data until LCD is initialized
        {           
            lcd_Put_Act_Pos();
        }
		
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
	DigitalInput_Init(pUsr_Btn_3);
	DigitalInput_Init(pLimit_SW_Minus);
	DigitalInput_Init(pLimit_SW_Plus);
	
	// Compute maximum value. This value will be compared with the integrator in filter code
	pOnboard_Btn->debounce.maximum = (pOnboard_Btn->debounce.time / scan_period);
	pUsr_Btn_1->debounce.maximum = (pUsr_Btn_1->debounce.time / scan_period);
	pUsr_Btn_2->debounce.maximum = (pUsr_Btn_2->debounce.time / scan_period);
	pUsr_Btn_3->debounce.maximum = (pUsr_Btn_3->debounce.time / scan_period);
	pLimit_SW_Minus->debounce.maximum = (pLimit_SW_Minus->debounce.time / scan_period);
	pLimit_SW_Plus->debounce.maximum = (pLimit_SW_Plus->debounce.time / scan_period);
	
	// Set initial filtered value same as pin value. Only after Reset
	pUsr_Btn_1->debounce.fl_input = DigitalInput_ReadPin(pUsr_Btn_1);
	pUsr_Btn_2->debounce.fl_input = DigitalInput_ReadPin(pUsr_Btn_2);
	pUsr_Btn_3->debounce.fl_input = DigitalInput_ReadPin(pUsr_Btn_3);
	pOnboard_Btn->debounce.fl_input = DigitalInput_ReadPin(pOnboard_Btn);
	pLimit_SW_Minus->debounce.fl_input = DigitalInput_ReadPin(pLimit_SW_Minus);
	pLimit_SW_Plus->debounce.fl_input = DigitalInput_ReadPin(pLimit_SW_Plus);

	xLastWakeTime = xTaskGetTickCount();
	
	for (;;)
	{
		DigitalInput_DebouncePin(pUsr_Btn_1);
		DigitalInput_DebouncePin(pUsr_Btn_2);
		DigitalInput_DebouncePin(pUsr_Btn_3);
		DigitalInput_DebouncePin(pOnboard_Btn);
		DigitalInput_DebouncePin(pLimit_SW_Minus);
		DigitalInput_DebouncePin(pLimit_SW_Plus);
		
 
        // Test Input and Output
		if (pUsr_Btn_3->debounce.fl_input == 0)
		{
			DigitalOutput_SetLow(pUsr_Out_1);
		}
		else
		{
			DigitalOutput_SetHigh(pUsr_Out_1);
		}

		vTaskDelayUntil(&xLastWakeTime, (scan_period / portTICK_RATE_MS));
	}
}

/**
  * Heater Controller Application - top component of the application
  */
static void Heat_Controller_App(void *argument)
{
	portTickType xLastWakeTime = 0;
    // Scan period in ms.
    // Should be the same as for Motor_Controller else JOG motion is interrupted
    // because how data is passed between this task and Motor_Controller
	portTickType hca_scan_period = 20;

    hca_Init();


	for (;;)
	{
        hca_Begin();
        hca_ProcessEvent();
        hca_End();

        // TODO: Add here function for this task
        vTaskDelayUntil(&xLastWakeTime, (hca_scan_period / portTICK_RATE_MS));
    }
}

/**** END OF FILE ****/
