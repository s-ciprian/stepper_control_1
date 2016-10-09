
#include "limits.h"
#include "stm32f4xx_nucleo.h"
#include "motorcontrol.h"
#include "stepper_ctrl.h"
#include "../io/di.h"

#include "main.h"

//Motor controller chip - definitions
#include "..\Components\l6474\l6474.h"
#ifdef USE_STM32F4XX_NUCLEO
#include "stm32f4xx_nucleo_ihm01a1.h"
#endif

///////////////////////////////////////////////////////////////////////////////
// Types definitions
///////////////////////////////////////////////////////////////////////////////
typedef enum _mcState_t
{
	Idle,
	Movement_Positioning_Absolute,
	Movement_Positioning_Relative,
	Run_Jog_Positive,
    Run_Jog_Negative,
	Wait_Standstill,
	Parametrization,
	ReadData
} mcState_t;

typedef struct _mcAxis_t
{
	uint8_t id;
	int32_t act_pos;
} mcAxis_t;


///////////////////////////////////////////////////////////////////////////////
// Private define
///////////////////////////////////////////////////////////////////////////////
#define POSITION_SETPOINT_INVALID    INT_MAX


///////////////////////////////////////////////////////////////////////////////
// Private functions protptype
///////////////////////////////////////////////////////////////////////////////
static void MyFlagInterruptHandler(void);

// Function prototypes for module commands
static int32_t process_mc_GoTo(int argc, char *argv[]);
static int32_t process_mc_GetPos(int argc, char *argv[]);
static int32_t process_mc_HMI_JogP(int argc, char *argv[]);
static int32_t process_mc_HMI_JogN(int argc, char *argv[]);
static int32_t process_mc_Stop(int argc, char *argv[]);


///////////////////////////////////////////////////////////////////////////////
// Private variables
///////////////////////////////////////////////////////////////////////////////
static mcState_t mcState;
static mcAxis_t firstAxis;

static uint16_t hmi_jog_p, hmi_jog_n, hmi_jog_p_old, hmi_jog_n_old;
static uint16_t limit_p, limit_n;

static int32_t position_setpoint = POSITION_SETPOINT_INVALID;

///////////////////////////////////////////////////////////////////////////////
// Public functions - implementation
///////////////////////////////////////////////////////////////////////////////

//******************************************************************************
// Controller function
//******************************************************************************
void stepper_ctrl_ProcessEvent(void)
{
	switch (mcState) {
		case Idle:  // Check HMI inputs and other movement commands
            //======= Jog commands =======
            // If not on Positive limit switch and Jog positive input ON.
            // Check also Jog negative to avoid movement from limit to limit when both button are pressed
		    if (!limit_p && hmi_jog_p && !hmi_jog_n)
		    {
    		    BSP_MotorControl_Run(firstAxis.id, FORWARD);
                mcState = Run_Jog_Positive;
		    }
            // If not on Negative limit switch and Jog negative input ON
            // Check also Jog negative to avoid movement from limit to limit when both button are pressed
            else if (!limit_n && hmi_jog_n && !hmi_jog_p)
            {
    			BSP_MotorControl_Run(firstAxis.id, BACKWARD);
                mcState = Run_Jog_Negative;
            }
            // Absolute positioning command
		    else if (position_setpoint != POSITION_SETPOINT_INVALID)
		    {
                // Send new setpoint to motor driver
                BSP_MotorControl_GoTo(firstAxis.id, position_setpoint);
                // And invalidate setpoit variable
                position_setpoint = POSITION_SETPOINT_INVALID;
                // Chage state and wait for the end of the movement
                mcState = Movement_Positioning_Absolute;
		    }
            else
            {
                // TODO: Add - error, command not supported
            }
		    break;

		case Run_Jog_Positive:
            firstAxis.act_pos = BSP_MotorControl_GetPosition(firstAxis.id); /* Axis actual position */

            if (!hmi_jog_p || limit_p)  // "HMI input OFF" OR "Limit SW P activated"
            {
                BSP_MotorControl_HardStop(firstAxis.id);  // Stop motor
                mcState = Wait_Standstill;    // Wait for stop
            }
			break;

		case Run_Jog_Negative:
            firstAxis.act_pos = BSP_MotorControl_GetPosition(firstAxis.id); /* Axis actual position */

            if (!hmi_jog_n || limit_n)  // "HMI input OFF" OR "Limit SW N activated"
            {
                BSP_MotorControl_HardStop(firstAxis.id);  // Stop motor
                mcState = Wait_Standstill;    // Wait for stop
            }
			break;

		case Movement_Positioning_Absolute:
			// TODO: Stop command ?
            firstAxis.act_pos = BSP_MotorControl_GetPosition(firstAxis.id); /* Axis actual position */

		    if (limit_p || limit_n)
		    {
			    BSP_MotorControl_HardStop(firstAxis.id);
                mcState = Wait_Standstill;    // Wait for stop
		    }

			if (INACTIVE == BSP_MotorControl_GetDeviceState(firstAxis.id))
			{
				mcState = Idle;
			}
			break;

		case Wait_Standstill:
            firstAxis.act_pos = BSP_MotorControl_GetPosition(firstAxis.id); /* Axis actual position */
            
            // Stop during DECELERATION, case when hiting an limit switch while DECELERATING.
            // Then need a HARD STOP  
		    if (limit_p || limit_n)
		    {
			    BSP_MotorControl_HardStop(firstAxis.id);
		    }
		
			// Wait here until motor decelerating
			if (INACTIVE == BSP_MotorControl_GetDeviceState(firstAxis.id))
			{
				mcState = Idle;
			}
			break;

		default:
			mcState = Idle;
			// TODO: Add an error if this state is reached
			break;
	}
}

//******************************************************************************
// Initialization function
//
// This function will register commands supported by this module
//******************************************************************************
void mcInit(void)
{
	// Initialize state machine state
	mcState = Idle;

    hmi_jog_n = hmi_jog_n_old = !(DigitalInput_GetValue(pUsr_Btn_2));
	hmi_jog_p = hmi_jog_p_old = !(DigitalInput_GetValue(pUsr_Btn_1));

    limit_p = !(DigitalInput_GetValue(pLimit_SW_Plus));
    limit_n = !(DigitalInput_GetValue(pLimit_SW_Minus));

    position_setpoint = POSITION_SETPOINT_INVALID;

    //----- Init of the Motor control library 
    /* Start the L6474 library to use 1 device */
    /* The L6474 registers are set with the predefined values */
    /* from file l6474_target_config.h*/
	BSP_MotorControl_Init(BSP_MOTOR_CONTROL_BOARD_ID_L6474, 1);
  
	/* Attach the function MyFlagInterruptHandler (defined below) to the flag interrupt */
	BSP_MotorControl_AttachFlagInterrupt(MyFlagInterruptHandler);

	  /* Attach the function Error_Handler (defined below) to the error Handler*/
	BSP_MotorControl_AttachErrorHandler(Error_Handler);

	/* Reset device 0 to 1/x microstepping mode */
	BSP_MotorControl_SelectStepMode(0, STEP_MODE_1_8);

	/* Update speed, acceleration, deceleration for 1/x microstepping mode*/
	BSP_MotorControl_SetMaxSpeed(0, 1000);
	BSP_MotorControl_SetMinSpeed(0, 0);
	BSP_MotorControl_SetAcceleration(0, 1200);
	BSP_MotorControl_SetDeceleration(0, 2000);

	// Initialize axis data
	firstAxis.id = 0;    /* Axis ID */
	firstAxis.act_pos = BSP_MotorControl_GetPosition(firstAxis.id); /* Axis actual position */

	/* Next command is necessary to fix the behavior described below:
	   After reset is execute a hard stop even the command is to execute a soft stop.
	   Fail first time after reset to execute a soft stop.
	   Test is done using User Button (blue button). First press motor is running then second press motor is soft stop.
	   Condition (relativePos >= devicePrm[deviceId].stepsToTake) is true first time after Reset because stepsToTake = 0 after initialization.
	   After first Hard Stop stepsToTake = MAX_STEPS, so the above condition is false.
	   Did not want to change ST Spin Library so this is an acceptable fix.
	*/
	BSP_MotorControl_HardStop(firstAxis.id);
}

//******************************************************************************
// stepper_ctrl_Get_Actual_Position
//
//******************************************************************************
int32_t stepper_ctrl_Get_Actual_Position(void)
{
	return firstAxis.act_pos;
}

//******************************************************************************
// Function stepper_ctrl_Set_New_Position
//******************************************************************************
void stepper_ctrl_Set_New_Position(int32_t new_pos)
{
    position_setpoint = new_pos;
}


//******************************************************************************
// stepper_ctrl_Begin
//
//******************************************************************************
void stepper_ctrl_Begin(void)
{
    hmi_jog_n = !(DigitalInput_GetValue(pUsr_Btn_2));
    hmi_jog_p = !(DigitalInput_GetValue(pUsr_Btn_1));
    limit_p = !(DigitalInput_GetValue(pLimit_SW_Plus));
    limit_n = !(DigitalInput_GetValue(pLimit_SW_Minus));
}

//******************************************************************************
// stepper_ctrl_End
//
//******************************************************************************
void stepper_ctrl_End(void)
{
    hmi_jog_n_old = hmi_jog_n;
    hmi_jog_p_old = hmi_jog_p;
}

///////////////////////////////////////////////////////////////////////////////
// Private functions - implementation
///////////////////////////////////////////////////////////////////////////////
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
