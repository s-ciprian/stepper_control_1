
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include "stm32f4xx_nucleo.h"
#include "motorcontrol.h"
#include "stepper_ctrl.h"
#include "cmd.h"
#include "sch_hlp.h"  // TODO: Remove this file

#include "main.h"

//Motor controller chip - definitions
#include "..\Components\l6474\l6474.h"
#ifdef USE_STM32F4XX_NUCLEO
#include "stm32f4xx_nucleo_ihm01a1.h"
#endif

//================================================================================
//
//================================================================================
typedef enum _mcState_t
{
	Idle,
	Movement_Positioning_Absolute,
	Movement_Positioning_Relative,
	Movement_Jog,
	Wait_Standstill,
	Parametrization,
	ReadData
} mcState_t;

typedef enum _mcCmd_t
{
	NoCmd,
	Tick,
    AbsPos,
    RelPos,
    Jog,
    Stop,
    SetParam,
    ReadParam
} mcCmd_t;

typedef enum _mcStop_t
{
	Hard,
	Soft
} mcStop_t;

typedef struct _mcAxis_t
{
	uint8_t id;
	int32_t act_pos;
} mcAxis_t;

typedef struct _mcCmdData_t
{
	mcCmd_t cmd;
    int32_t pos;
    int32_t dir;
    mcStop_t s_type;
} mcCmdData_t;

//================================================================================
// Private functions protptype
//================================================================================
static inline void mcMovementEnding(void);

static void stepper_ctrl_ProcessEvent(mcCmdData_t *c);

static void MyFlagInterruptHandler(void);

// Function prototypes for module commands
static int32_t process_mc_GoTo(int argc, char *argv[]);
static int32_t process_mc_GetPos(int argc, char *argv[]);
static int32_t process_mc_HMI_JogP(int argc, char *argv[]);
static int32_t process_mc_HMI_JogN(int argc, char *argv[]);
static int32_t process_mc_Stop(int argc, char *argv[]);


//===== Define commands that will be supported by this module ====
// GoTo command - absolute positioning
cmdObj_t mc_GoTo = { "mc_GoTo", process_mc_GoTo, 0 };
// Jog Forward
cmdObj_t mc_JogP = { "HMI_JogP", process_mc_HMI_JogP, 0 };
// Jog Forward
cmdObj_t mc_JogN = { "HMI_JogN", process_mc_HMI_JogN, 0 };
// Stop command - Soft or Hard as parameter
cmdObj_t mc_Stop = { "mc_Stop", process_mc_Stop, 0 };
// Get Actual Position command - absolute position
cmdObj_t mc_GetPos = { "mc_GetPos", process_mc_GetPos, 0 };

//================================================================================
// Private variables
//================================================================================
static mcState_t mcState;
static mcCmdData_t mcCmdData;
static mcAxis_t firstAxis;


//=========================================================================
// Implements GoTo command - motor driver
//
//=========================================================================
static int32_t process_mc_GoTo(int argc, char *argv[])
{
	// TODO: check if argc == 2. If argc <> 2 then report an error

    char *end;
    int32_t n;

    n = strtol(argv[1], &end, 10);

    // ERR: Not possible to convert from string
    if (errno == ERANGE)
    {
    	errno = 0;
    	// TODO: Add application level error handling

    	return -1;
    }

    // ERR: Command rejected because another motor mc command execution is in progress
    if (mcState != Idle)
    {
    	// TODO: Add application level error handling

    	return -1;
    }

    mcCmdData.cmd = AbsPos;
    mcCmdData.dir = -1;
    mcCmdData.pos = n;

    stepper_ctrl_ProcessEvent(&mcCmdData);

	return 1;
}

//=========================================================================
// Implements Get Absolute Position command
//
// Note! This function returns position saved in this module and not
// read actual position from drive chip. To be consistent after each
// positioning or movement operation local axis position should be updated.
// This implementation is chosen in order to minimize the commands sent to
// chip if there is only an interrogation about some of the parameters
// (e.g. check actual position).
//=========================================================================
static int32_t process_mc_GetPos(int argc, char *argv[])
{
	// TODO: check if argc == 2. If argc <> 2 then report an error

    int32_t *p;

    p = (int32_t *) strtol(argv[1], NULL, 16);
    *p = firstAxis.act_pos;

    if (errno == ERANGE)
    {
    	errno = 0;
    	// TODO: Add application level error handling

    	return -1;
    }

	return 1;
}

//-------------------------------------------------------------------------
// Process Jog Positive Dir event
//-------------------------------------------------------------------------
static int32_t process_mc_HMI_JogP(int argc, char *argv[])
{
	// TODO: check if argc == 2. If argc <> 2 then report an error

	if (strcmp(argv[1], "Down") == 0)
	{
		mcCmdData.cmd = Jog;	
		mcCmdData.dir = FORWARD;
	}
	else if (strcmp(argv[1], "Up") == 0)
	{
		mcCmdData.cmd = Stop;
		mcCmdData.s_type = Soft;		
	}
	else
	{
		//ERR: Not a valid direction in command. Syntax error.
		// TODO: Add application level error handling
		return -1;
	}
	
    stepper_ctrl_ProcessEvent(&mcCmdData);

	return 1;
}

//-------------------------------------------------------------------------
// Process Jog Positive Dir event
//-------------------------------------------------------------------------
static int32_t process_mc_HMI_JogN(int argc, char *argv[])
{
	// TODO: check if argc == 2. If argc <> 2 then report an error

	if (strcmp(argv[1], "Down") == 0)
	{
		mcCmdData.cmd = Jog;
		mcCmdData.dir = BACKWARD;
	}
	else if (strcmp(argv[1], "Up") == 0)
	{
		mcCmdData.cmd = Stop;
		mcCmdData.s_type = Soft;
	}
	else
	{
		//ERR: Not a valid direction in command. Syntax error.
		// TODO: Add application level error handling
		return -1;
	}
	
	stepper_ctrl_ProcessEvent(&mcCmdData);

	return 1;
}

//=========================================================================
// Implements Stop Hard/Stop command - motor driver
//
//=========================================================================
static int32_t process_mc_Stop(int argc, char *argv[])
{
	// TODO: check if argc == 2. If argc <> 2 then report an error

	mcStop_t s;

	if (strcmp(argv[1], "Hard") == 0)
	{
		s = Hard;
	}
	else if (strcmp(argv[1], "Soft") == 0)
	{
		s = Soft;
	}
	else
	{
		//ERR: Not a valid parameter for this command. Syntax error.
		// TODO: Add application level error handling
		return -1;
	}

    // ERR: Command rejected because another motor mc command execution is in progress
    if ( (mcState != Movement_Jog) &&
    	 (mcState != Movement_Positioning_Absolute)	&&
    	 (mcState != Movement_Positioning_Relative) &&
		 (mcState != Wait_Standstill) )
    {
    	// TODO: Add application level error handling

    	return -1;
    }

    mcCmdData.cmd = Stop;
    mcCmdData.s_type = s;

    stepper_ctrl_ProcessEvent(&mcCmdData);

	return 1;
}

//=========================================================================
// Controller function
//
// This function will handle all events that are processed by this
// controller
//=========================================================================
static void stepper_ctrl_ProcessEvent(mcCmdData_t *c)
{
	switch (mcState) {
		case Idle:
            if (c->cmd == AbsPos)
            {
            	BSP_MotorControl_GoTo(firstAxis.id, c->pos);
            	mcState = Movement_Positioning_Absolute;
            }
            else if (c->cmd == RelPos)
            {

            }
            else if (c->cmd == Jog)
            {
            	BSP_MotorControl_Run(firstAxis.id, c->dir);
            	mcState = Movement_Jog;
            }
            else if (c->cmd == SetParam)
            {

            }
            else if (c->cmd == ReadParam)
            {

            }
            else
            {
            	// TODO: Add - error, command not supported
            }
			break;

		case Movement_Positioning_Absolute:
			// TODO: Handle stop command
			if (INACTIVE == BSP_MotorControl_GetDeviceState(firstAxis.id))
			{
				mcMovementEnding();
			}
			break;

		case Movement_Positioning_Relative:

			break;

		case Movement_Jog:
			if (c->cmd == Stop)
			{
				if (c->s_type == Hard)
				{
					BSP_MotorControl_HardStop(firstAxis.id);
				}
				else
				{
					BSP_MotorControl_SoftStop(firstAxis.id);
				}
				mcState = Wait_Standstill;
			}
			firstAxis.act_pos = BSP_MotorControl_GetPosition(firstAxis.id); /* Axis actual position */
			break;

		case Wait_Standstill:
		    if (c->cmd == Stop)   // Stop during DECELERATION, case when hiting limit switch while DECELERATING. Then need a HARD STOP  
		    {
			    if (c->s_type == Hard)
			    {
				    BSP_MotorControl_HardStop(firstAxis.id);
			    }
			    else
			    {
				    BSP_MotorControl_SoftStop(firstAxis.id);
			    }
		    }
		
			// Wait here until motor decelerating
			if (INACTIVE == BSP_MotorControl_GetDeviceState(firstAxis.id))
			{
				mcMovementEnding();
			}
			firstAxis.act_pos = BSP_MotorControl_GetPosition(firstAxis.id); /* Axis actual position */
			break;

		case Parametrization:

			break;
		case ReadData:

			break;

		default:
			mcState = Idle;
			// TODO: Add an error if this state is reached
			break;
	}

}

//=========================================================================
// Put together some commands that seems to repeat at the end of movement
// states
//
//=========================================================================
static inline void mcMovementEnding(void)
{
	// Get current position of device. Prepare for other components that needs this
	firstAxis.act_pos = BSP_MotorControl_GetPosition(firstAxis.id);

	mcCmdData.cmd = NoCmd;
	mcState = Idle;
}

//=========================================================================
// Initialization function
//
// This function will register commands supported by this module
//=========================================================================
void mcInit(void)
{
	// Register commands
	cmdAddNewCommand(&mc_GoTo);
	cmdAddNewCommand(&mc_GetPos);
	cmdAddNewCommand(&mc_JogP);
	cmdAddNewCommand(&mc_JogN);
	cmdAddNewCommand(&mc_Stop);

	// Initialize state machine state
	mcState = Idle;

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

//=========================================================================
// Main function of this code block
//
//=========================================================================
void mcRecurrentFnc(uint32_t current_time)
{
	// call here function "stepper_ctrl_ProcessEvent"
	mcCmdData.cmd = Tick;
	mcCmdData.dir = -1;
	mcCmdData.pos = 0;
	stepper_ctrl_ProcessEvent(&mcCmdData);
}

//=========================================================================
// Provide drive status
//
// mcDriveReady - a new command could be executed
// mcDriveBusy - a command execution is in progress, new commands not
// accepted
//=========================================================================
mcDriveStatus_t mcGetDriverStatus(void)
{
	if (mcState == Idle)
	{
		return mcDriverReady;
	}
	else if (mcState == Movement_Jog)
	{
		return mcDriveJogging;
	}
	else
	{
		return mcDriverBusy;
	}
}

int32_t mc_Get_MotorPosition(void)
{
	return firstAxis.act_pos;
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
