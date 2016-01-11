
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include "stm32f4xx_nucleo.h"
#include "motorcontrol.h"
#include "stepper_ctrl.h"
#include "cmd.h"
#include "sch_hlp.h"


typedef enum _mcState_t
{
	Idle,
	Movement_Positioning_Absolute,
	Movement_Positioning_Relative,
	Movement_Jog,
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
    SetParam,
    ReadParam
} mcCmd_t;


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
} mcCmdData_t;


static void stepper_ctrl_ProcessEvent(mcCmdData_t *c);

// Function prototypes for module commands
static int32_t process_mc_GoTo(int argc, char *argv[]);
static int32_t process_mc_GetPos(int argc, char *argv[]);

//===== Define commands that will be supported by this module ====
// GoTo command - absolute positioning
cmdObj_t mc_GoTo = { "mc_GoTo", process_mc_GoTo, 0 };
// Get Actual Position command - absolute position
cmdObj_t mc_GetPos = { "mc_GetPos", process_mc_GetPos, 0 };

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

    if (errno == ERANGE)
    {
    	errno = 0;
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
// chip if there is only an interrogation (e.g. check actual position).
//=========================================================================
static int32_t process_mc_GetPos(int argc, char *argv[])
{
	// TODO: check if argc == 2. If argc <> 2 then report an error

    int32_t *p;

    uint32_t n;

    n = strlen(argv[1]);

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

	// Initialize state machine state
	mcState = Idle;
	// Initialize axis data
	firstAxis.id = 0;
}

//=========================================================================
// Controller function
//
// This function will handle all events that are processed by this
// controller
//=========================================================================
void stepper_ctrl_ProcessEvent(mcCmdData_t *c)
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
			if (INACTIVE == BSP_MotorControl_GetDeviceState(firstAxis.id))
			{
				// Get current position of device. Prepare for other components that needs this
				firstAxis.act_pos = BSP_MotorControl_GetPosition(firstAxis.id);

				mcCmdData.cmd = NoCmd;
				mcState = Idle;
			}
			break;
		case Movement_Positioning_Relative:

			break;
		case Movement_Jog:

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
	else
	{
		return mcDriverBusy;
	}
}

//=========================================================================
// Main function of this code block
//
//=========================================================================
void mcRecurrentFnc(uint32_t current_time)
{
	const uint32_t tsk_recr = 10;
	static uint32_t elapsed_time = 0;
	static uint32_t prevoius_time = 0;


	elapsed_time = timeDiff(current_time, prevoius_time);

	if (elapsed_time >= tsk_recr)
	{
		// call here function "stepper_ctrl_ProcessEvent"
	    mcCmdData.cmd = Tick;
	    mcCmdData.dir = -1;
	    mcCmdData.pos = 0;
		stepper_ctrl_ProcessEvent(&mcCmdData);

		// store current time
		prevoius_time = current_time;
	}

}
