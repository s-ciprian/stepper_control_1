
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include "motorcontrol.h"
#include "stepper_ctrl.h"
#include "cmd.h"


#define FIRST_AXIS 0

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
    AbsPos,
    RelPos,
    Jog,
    SetParam,
    ReadParam
} mcCmd_t;


typedef struct _mcCmdData_t
{
	mcCmd_t cmd;
    int32_t pos;
    int32_t dir;
} mcCmdData_t;


// Function prototypes for module commands
static int32_t process_sm_GoTo(int argc, char *argv[]);

// Define commands that will be supported by this module
cmdObj_t sm_GoTo = { "sm_GoTo", process_sm_GoTo, 0 };

static mcState_t mcState;
static mcCmdData_t mcCmdData;


//=========================================================================
// Implements GoTo command - motor driver
//
//=========================================================================
static int32_t process_sm_GoTo(int argc, char *argv[])
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

	return 1;
}

//=========================================================================
// Initialization function
//
// This function will register commands supported by this module
//=========================================================================
void stepper_ctrl_Init(void)
{
	// Register commands
	cmdAddNewCommand(&sm_GoTo);

	// Initialize state machine state
	mcState = Idle;
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
            	BSP_MotorControl_GoTo(FIRST_AXIS, c->pos);
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
			if (INACTIVE == BSP_MotorControl_GetDeviceState(FIRST_AXIS))
			{
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
