
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include "stepper_ctrl.h"
#include "cmd.h"

// Function prototypes for module commands
static int32_t process_sm_GoTo(int argc, char *argv[]);

// Define commands that will be supported by this module
cmdObj_t sm_GoTo = { "sm_GoTo", process_sm_GoTo, 0 };

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
    }

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
}
