#ifndef __CMD_H__
#define __CMD_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

//#define CMD_ERR_BAD_CMD         (-1)   // Error code - command is not found.
//#define CMD_ERR_TOO_MANY_ARGS   (-2)   // Error code -  too many arguments.

enum cmd_err
{
	CMD_ERR_NONE = 0,
	CMD_ERR_UNKNOWN_CMD = -1,
	CMD_ERR_TOO_MANY_ARGS = -2,
	CMD_ERR_STRING_TO_LONG = -3
};

//*****************************************************************************
//
// Command line function callback type.
//
//*****************************************************************************
typedef int32_t (*cmdFunction_t)(int argc, char *argv[]);

// Structure defines an entry in the command list table.
typedef struct _cmdObj
{
    const char     *cmdName;       // Pointer to a string, the name of the command.
	cmdFunction_t  cmdFunction;	   // Function pointer to the implementation of the command.
	struct _cmdObj *next;		   // Reference to the next object in the list
} cmdObj_t;

//*****************************************************************************
// Prototypes for the APIs.
//*****************************************************************************
extern int ExecuteCommand(char *cmdBuf);

int cmdAddNewCommand(cmdObj_t *newCmd);


#ifdef __cplusplus
}
#endif

#endif // __CMDLINE_H__
