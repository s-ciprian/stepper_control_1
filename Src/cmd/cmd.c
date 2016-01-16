
//*****************************************************************************
//
//! \addtogroup cmd_api
//! @{
//
//*****************************************************************************

#include <string.h>
#include "cmd.h"
#include "stdint.h"


// Defines the maximum number of arguments that can be parsed.
#ifndef CMD_MAX_ARGS
#define CMD_MAX_ARGS        8
#endif

// Define parser state machine STATES
typedef enum
{
	INIT,
	TOKEN,
	DELIMITER,
	ERROR
} parser_state_t;

// argv table - holding start pointer of each argument
static char *argv[CMD_MAX_ARGS + 1];
// temporary buffer
static char tmpBuf[64] = "";
// number of arguments (counter)
static int argc;


// Start of the command list
cmdObj_t *cmdListHead = NULL;
uint32_t cmdListCnt = 0;
//cmdObj_t *cmdCurrentCmd = NULL;

static int cmdUpdateArgvTable(char *cmdBuf);
static int cmdExecuteCommand(void);
static int cmdParseCommand(char *cmdBuf);


//*****************************************************************************
// Process a command. First the parser function is called and if this is
// executed without error than Execute Command function is called.
//
// \param cmdBuf points to a string that contains a command.
//
// \return Returns an error value if the parse phase ends with error of if 
// the command function not found.
// In case of success (function associated with command is executed), returns
// the code returned by command function
//*****************************************************************************
int ExecuteCommand(char *cmdBuf)
{
	int ret = 0;

	ret = cmdParseCommand(cmdBuf);

	// If parsing is sucessful
	if (ret == 0)
	{
		// If one or more arguments was found, then process the command.
		if (argc)
		{
			ret = cmdExecuteCommand();
		}
	}

	return ret;
}

//=========================================================================
// This function parse command buffer
//
// This function will take the supplied command line string and break it
// up into individual arguments. All of the command line arguments are
// save in the normal argc, argv form in *argv[] table. Integer argc
// stores the number of parameters including the command itself
//
// \param cmdBuf points to the command buffer. 
//========================================================================= 
static int cmdParseCommand(char *cmdBuf)
{
	// Pointer to the command buffer
	char *ch;
	// State of the parser
	static parser_state_t parser_state;
	// Store return value
	int ret_val;
	// Error flag
	int error = 0;
	// Input string length counter
	unsigned int str_cnt = 0;

	// Initialize state machine
	parser_state = INIT;
	// Initialize argument counter
	argc = 0;
	// Point to the first character of the input string
	ch = tmpBuf;
	ret_val = CMD_ERR_NONE;

	// Copy input string to a temporary buffer
	for (; str_cnt < 64; str_cnt++)
	{
		tmpBuf[str_cnt] = 0; // Clear entire array
		if (cmdBuf[str_cnt] != '\0')
		{
			// Copy characters
			tmpBuf[str_cnt] = cmdBuf[str_cnt];
		}
	}
	if (str_cnt > 64)
	{
		return CMD_ERR_STRING_TO_LONG;
	}

	// Parse the command string. Stop in case of an error.
	while (*ch)
	{
		// Parse state machine
		switch (parser_state)
		{
			// This is the state after initialization.
		case INIT:
			// Replace all spaces characts with a 0 value
			// Keep doing this until a characted other than space is found
			if (*ch == ' ')
			{
				*ch = 0;
				parser_state = INIT;
			}
			// In case of start of a new token, save the pointer in argv table and increment the argument counter and switch to TOKEN state.
			else
			{
				ret_val = cmdUpdateArgvTable(ch);	//! This function will update argc counter and *argv[] table. Not passed as arguments beacause is more easy to understand like this.
				if (ret_val != CMD_ERR_NONE)
				{
					parser_state = ERROR;
				}
				else
				{
					parser_state = TOKEN;
				}
			}
			break;
		case TOKEN:
			// Skip all characters that are valid to form a token, but switch state if a delimiter character (space) is found.
			// Patch the input buffer in case of space char 
			if (*ch == ' ')
			{
				*ch = 0;
				parser_state = DELIMITER;
			}
			break;
		case DELIMITER:
			if (*ch == ' ')
			{
				*ch = 0;
				parser_state = DELIMITER;
			}
			// In case of start of a new token, save the pointer in argv table and increment the argument counter and switch to TOKEN state.
			else
			{
				ret_val = cmdUpdateArgvTable(ch); //! This function will update argc counter and *argv[] table. Not passed as arguments beacause is more easy to understand like this.
				if (ret_val != CMD_ERR_NONE)
				{
					parser_state = ERROR;
				}
				else
				{
					parser_state = TOKEN;
				}
			}
			break;
		case ERROR:
			error = 1;
			break;
		default:
			break;
		}

		if (error)
		{
			break; // Stop "while" loop, and return curent error code saved in ret_val
		}
		// Advance to the next character in input buffer (commnad buffer).
		ch++;
	}

	return ret_val;
}

//=========================================================================
// In case of start of a new token, save the pointer in argv table and
// increment the argument counter
//
// \param cmdBuf points to a charcter from command buffer. This charcter
// is the first character of a token
//
// This function globally access variables argc and argv.
// This is easier to understood that to pass pointers to this two variables
//========================================================================= 
static int cmdUpdateArgvTable(char *cmdBuf)
{
	if (argc < CMD_MAX_ARGS)
	{
		argv[argc] = cmdBuf;
		argc++;
	}
	else
	{
		return (CMD_ERR_TOO_MANY_ARGS);
	}

	return (CMD_ERR_NONE);
}

//=========================================================================
// Call function associated with the command (execute de command)
//
// \param cmdObj points to an object in command table. 
//========================================================================= 
static int cmdExecuteCommand(void)
{
	cmdObj_t *cmdList_itor = NULL;

	cmdList_itor = cmdListHead;

	// Search through the command table.
	do
	{
		// If this command entry command string matches argv[0], then call
		// the function for this command, passing the command line arguments.
		if (!strcmp(argv[0], cmdList_itor->cmdName))
		{
			return(cmdList_itor->cmdFunction(argc, argv));
		}

		// Not found, so advance to the next entry.
		cmdList_itor = cmdList_itor->next;
	} while (cmdList_itor != NULL);

	// Fall through to here means that no matching command was found, so return an error.
	return (int)(CMD_ERR_UNKNOWN_CMD);

	//return (cmdListHead->cmdFunction(argc, argv));
}

//=========================================================================
//
//=========================================================================
int cmdAddNewCommand(cmdObj_t *newCmd)
{
	cmdObj_t *cmdList_itor = NULL;
	int ret_val = 0;

	if (newCmd != NULL)
	{
		if (cmdListHead == NULL)
		{
			cmdListHead = newCmd;
		}
		else
		{
			cmdList_itor = cmdListHead;
			while (cmdList_itor->next != NULL)
			{
				cmdList_itor = cmdList_itor->next;
			}
			cmdList_itor->next = newCmd;
		}
		newCmd->next = NULL;
		cmdListCnt++;
	}
	else
	{
		ret_val = -1;
	}

	return ret_val;
}



//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
