#include "stm32f4xx_nucleo.h"
#include "main.h"
#include "stepper_ctrl.h"
#include "../io/di.h"

#include "hca.h"


///////////////////////////////////////////////////////////////////////////////
// Private types definitions
///////////////////////////////////////////////////////////////////////////////
struct HCA_Data
{
    int32_t  target_position;
    int32_t  actual_position;
};


///////////////////////////////////////////////////////////////////////////////
// Private define
///////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////
// Private functions protptype
///////////////////////////////////////////////////////////////////////////////
static void Jog_Mode_Controller(void);
static int32_t Reference_Mode_Controller(void);
static void Positioning_Mode_Controller(void);


///////////////////////////////////////////////////////////////////////////////
// Private variables
///////////////////////////////////////////////////////////////////////////////
static struct HCA_Data hca_data = {0, 0};

static uint16_t hmi_jog_p, hmi_jog_n, hmi_jog_p_old, hmi_jog_n_old;
static uint16_t limit_p, limit_n;


///////////////////////////////////////////////////////////////////////////////
// Public functions - implementation
///////////////////////////////////////////////////////////////////////////////

//******************************************************************************
// Function hca_Init
//******************************************************************************
void hca_Init(void)
{
    hca_data.actual_position = 0;
    hca_data.target_position = 0;

    hmi_jog_n = hmi_jog_n_old = !(DigitalInput_GetValue(pUsr_Btn_2));
	hmi_jog_p = hmi_jog_p_old = !(DigitalInput_GetValue(pUsr_Btn_1));

    limit_p = !(DigitalInput_GetValue(pLimit_SW_Plus));
    limit_n = !(DigitalInput_GetValue(pLimit_SW_Minus));
}

//********************************************************************************
// Function hca_Begin
//********************************************************************************
void hca_Begin(void)
{
    hmi_jog_n = !(DigitalInput_GetValue(pUsr_Btn_2));
    hmi_jog_p = !(DigitalInput_GetValue(pUsr_Btn_1));
    limit_p = !(DigitalInput_GetValue(pLimit_SW_Plus));
    limit_n = !(DigitalInput_GetValue(pLimit_SW_Minus));
}

//********************************************************************************
// Function hca_ProcessEvent
//********************************************************************************
void hca_ProcessEvent(void)
{
    // Application Main Controller states
    enum HCA_State
    {
        Reference_Mode,
        Set_Start_Position,
        Wait_Start_Position,
        Jog_Mode,
        Positioning_Mode
    };

	static enum HCA_State hca_state = Reference_Mode;
    int32_t rmc_ret;

	switch (hca_state)
	{
	case Reference_Mode:
        rmc_ret = Reference_Mode_Controller();
		if (rmc_ret)
        {
		    hca_state = Set_Start_Position;
        }
		break;

    case Set_Start_Position:
        hca_data.target_position = -200;
        stepper_ctrl_Set_New_Position(hca_data.target_position);
        hca_state = Wait_Start_Position;
        break;

	case Wait_Start_Position:
        hca_data.actual_position = stepper_ctrl_Get_Actual_Position();
        if (hca_data.actual_position == hca_data.target_position)
        {
            hca_state = Jog_Mode;
        }
        break;

	case Jog_Mode:
        Jog_Mode_Controller();
		break;

	case Positioning_Mode:
        Positioning_Mode_Controller();
		break;

	default:
		hca_state = Reference_Mode;
		break;
	}
}

//********************************************************************************
// Function hca_End
//********************************************************************************
void hca_End(void)
{
}


///////////////////////////////////////////////////////////////////////////////
// Private functions - implementation
///////////////////////////////////////////////////////////////////////////////

//********************************************************************************
// Function Jog_Mode_Controller
// Moving axix in Jog mode by two HMI buttons. Stop when limits switches are hit
//********************************************************************************
static void Jog_Mode_Controller(void)
{
    // Jog Mode Controller states
    enum JMC_State
    {
        Idle,
        Jog_P,
        Jog_N
    };

    // State of Reference Mode Controller state machine
    static enum JMC_State jmc_state = Idle;

    switch (jmc_state)
    {
    case Idle:
        // If not on Positive limit switch and Jog positive input ON.
        // Check also Jog negative to avoid movement from limit to limit when both button are pressed
	    if (!limit_p && hmi_jog_p && !hmi_jog_n)
	    {
		    jmc_state = Jog_P;
	    }
		// If not on Negative limit switch and Jog negative input ON
		// Check also Jog negative to avoid movement from limit to limit when both button are pressed
	    else if (!limit_n && hmi_jog_n && !hmi_jog_p)
	    {
		    jmc_state = Jog_N;
	    }
        break;

    case Jog_P:
        stepper_ctrl_Jog_P();
        if (!hmi_jog_p || limit_p)  // "HMI input OFF" OR "Limit SW P activated"
        {
            stepper_ctrl_Stop();  // Stop motor
            jmc_state = Idle;
        }
        break;

    case Jog_N:
        stepper_ctrl_Jog_N();
	    if (!hmi_jog_n || limit_n)  // "HMI input OFF" OR "Limit SW N activated"
        {
            stepper_ctrl_Stop();  // Stop motor
            jmc_state = Idle;
        }
        break;

    default:
        jmc_state = Idle;
        break;
    }
}

//********************************************************************************
// Function Reference_Mode_Controller()
// This function is called at 20 ms recurrence by OS
// Returns:
//  0 = REF not done, 1 = REF done, -1 = Not possible to do REF, error 
//********************************************************************************
static int32_t Reference_Mode_Controller(void)
{
    // Possible states for Reference Mode Controller state machine
    enum RMC_State
    {
        Start_Delay,
        Search_Ref_Switch,
        Ref_Safe_Position,
        Wait_For_Stop,
        Ref_Done
    };
    // Delays used by RMC state machine. This function is called @ 20 ms
    enum DELAYS
    {
        START_DELAY = 20,      // x20 ms
        WAIT_STOP_DELAY = 30,
    };

    // Return value. TODO - Add a timeout in state Search_Ref_Switch and return -1 if timeout
    int32_t rmc_return = 0;
    // State of Reference Mode Controller state machine
    static enum RMC_State rmc_state = Start_Delay;
    // Generic timer (now set with initial delay, before to start REF process)
    static uint8_t rmc_tmr = START_DELAY;

	switch (rmc_state)
    {
    // Short delay after MCU, OS has started (not really necessary)
    case Start_Delay:
	    if (rmc_tmr)
	    {
		    rmc_tmr--;
	    }
	    else
        {
            rmc_state = Search_Ref_Switch;
        }
        break;

    // Simulate a JOG Plus command to hit the limit switch. This is REFERENCE position
    case Search_Ref_Switch:
	    if (!limit_p)   // If not on limit switch go to Plus direction
	    {
		    stepper_ctrl_Jog_P();
	    }
	    else    // Stop when hit positive limit switch
        {
            stepper_ctrl_Stop();
            rmc_tmr = WAIT_STOP_DELAY;      // Set a delay, see below why
            rmc_state = Wait_For_Stop;
        }
        break;

    // Wait for motor to decelerate and stop
    case Wait_For_Stop:
        if (rmc_tmr)    // Decrement timer
        {
            rmc_tmr--;
        }
        else
        {
            // Set zero position into motor controller chip
            stepper_ctrl_Set_Home();
            rmc_state = Ref_Done;
        }
        break;

    // Reference is DONE
    case Ref_Done:
        rmc_return = 1; // Sucessful
        break;

    default:
        rmc_state = Search_Ref_Switch;
        break;
    }

    return rmc_return;
}

//********************************************************************************
// Function Jog_Mode_Controller
//********************************************************************************
static void Positioning_Mode_Controller(void)
{

}
