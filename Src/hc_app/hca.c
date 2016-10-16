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

enum HCA_State    // Application Main Controller state
{
    Reference_Mode,
    Jog_Mode,
    Positioning_Mode
};

enum JMC_State    // Jog Mode Controller state
{
    Idle,
    Jog_P,
    Jog_N
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
	static enum HCA_State hca_state = Reference_Mode;

	switch (hca_state)
	{
	case Reference_Mode:
        Reference_Mode_Controller();
		hca_state = Jog_Mode; // Stub - just for test
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

    //======= Testing position commands =======
//    hca_data.actual_position = stepper_ctrl_Get_Actual_Position();
//
//    if (hca_data.actual_position != hca_data.target_position)
//    {
//        stepper_ctrl_Set_New_Position(hca_data.target_position);
//    }
//    else
//    {
//        if (hca_data.actual_position == 0)
//        {
//            hca_data.target_position = -600;
//        }
//        else if  (hca_data.actual_position == -600)
//        {
//            hca_data.target_position = 0;
//        }
//    }
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
//********************************************************************************
static void Jog_Mode_Controller(void)
{
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
// Returns:
//  0 = REF not done, 1 = REF done, -1 = Not possible to do REF, error 
//********************************************************************************
static int32_t Reference_Mode_Controller(void)
{
    return 0;
}

static void Positioning_Mode_Controller(void)
{

}
