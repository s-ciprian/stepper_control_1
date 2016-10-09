#include "stm32f4xx_nucleo.h"
#include "main.h"
#include "stepper_ctrl.h"
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


///////////////////////////////////////////////////////////////////////////////
// Private variables
///////////////////////////////////////////////////////////////////////////////
static struct HCA_Data hca_data = {0, 0};


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
}

//********************************************************************************
// Function hca_Begin
//********************************************************************************
void hca_Begin(void)
{
}

//********************************************************************************
// Function hca_ProcessEvent
//********************************************************************************
void hca_ProcessEvent(void)
{
    //======= Testing position commands =======
    hca_data.actual_position = stepper_ctrl_Get_Actual_Position();

    if (hca_data.actual_position != hca_data.target_position)
    {
        stepper_ctrl_Set_New_Position(hca_data.target_position);
    }
    else
    {
        if (hca_data.actual_position == 0)
        {
            hca_data.target_position = -600;
        }
        else if  (hca_data.actual_position == -600)
        {
            hca_data.target_position = 0;
        }
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

