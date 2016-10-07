#ifndef __STEPPER_CTRL_H__
#define __STEPPER_CTRL_H__


#ifdef __cplusplus
extern "C"
{
#endif

enum Stepper_Ctrl_Event
{
    HMI_JOG_PLUS_BTN_DOWN,
    HMI_JOG_PLUS_BTN_UP,
    HMI_JOG_MINUS_BTN_DOWN,
    HMI_JOG_MINUS_BTN_UP,
    GOTO_ABS_POS            //Position should be sent before this call with another interface
};

typedef enum _mcDriveStatus_t
{
	mcDriverReady,
	mcDriveJogging,
	mcDriverBusy
} mcDriveStatus_t;

void mcInit(void);
void mcRecurrentFnc(uint32_t current_time);
int32_t mc_Get_MotorPosition(void);

/* Interface used to send events to this module */
void Send_Event_To_Stepper_Ctrl(enum Stepper_Ctrl_Event mc_ev);


#ifdef __cplusplus
}
#endif

#endif // __STEPPER_CTRL_H__
