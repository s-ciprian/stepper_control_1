#ifndef __STEPPER_CTRL_H__
#define __STEPPER_CTRL_H__


#ifdef __cplusplus
extern "C"
{
#endif

void mcInit(void);
void stepper_ctrl_Begin(void);
void stepper_ctrl_ProcessEvent(void);
void stepper_ctrl_End(void);
int32_t stepper_ctrl_Get_Actual_Position(void);
void stepper_ctrl_Set_New_Position(int32_t new_pos);
void stepper_ctrl_Set_Home(void);
void stepper_ctrl_Jog_P(void);
void stepper_ctrl_Jog_N(void);
void stepper_ctrl_Stop(void);

#ifdef __cplusplus
}
#endif

#endif // __STEPPER_CTRL_H__
