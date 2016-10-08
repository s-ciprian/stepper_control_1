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
int32_t mc_Get_MotorPosition(void);

#ifdef __cplusplus
}
#endif

#endif // __STEPPER_CTRL_H__
