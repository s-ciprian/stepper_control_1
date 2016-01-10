#ifndef __STEPPER_CTRL_H__
#define __STEPPER_CTRL_H__


#ifdef __cplusplus
extern "C"
{
#endif


void stepper_ctrl_Init(void);
void stepper_ctrlFnc(uint32_t current_time);


#ifdef __cplusplus
}
#endif

#endif // __STEPPER_CTRL_H__
