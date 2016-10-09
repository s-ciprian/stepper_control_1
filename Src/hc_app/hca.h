#ifndef __HCA_H__
#define __HCA_H__


#ifdef __cplusplus
extern "C"
{
#endif

void hca_Init(void);
void hca_Begin(void);
void hca_ProcessEvent(void);
void hca_End(void);


#ifdef __cplusplus
}
#endif

#endif // __STEPPER_CTRL_H__