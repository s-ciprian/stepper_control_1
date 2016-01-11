#ifndef __STEPPER_CTRL_H__
#define __STEPPER_CTRL_H__


#ifdef __cplusplus
extern "C"
{
#endif

typedef enum _mcDriveStatus_t
{
	mcDriverReady,
	mcDriverBusy
} mcDriveStatus_t;

void mcInit(void);
void mcRecurrentFnc(uint32_t current_time);
mcDriveStatus_t mcGetDriverStatus(void);


#ifdef __cplusplus
}
#endif

#endif // __STEPPER_CTRL_H__
