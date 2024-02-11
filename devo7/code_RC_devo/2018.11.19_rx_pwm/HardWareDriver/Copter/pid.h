#include "Fmath.h"

#define RP_PID_MIN -350.0f
#define RP_PID_MAX 350.0f

#define RP_PID_I_MIN -150.0f
#define RP_PID_I_MAX 150.0f

#define RP_PID_I_MIN -150.0f
#define RP_PID_I_MAX 150.0f

#define RP_KI_Enable 1

//extern from main
extern float rate_rpy_setpoint[3],rate_rpy[3];
extern double dt;
void PID_RollPitchYaw(int16_t *PID_Roll_Out, int16_t *PID_Pitch_Out, int16_t *PID_Yaw_Out);

void reset_rate_PID(void);
