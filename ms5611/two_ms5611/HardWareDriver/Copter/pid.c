#include "pid.h"

typedef struct {
    float p,i,d;
} pid_para_t;
//pid_para_t rate_roll = {.54f, .008f, 17.6f};


pid_para_t rate_roll = {0.54f, 1.7f, 0.0704f};
pid_para_t rate_pitch = {0.54f, 1.7f, 0.0704f};
pid_para_t rate_yaw = {3.3f, 2.0f, 0.0f};

//float Kp_Rate_Roll_PID=.54,Ki_Rate_Roll_PID=.004,Kd_Rate_Roll_PID=17.6;
//float Kp_Rate_Pitch_PID=.54,Ki_Rate_Pitch_PID=.004, Kd_Rate_Pitch_PID=17.6;
//float Kp_Rate_Yaw_PID=.54,Ki_Rate_Yaw_PID=.004, Kd_Rate_Yaw_PID=17.6;

float Rate_Roll_PID, Rate_Pitch_PID, Rate_Yaw_PID;
		// Must remain values
float Pre_Rate_Roll_error, Pre_Rate_Pitch_error, Pre_Rate_Yaw_error;
float Rate_Roll_PID_I, Rate_Pitch_PID_I, Rate_Yaw_PID_I;
float Rate_Roll_PID_D, Rate_Pitch_PID_D, Rate_Yaw_PID_D;
/*
+ PID depends on dt. Because the loop is constant time so we temporarily don't take it into account.
+ Apply LPF.
*/

void PID_RollPitchYaw(int16_t *PID_Roll_Out, int16_t *PID_Pitch_Out, int16_t *PID_Yaw_Out) {
    float Rate_Roll_error,Rate_Pitch_error,Rate_Yaw_error;
/*
input: Angular RATE calculated, And Rate setpoint
1. I=I+Ki*Error
2. Constrain PID 
3. Decide to control with Ki or not, if not then I=0
4. D=Kd*(error-pre_error)
5. P=Kp*Error
6. PID=P+I+D.
7. constrain PID value
*/
    Rate_Roll_error = rate_rpy_setpoint[0] - rate_rpy[0] ;
    Rate_Roll_PID_I += rate_roll.i *(Rate_Roll_error + Pre_Rate_Roll_error) * dt / 2.0f;
		Rate_Roll_PID_I = Math_fConstrain(Rate_Roll_PID_I, RP_PID_I_MIN, RP_PID_I_MAX);
		Rate_Roll_PID_I *= KI_Enable;
		Rate_Roll_PID_D = rate_roll.d*(Rate_Roll_error - Pre_Rate_Roll_error)/dt;
    Rate_Roll_PID = rate_roll.p * Rate_Roll_error + Rate_Roll_PID_I + Rate_Roll_PID_D;
		Rate_Roll_PID = Math_fConstrain(Rate_Roll_PID, RP_PID_MIN, RP_PID_MAX);
    Pre_Rate_Roll_error = Rate_Roll_error;

    //Pitch calculations
    Rate_Pitch_error = rate_rpy_setpoint[1] - rate_rpy[1] ;
    Rate_Pitch_PID_I += rate_pitch.i * (Rate_Pitch_error + Pre_Rate_Pitch_error) * dt / 2.0f;
		
		Rate_Pitch_PID_I = Math_fConstrain(Rate_Pitch_PID_I, RP_PID_I_MIN, RP_PID_I_MAX);
		Rate_Pitch_PID_I *= KI_Enable;
		Rate_Pitch_PID_D = rate_pitch.d * (Rate_Pitch_error - Pre_Rate_Pitch_error)/dt;
    Rate_Pitch_PID = rate_pitch.p * Rate_Pitch_error + Rate_Pitch_PID_I + Rate_Pitch_PID_D;
		Rate_Pitch_PID = Math_fConstrain(Rate_Pitch_PID, RP_PID_MIN, RP_PID_MAX);
    Pre_Rate_Pitch_error = Rate_Pitch_error;
		
    //This should be reconsider 
    Rate_Yaw_error = rate_rpy_setpoint[2] - rate_rpy[2] ;
    Rate_Yaw_PID_I += rate_yaw.i * (Rate_Yaw_error + Pre_Rate_Yaw_error)*dt/2.0f;
		Rate_Yaw_PID_I = Math_fConstrain(Rate_Yaw_PID_I, -50.0f, 50.0f);
		Rate_Yaw_PID_I *= KI_Enable;
		Rate_Yaw_PID_D = rate_pitch.d * (Rate_Yaw_error - Pre_Rate_Yaw_error)/dt;
    Rate_Yaw_PID = rate_yaw.p * Rate_Yaw_error + Rate_Yaw_PID_I + Rate_Yaw_PID_D;
		Rate_Yaw_PID = Math_fConstrain(Rate_Yaw_PID, -150.0f, 150.0f);
		Pre_Rate_Yaw_error = Rate_Yaw_error;
	//	Rate_Yaw_PID = 0.0f;
	  *PID_Roll_Out = (int16_t)Rate_Roll_PID;
  	*PID_Pitch_Out = (int16_t)Rate_Pitch_PID;
  	*PID_Yaw_Out = (int16_t)Rate_Yaw_PID;
}
		
void reset_rate_PID(void) {
    Rate_Roll_PID = Rate_Pitch_PID = Rate_Yaw_PID = 0.0f;
    Rate_Roll_PID_I = Rate_Pitch_PID_I = Rate_Yaw_PID_I = 0.0f;
    Pre_Rate_Roll_error = Pre_Rate_Pitch_error = Pre_Rate_Yaw_error = 0.0f;
}
