#include "pid.h"


//pid_para_t rate_roll = {.54f, .008f, 17.6f};

/* 
PID para with brokking IMU code: {0.54f, 1.7f, .0704f};
with quaternion based algorithm the PID must be changed for better result.

*/

//brokking. P = 1.3, I = 10,  D = 0.072. Original: 1.3 | 0.04 | 18.0 


//pid_para_t rate_roll = {0.54f, 1.5f, .0604f};
//pid_para_t rate_pitch = {0.54f, 1.5f, .0604f};

//pid_para_t rate_roll = {1.2f, 1.5f, .0704f};
//pid_para_t rate_pitch = {1.2f, 1.5f, .0704f};

//pid_para_t rate_roll = {1.0f, 1.3f, .0304f};
//pid_para_t rate_pitch = {1.0f, 1.3f, .0304f};

//pid_para_t rate_roll = {0.54f, 1.3f, .0504f};
//pid_para_t rate_pitch = {0.54f, 1.3f, .0504f};

// vip PID
pid_para_t rate_roll = {0.54f, 0.0052f, 12.6f};
pid_para_t rate_pitch = {0.54f, 0.0052f, 12.6f};

//pid_para_t rate_roll = {0.45f, 0.0052f, 11.8f};
//pid_para_t rate_pitch = {0.45f, 0.0052f, 11.8f};

//pid_para_t rate_roll = {1.3f, 0.04f, 15.6f};
//pid_para_t rate_pitch = {1.3f, 0.04f, 15.6f};

// brokking
/*

float pid_p_gain_roll = 1.3;               //Gain setting for the pitch and roll P-controller (default = 1.3).
float pid_i_gain_roll = 0.04;              //Gain setting for the pitch and roll I-controller (default = 0.04).
float pid_d_gain_roll = 18.0;   
*/
pid_para_t rate_yaw = {3.3f, 2.0f, 0.0f};


//float Kp_Rate_Roll_PID=.54,Ki_Rate_Roll_PID=.004,Kd_Rate_Roll_PID=17.6;
//float Kp_Rate_Pitch_PID=.54,Ki_Rate_Pitch_PID=.004, Kd_Rate_Pitch_PID=17.6;
//float Kp_Rate_Yaw_PID=.54,Ki_Rate_Yaw_PID=.004, Kd_Rate_Yaw_PID=17.6;

float Rate_Roll_PID, Rate_Pitch_PID, Rate_Yaw_PID;
// Must remain values
float Pre_Rate_Roll_error, Pre_Rate_Pitch_error, Pre_Rate_Yaw_error;
float Rate_Roll_PID_I, Rate_Pitch_PID_I, Rate_Yaw_PID_I;
float Rate_Roll_PID_D, Rate_Pitch_PID_D, Rate_Yaw_PID_D;
float Rate_Roll_error,Rate_Pitch_error,Rate_Yaw_error;
/*
+ PID depends on dt. Because the loop is constant time so we temporarily don't take it into account.
+ Apply LPF.
*/

void PID_RollPitchYaw(int16_t *PID_Roll_Out, int16_t *PID_Pitch_Out, int16_t *PID_Yaw_Out) {
    
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
    Rate_Roll_PID_I += rate_roll.i *Rate_Roll_error;
		Rate_Roll_PID_I = Math_fConstrain(Rate_Roll_PID_I, -30.0f, 30.0f);
		Rate_Roll_PID_I *= RP_KI_Enable;
		Rate_Roll_PID_D = rate_roll.d*(Rate_Roll_error - Pre_Rate_Roll_error);
	//  Rate_Roll_PID_D = Math_fConstrain(Rate_Roll_PID_D, -120.0f, 120.0f);
    Rate_Roll_PID = rate_roll.p * Rate_Roll_error + Rate_Roll_PID_I + Rate_Roll_PID_D;
	//	Rate_Roll_PID = Math_fConstrain(Rate_Roll_PID, RP_PID_MIN, RP_PID_MAX);
    Pre_Rate_Roll_error = Rate_Roll_error;

    //Pitch calculations
    Rate_Pitch_error = rate_rpy_setpoint[1] - rate_rpy[1] ;
    Rate_Pitch_PID_I += rate_pitch.i * Rate_Pitch_error;
		
		Rate_Pitch_PID_I = Math_fConstrain(Rate_Pitch_PID_I, -30.0f, 30.0f);
		Rate_Pitch_PID_I *= RP_KI_Enable;
		Rate_Pitch_PID_D = rate_pitch.d * (Rate_Pitch_error - Pre_Rate_Pitch_error);
	//	Rate_Pitch_PID_D = Math_fConstrain(Rate_Pitch_PID_D, -120.0f, 120.0f);
    Rate_Pitch_PID = rate_pitch.p * Rate_Pitch_error + Rate_Pitch_PID_I + Rate_Pitch_PID_D;
//		Rate_Pitch_PID = Math_fConstrain(Rate_Pitch_PID, RP_PID_MIN, RP_PID_MAX);
    Pre_Rate_Pitch_error = Rate_Pitch_error;
		
    //This should be reconsider 
    Rate_Yaw_error = rate_rpy_setpoint[2] - rate_rpy[2] ;
    Rate_Yaw_PID_I += rate_yaw.i * (Rate_Yaw_error + Pre_Rate_Yaw_error)*dt/2.0f;
		Rate_Yaw_PID_I = Math_fConstrain(Rate_Yaw_PID_I, -30.0f, 30.0f);
		Rate_Yaw_PID_I *= 1;
		Rate_Yaw_PID_D = rate_pitch.d * (Rate_Yaw_error - Pre_Rate_Yaw_error)/dt;
    Rate_Yaw_PID = rate_yaw.p * Rate_Yaw_error + Rate_Yaw_PID_I + Rate_Yaw_PID_D;
		Rate_Yaw_PID = Math_fConstrain(Rate_Yaw_PID, -60.0f, 60.0f);
		Pre_Rate_Yaw_error = Rate_Yaw_error;
		Rate_Yaw_PID = 0.0f;
	  *PID_Roll_Out = (int16_t)round(Rate_Roll_PID);
  	*PID_Pitch_Out = (int16_t)round(Rate_Pitch_PID);
  	*PID_Yaw_Out = (int16_t)round(Rate_Yaw_PID);
}
		
void reset_rate_PID(void) {
    Rate_Roll_PID = Rate_Pitch_PID = Rate_Yaw_PID = 0.0f;
    Rate_Roll_PID_I = Rate_Pitch_PID_I = Rate_Yaw_PID_I = 0.0f;
    Pre_Rate_Roll_error = Pre_Rate_Pitch_error = Pre_Rate_Yaw_error = 0.0f;
}
