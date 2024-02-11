#include "common.h"
#include <math.h>
//void get_Baro(void);
void read_barometer(void);
void read_barometer0(void);
void calculate_pid_altitude(void);
void getRate_SetPoint(uint16_t RC_roll, 
	                    uint16_t RC_pitch, 
	                    uint16_t RC_yaw,
	                    float *angles_rpy, 
	                    float *rate_rpy_setpoint);
void calculate_motor_output(uint16_t *motor_out, 
	                          uint16_t motor_throttle, 
                            uint16_t motor_roll, 
                            uint16_t motor_pitch, 
                            uint16_t motor_yaw);
void stabilize_mode(int16_t *Stabilize_Thro,
                    int16_t *Stabilize_Roll,
                    int16_t *Stabilize_Pitch,
                    int16_t *Stabilize_Yaw);

void althold_mode(int16_t *AltHold_Thro, 
	                int16_t *AltHold_Roll,
	                int16_t *AltHold_Pitch,
	                int16_t *AltHold_Yaw);
//void pid_altitude(float altitude_setpoint, float altitude_current,int16_t *AltPID_Out);
void main_loop(void);
void usart_printf(void);
void reset_pid_altitude(void);

#define rc_elev channels[0]
#define rc_aile channels[1]
#define rc_thro channels[2]
#define rc_rudd channels[3]
#define rc_gear channels[4]
#define rc_mix channels[5]
#define rc_aux channels[6]
uint8_t read_mode(void);
extern float offset_gx,offset_gy,offset_gz;
extern float offset_ax, offset_ay, offset_az;
