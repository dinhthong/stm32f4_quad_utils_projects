#include "common.h"
#include <math.h>

//#define Optical_Flow
//#define usart_debug

//#define gps

//#define channels[0] channels[0]
//#define channels[1] channels[1]
//#define channels[2] channels[2]
//#define channels[3] channels[3]

//#define channels[0] channels[0]
//#define channels[1] channels[1]
//#define channels[2] channels[2]
//#define channels[3] channels[3] // CH4

//#define rc_gear channels[4]
//#define rc_mix channels[5]
//#define rc_aux channels[6]
//#define TAKEOFF_ALT 70.0f
//#define MAX_ALT_SP 140.0f

//#define raw_P_buffer_size 10
//#define raw_T_buffer_size 5
//#define ms5611_vel_buffer_size 25

//#define MOTOR_SPIN 1230
/* define the range that the copter will most likely to remain altitude.
note that as the smaller battery get, the value it requires to takeoff quadcopter increases.

*/
//#define HOLD_THROTTLE_MIN 1497
//#define HOLD_THROTTLE_MAX 1534

//#define TAKEOFF_THROTTLE_MAX 1620

//#define DETECT_TAKEOFF_ALT 18.0f

#define Motor_MIN 1050
#define Motor_MAX 1650

//void get_ms5611(void);
//void getRate_SetPoint(uint16_t RC_roll, 
//	                    uint16_t RC_pitch, 
//	                    uint16_t RC_yaw,
//	                    float *angles_rpy, 
//	                    float *rate_rpy_setpoint);
//void calculate_motor_output(uint16_t *motor_out, 
//	                          uint16_t motor_throttle, 
//                            uint16_t motor_roll, 
//                            uint16_t motor_pitch, 
//                            uint16_t motor_yaw);
//void stabilize_mode(int16_t *Stabilize_Thro,
//                    int16_t *Stabilize_Roll,
//                    int16_t *Stabilize_Pitch,
//                    int16_t *Stabilize_Yaw);


//void sonar_pid_alt(float altitude_setpoint, float altitude_input, float *AltPID_Out);
//void main_loop(void);
//void usart_printf(void);
//void sonar_reset_pid_alt(void);
//void baro_reset_pid_alt(void);
//void adns380_update(void);
void EXTILine_Config(void);
//void set_altsp(float setpoint);
//void baro_pid_alt(void);
//uint8_t read_mode(void);
extern float offset_gx,offset_gy,offset_gz;
extern float ct;
///*
//will be change later
//*/
//#define GPS_LAT_CENTER
//#define GPS_LON_CENTER
///*
//10km = 10 000 00 cm
//-> about 10^6/11 = 90000 unit.

//*/
//#define GPS_RADIUS

//#ifdef gps
//void gps_parsing(void);
//#endif
