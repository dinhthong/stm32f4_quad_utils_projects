/*
1. add some calculation for barometer before main loop.
2. get baro as a function.
3. convert to exact brokking's formula.
4. usart interrupt receipt for PID tuning.
5. handle ms5611 function warning

First Arm in stabilize mode, move throttle to hover. Then switch to alt hold if want.
Disarm in any mode.
Don't arm in alt hold mode. it will behave unexpectedly.

*/
#include "main.h"

void Initial_System_Timer(void)
{
	RCC->APB1ENR |= 0x0008;	
	TIM5->CR1 = 0x0080; //TIMx_ARR buffered  upcounter
	TIM5->CR2 = 0x0000;
	TIM5->CNT = 0x0000;
	TIM5->ARR = 0xFFFFFFFF;
	TIM5->PSC = 84 - 1;	//1us
	TIM5->EGR = 0x0001;
	TIM5->CR1 |= 0x0001; // Enable
}

// LPF
float cut_off=10;

float rpy[3], rate_rpy[3]; //roll pitch yaw 
float rate_rpy_setpoint[3];
//float rate_alt;
uint16_t Motors[4];
uint16_t temp_throttle;
int16_t Motor_throttle, Motor_roll, Motor_pitch, Motor_yaw;
uint8_t flight_mode;
uint8_t arm, alt_hold_status = 200;
int8_t calib_count = 0;
uint16_t j;
uint32_t loop_cnt;

float cf_acc_z;
long realPressure, realPressure0;
double dt;
// brokking's baro variables.
//float pid_p_gain_altitude =1.2f, pid_i_gain_altitude = 0.002f, pid_d_gain_altitude = 1.5;
float pid_p_gain_altitude =3.2f, pid_i_gain_altitude = 0.008f, pid_d_gain_altitude = 1.5;
uint8_t temperature_index;
float actual_pressure, pressure_parachute_previous, actual_pressure_slow, actual_pressure_fast, actual_pressure_diff;
float pid_altitude_input, pid_altitude_setpoint;
float pid_error_temp, pid_error_gain_altitude;
uint32_t raw_pressure, raw_temperature, temp, raw_temperature_rotating_memory[6], raw_average_temperature_total;
int32_t pressure_rotating_mem[50], parachute_buffer[30], pressure_total_avarage;
uint8_t pressure_rotating_mem_location, parachute_rotating_mem_location;
float pressure_rotating_mem_actual, parachute_throttle;
uint8_t barometer_counter, temperature_counter;
float brokking_estalt, actual_pressurechange;
float pid_i_mem_altitude;
double alt_pid_dt;
uint32_t alt_pid_loop;
int16_t pid_output_altitude;
int16_t Thro_Altitude;
int main(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 
    delay_init(168);
    delay_ms(200);
    board_leds_config();
	  usart_printf_config(57600);
    Initial_System_Timer();
	// in case no RC input at beginning.
	//  rc_thro = rc_aile = rc_elev = rc_rudd = 1500;
    IMU_init();
	
	//ms5611 IOIIC 1
	    GPIO_InitTypeDef GPIO_InitStructure;
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
//    // I2C
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
//	  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	

		
		
	  // ms5611 I2C. 0
	
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    // I2C
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOD, &GPIO_InitStructure);
		
	  begin();
	 // begin0();
		
//    TIM_PWM_Configuration();
//    exti_gpio_config();
//    delay_ms(100);
		
		
//		printf(" Simple imu, alt hold, based on brokking code! \n\r");

    // assign dummy data for the first while loop returns 1. this can be achieved alternatively 
    // without set dummy data using do...while loop
//    rpy[0] = rpy[1] = rpy[2] = rate_rpy[0] = rate_rpy[1] = rate_rpy[2] = 55.55f;
//    while (fabs(rate_rpy[0]) > 0.15f || fabs(rate_rpy[1]) > 0.15f || fabs(rate_rpy[2]) > 0.15f)
//		{
//	  MPU6050_Calculate_MPU6050_Offset(&offset_ax, &offset_ay, &offset_az,&offset_gx,&offset_gy,&offset_gz,1000);
//	  dt = 0.004;
//		rpy[0] = rpy[1] = rpy[2] = 0.0f;
//		for (j=0; j <= 200; j++){
//		//	get_Baro();
//			read_barometer();
//			// calculate Attitude also
//			IMU_getAttitude(rpy, rate_rpy, &cf_acc_z);
//			delay_ms(4);
//		}
//		calib_count++;
//	}
//    ms5611_altitude_offset = EstAlt;
	
 //   Motors[0] = Motors[1] = Motors[2] = Motors[3] = 1000;
 
 
		tim4_config();
	//	delay_ms(200);
		loop_cnt = micros();
		tim3_config();
        while(1) {
    }
}

void TIM3_IRQHandler(void) {
	 if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)
  {
		        GPIO_ToggleBits(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);

		        read_barometer();
		 //       read_barometer0();
			 
            dt=(micros()-loop_cnt)*0.000001;
            loop_cnt = micros();
						TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
}

//void main_loop(void)
//{
//			switch (flight_mode){
//          case 1:
//					// Auto landing code
//					    break;
//				  case 2: 
//              /*
//					    */
//          //    getRate_SetPoint(rc_aile, rc_elev, rc_rudd, rpy, rate_rpy_setpoint);
//					    stabilize_mode(&Motor_throttle, &Motor_roll, &Motor_pitch, &Motor_yaw);
//					    break;
//				  case 3:
//					// Altitude hold mode

//				//	    getRate_SetPoint(rc_aile, rc_elev, rc_rudd, rpy, rate_rpy_setpoint);
//					    althold_mode(&Motor_throttle, &Motor_roll, &Motor_pitch, &Motor_yaw);
//					    break;
//				  default:
//					;
//			}
//			
//			calculate_motor_output(&Motors[0] ,Motor_throttle ,Motor_roll ,Motor_pitch ,Motor_yaw);
//}

void TIM4_IRQHandler(void)
{
  static uint8_t printf_cnt=0;
  if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
  {
    if(++printf_cnt>12)
    {
			usart_printf();
      printf_cnt = 0;
    }
   TIM_ClearITPendingBit(TIM4, TIM_IT_Update); //update interrupt is set by the hardware, and it's cleared by the software.
  }
}
void usart_printf(void) {
	  // 4 + 3 + 3 + 4 + 3 +3
		printf("*%0.3f,%0.3f,%0.3f,%0.3f,",(float)(TIM5->CNT)*0.000001f, rpy[0], rpy[1], rpy[2]);
	  printf("%0.3f,%0.3f,%0.3f,",rate_rpy[0], rate_rpy[1], rate_rpy[2]);
		printf("%0.3f,%0.3f,%0.3f,",rate_rpy_setpoint[0], rate_rpy_setpoint[1], rate_rpy_setpoint[2]);
		printf("%0.1f,%0.1f,%0.1f,%0.1f,",(float)Motors[0], (float)Motors[1], (float)Motors[2], (float)Motors[3]);
		printf("%0.1f,%0.1f,%0.1f,",(float)Motor_roll, (float)Motor_pitch, (float)Motor_yaw);	
		printf("%0.1f,%0.1f,%0.1f,\r\n" ,(float)actual_pressure, (float)pid_i_mem_altitude, pid_output_altitude);	
}

//uint8_t read_mode(void)
//{
//	if ( rc_gear > 1300 ) // gear = 1900
//		return 3;
//	else
//		return 2;
//}

/*
Input: 
+ raw RC channels values for 3 axes.
+ Euler angles 
+ Input dt
Return: Rate ( Angular Velocity ) Setpoint in [deg/sec]

* Todos:
Scale the Setpoint. -> Math_constrain
*/

void getRate_SetPoint(uint16_t RC_roll, uint16_t RC_pitch, uint16_t RC_yaw, float *angles_rpy, float *rate_rpy_setpoint)
{
	float rate_roll_setpoint, rate_pitch_setpoint, rate_yaw_setpoint;

        rate_roll_setpoint = RC_roll - 1500;
        rate_roll_setpoint -= (angles_rpy[0]*13.0f);
        rate_roll_setpoint = 0.0004f * rate_roll_setpoint/dt;
		   	rate_roll_setpoint = Math_fConstrain(rate_roll_setpoint, -35.0f, 35.0f);
	
        rate_pitch_setpoint = RC_pitch - 1500;
        rate_pitch_setpoint -= (angles_rpy[1]*13.0f);
        rate_pitch_setpoint = 0.0004f * rate_pitch_setpoint / dt;
	      rate_pitch_setpoint = Math_fConstrain(rate_pitch_setpoint, -35.0f, 35.0f);
	
				// Check if thro is not in lowest point !
        if (rc_thro > 1200) {
				    rate_yaw_setpoint = RC_yaw - 1500;
//				  	rate_yaw_setpoint -= angles_rpy[2]*5.0f;
//            rate_yaw_setpoint = 0.0001f * rate_yaw_setpoint / dt;
					  rate_yaw_setpoint*=0.09f;
					  rate_yaw_setpoint = Math_fConstrain(rate_yaw_setpoint, -20.0f, 20.0f);
	      }
				else { 
				    rate_yaw_setpoint = 0.0f;
				}
				rate_rpy_setpoint[0] = rate_roll_setpoint;
				rate_rpy_setpoint[1] = rate_pitch_setpoint;
				rate_rpy_setpoint[2] = rate_yaw_setpoint;
}
/*

*/
#define Motor_MIN 1150
#define Motor_MAX 1800

void calculate_motor_output(uint16_t *motor_out, 
	                          uint16_t motor_throttle, 
                            uint16_t motor_roll, 
                            uint16_t motor_pitch, 
                            uint16_t motor_yaw) {
    motor_out[0] = motor_throttle - motor_roll + motor_pitch + motor_yaw;
    motor_out[1] = motor_throttle - motor_roll - motor_pitch - motor_yaw;
    motor_out[2] = motor_throttle + motor_roll - motor_pitch + motor_yaw;
    motor_out[3] = motor_throttle + motor_roll + motor_pitch - motor_yaw;
	
    motor_out[0] = Math_Constrain(motor_out[0], Motor_MIN, Motor_MAX);
    motor_out[1] = Math_Constrain(motor_out[1], Motor_MIN, Motor_MAX);
    motor_out[2] = Math_Constrain(motor_out[2], Motor_MIN, Motor_MAX);
    motor_out[3] = Math_Constrain(motor_out[3], Motor_MIN, Motor_MAX);
}

/*
Get throttle, use rpy, 
PID function
output Motor_throttle ,Motor_roll ,Motor_pitch ,Motor_yaw 
*/

void stabilize_mode(int16_t *Stabilize_Thro, int16_t *Stabilize_Roll, int16_t *Stabilize_Pitch, int16_t *Stabilize_Yaw) {
		float Thro;
	  
  	Thro = rc_thro;
  	*Stabilize_Thro = (int16_t)Thro;
		PID_RollPitchYaw(Stabilize_Roll, Stabilize_Pitch, Stabilize_Yaw);
}

void althold_mode(int16_t *AltHold_Thro, int16_t *AltHold_Roll, int16_t *AltHold_Pitch, int16_t *AltHold_Yaw) {
		uint16_t BaseThro; 
	
	  BaseThro = temp_throttle;
		// pid output is calculated when read_barometer() and flight mode == 3
  	 *AltHold_Thro = BaseThro + (int16_t)pid_output_altitude + Thro_Altitude;
	  // we also need TX control for roll pitch yaw.
	  PID_RollPitchYaw(AltHold_Roll, AltHold_Pitch, AltHold_Yaw);
}

void read_barometer(void) {
	   barometer_counter++;
     if (barometer_counter == 1) {
         if (temperature_counter == 0) {
                raw_average_temperature_total -= raw_temperature_rotating_memory[temperature_index];
                // Read Temperature data.
                raw_temperature_rotating_memory[temperature_index] = readRawTemperature();
                raw_average_temperature_total += raw_temperature_rotating_memory[temperature_index];
                temperature_index++;
                if (temperature_index == 5)temperature_index = 0;
             // raw_temperature
                ut = raw_average_temperature_total / 5;
            }
            else {
            //Get pressure data from MS-5611
            raw_pressure = readRawPressure();
            }			
						
            temperature_counter ++;
            if (temperature_counter == 20) {
                temperature_counter = 0;
                // request temperature conversion.
                ms5611WriteByte(MS5611_CMD_CONV_D2 + uosr);  
            }
            else {
                //Request pressure data conversion
                ms5611WriteByte(MS5611_CMD_CONV_D1 + uosr);      
            }
        }
					 
        if (barometer_counter == 2) {
            // ut is ok
            up = raw_pressure; //
            // read pressure using the formula.
            // the ut is calculated using rotating buffer average.
            realPressure = readPressure(0);
            //Let's use a rotating mem
            pressure_total_avarage -= pressure_rotating_mem[pressure_rotating_mem_location];    //Subtract the current memory position to make room for the new value.
            pressure_rotating_mem[pressure_rotating_mem_location] = realPressure;                          //Calculate the new change between the actual pressure and the previous measurement.
            pressure_total_avarage += pressure_rotating_mem[pressure_rotating_mem_location];    //Add the new value to the long term avarage value.
            pressure_rotating_mem_location++;                                                   //Increase the rotating memory location.
            if (pressure_rotating_mem_location == 20)pressure_rotating_mem_location = 0;        //Start at 0 when the memory location 20 is reached.
            actual_pressure_fast = (float)pressure_total_avarage / 20.0f;                        //Calculate the avarage pressure value of the last 20 readings.
            if(mili() < 5000)actual_pressure_slow = actual_pressure_fast;                     //Keep the slow and fast avareges the same for the first 5 seconds.

            actual_pressure_slow = actual_pressure_slow * (float)0.985f + actual_pressure_fast * (float)0.015f;
							
            actual_pressure_diff = actual_pressure_slow - actual_pressure_fast;
								// apply dead band.
            if (actual_pressure_diff > 8)actual_pressure_diff = 8;
            if (actual_pressure_diff < -8)actual_pressure_diff = -8;
            if (actual_pressure_diff > 1 || actual_pressure_diff < -1)actual_pressure_slow -= actual_pressure_diff / 6.0f;
            actual_pressure = actual_pressure_slow;
						brokking_estalt = 10*getAltitude(actual_pressure,101325);
        }
	
        if (barometer_counter == 3) {                                                         //When the barometer counter is 3
            barometer_counter = 0;                                                              //Set the barometer counter to 0 for the next measurements.
					
          // call pid calculator.
//					if (flight_mode == 3 && alt_hold_status == 1) {
//					calculate_pid_altitude();
//					}
        }
}

uint8_t temperature_index0;
float actual_pressure0, pressure_parachute_previous0, actual_pressure_slow0, actual_pressure_fast0, actual_pressure_diff0;
//float pid_altitude_input, pid_altitude_setpoint;
//float pid_error_temp, pid_error_gain_altitude;
uint32_t raw_pressure0, raw_temperature0, raw_temperature_rotating_memory0[6], raw_average_temperature_total0;
int32_t pressure_rotating_mem0[50], parachute_buffer0[30], pressure_total_avarage0;
uint8_t pressure_rotating_mem_location0, parachute_rotating_mem_location0;
float pressure_rotating_mem_actual0, parachute_throttle0;
uint8_t barometer_counter0, temperature_counter0;


void read_barometer0(void) {
	   barometer_counter0++;
     if (barometer_counter0 == 1) {
         if (temperature_counter0 == 0) {
                raw_average_temperature_total0 -= raw_temperature_rotating_memory0[temperature_index0];
                // Read Temperature data.
                raw_temperature_rotating_memory0[temperature_index0] = readRawTemperature0();
                raw_average_temperature_total0 += raw_temperature_rotating_memory0[temperature_index0];
                temperature_index0++;
                if (temperature_index0 == 5)temperature_index0 = 0;
             // raw_temperature
                ut0 = raw_average_temperature_total0 / 5;
            }
            else {
            //Get pressure data from MS-5611
            raw_pressure0 = readRawPressure0();
            }			
						
            temperature_counter0 ++;
            if (temperature_counter0 == 20) {
                temperature_counter0 = 0;
                // request temperature conversion.
                ms5611WriteByte0(MS5611_CMD_CONV_D2 + uosr);  
            }
            else {
                //Request pressure data conversion
                ms5611WriteByte0(MS5611_CMD_CONV_D1 + uosr);      
            }
        }
					 
        if (barometer_counter0 == 2) {
            // ut is ok
            up0 = raw_pressure0; //
            // read pressure using the formula.
            // the ut is calculated using rotating buffer average.
            realPressure0 = readPressure0(0);
            //Let's use a rotating mem
            pressure_total_avarage0 -= pressure_rotating_mem0[pressure_rotating_mem_location0];    //Subtract the current memory position to make room for the new value.
            pressure_rotating_mem0[pressure_rotating_mem_location0] = realPressure0;                          //Calculate the new change between the actual pressure and the previous measurement.
            pressure_total_avarage0 += pressure_rotating_mem0[pressure_rotating_mem_location0];    //Add the new value to the long term avarage value.
            pressure_rotating_mem_location0++;                                                   //Increase the rotating memory location.
            if (pressure_rotating_mem_location0 == 20)pressure_rotating_mem_location0 = 0;        //Start at 0 when the memory location 20 is reached.
            actual_pressure_fast0 = (float)pressure_total_avarage0 / 20.0f;                        //Calculate the avarage pressure value of the last 20 readings.
            if(mili() < 5000)actual_pressure_slow0 = actual_pressure_fast0;                     //Keep the slow and fast avareges the same for the first 5 seconds.

            actual_pressure_slow0 = actual_pressure_slow0 * (float)0.985f + actual_pressure_fast0 * (float)0.015f;
							
            actual_pressure_diff0 = actual_pressure_slow0 - actual_pressure_fast0;
								// apply dead band.
            if (actual_pressure_diff0 > 8)actual_pressure_diff0 = 8;
            if (actual_pressure_diff0 < -8)actual_pressure_diff0 = -8;
            if (actual_pressure_diff0 > 1 || actual_pressure_diff0 < -1)actual_pressure_slow0 -= actual_pressure_diff0 / 6.0f;
            actual_pressure0 = actual_pressure_slow0;
					//	brokking_estalt = 10*getAltitude(actual_pressure,101325);
        }
	
        if (barometer_counter0 == 3) {                                                         //When the barometer counter is 3
            barometer_counter0 = 0;                                                              //Set the barometer counter to 0 for the next measurements.
					
          // call pid calculator.
//					if (flight_mode == 3 && alt_hold_status == 1) {
//					calculate_pid_altitude();
//					}
        }
}


void calculate_pid_altitude(void) {
	        // in case throttle change when in alt hold mode. <1450 or >1650. 
	     if (abs(rc_thro - 1500) > 150) {
	        if (rc_thro > 1650) {
						  Thro_Altitude = rc_thro - 1500 - 150;
					}
					if (rc_thro < 1350) {
						  Thro_Altitude = rc_thro - 1500 + 150;
					}
			//		pid_altitude_setpoint = actual_pressure;
					reset_pid_altitude();
				}
			 else {
				  Thro_Altitude = 0;
					// can be used to calculate direction z axis. quad goes up -> negative.
					parachute_throttle -= parachute_buffer[parachute_rotating_mem_location];
					parachute_buffer[parachute_rotating_mem_location] = actual_pressure * 10 - pressure_parachute_previous;
					parachute_throttle += parachute_buffer[parachute_rotating_mem_location];
					pressure_parachute_previous = 10 * actual_pressure;
					parachute_rotating_mem_location++;
					if (parachute_rotating_mem_location == 30) parachute_rotating_mem_location = 0;
			
					// calculate additional P gain term. smaller if current_point nears setpoint.
					
					pid_altitude_input = actual_pressure;
					pid_error_temp = pid_altitude_input - pid_altitude_setpoint;
					pid_error_gain_altitude = 0;
					if (pid_error_temp > 10 || pid_error_temp < -10) {
						  pid_error_gain_altitude = (abs(pid_error_temp)-10)/20.0f;
						  if (pid_error_gain_altitude > 3) pid_error_gain_altitude = 3;
					}
				//  pid_i_mem_altitude += (pid_i_gain_altitude/10.0f)*pid_error_temp;
					pid_i_mem_altitude += (pid_i_gain_altitude)*pid_error_temp;
					// brokking = 400
					if (pid_i_mem_altitude > 350) pid_i_mem_altitude = 350;
					if (pid_i_mem_altitude < -350) pid_i_mem_altitude = -350;

           pid_output_altitude =  (pid_p_gain_altitude + pid_error_gain_altitude) * pid_error_temp + pid_i_mem_altitude + pid_d_gain_altitude*parachute_throttle;
				   // in brokking's code it's 400
					if (pid_output_altitude > 375 ) pid_output_altitude = 375;
					if (pid_output_altitude < -375) pid_output_altitude = -375;
					alt_pid_dt = (micros() - alt_pid_loop)*0.000001f;
					alt_pid_loop = micros();
			 }
				
}

void reset_pid_altitude(void) {
	  pressure_parachute_previous = actual_pressure * 10;
	  pid_altitude_setpoint = actual_pressure;
    
		pid_i_mem_altitude = 0.0f;
		pid_output_altitude = 0.0f;
		alt_pid_loop = micros();
}