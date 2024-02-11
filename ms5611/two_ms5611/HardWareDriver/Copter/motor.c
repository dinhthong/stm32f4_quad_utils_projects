#include "motor.h"
//#define motor_min 1050
//#define motor_max 1750
//uint16_t Math_u16_Constrain(uint16_t value, uint16_t min, uint16_t max);
void send_pwm_motor(uint16_t motor_1,uint16_t motor_2,uint16_t motor_3,uint16_t motor_4)
{
				TIM1->CCR1 = motor_1;
				TIM1->CCR2 = motor_2;
				TIM1->CCR3 = motor_3;
				TIM1->CCR4 = motor_4;
}
