#include "IMU.h"
#define DATA_SIZE 200
float safe_asin(float v);

volatile float exInt, eyInt, ezInt;  
volatile float integralFBx, integralFBy, integralFBz;
volatile float q0, q1, q2, q3, qa0, qa1, qa2, qa3;
volatile float integralFBhand, handdiff;
volatile double halfT ,elapsedT;
volatile uint32_t lastUpdate, now;
volatile float acc_vector = 0;  //  M/S^2
volatile float IMU_Pitch, IMU_Roll, IMU_Yaw;

// Fast inverse square-root
/**************************ʵ�ֺ���********************************************

*******************************************************************************/
float invSqrt(float x)
{

	volatile float halfx = 0.5f * x;
	volatile float y = x;
	long i = *(long *)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/*********************************************************************
Initialise MPU6050 and HMC5883, configure gyro and acc readings.

*******************************************************************************/
float offset_gx, offset_gy, offset_gz;
float offset_ax, offset_ay, offset_az;
void IMU_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    // I2C
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	// DRY  - PB 13
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	MPU6050_initialize();

	HMC5883L_SetUp();
	delay_ms(20);
}

/**************************ʵ�ֺ���********************************************

*******************************************************************************/
#define new_weight 0.4f
#define old_weight 0.6f
/*
Get values from sensors
MPU6050 returns 6 values.
0, 1, 2 are accelemeter data
3 4 5 are gyro data.

*/
static int16_t MPU6050_raw[6];
void IMU_getValues(float *values)
{
	int16_t accgyroval[6];
	static  volatile float lastacc[3] = {0, 0, 0};
	uint8_t i;
	//
	MPU6050_getMotion6(&accgyroval[0], &accgyroval[1], &accgyroval[2], &accgyroval[3], &accgyroval[4], &accgyroval[5]);
	// added to watch the 'raw' values
	for (i=0;i<6;i++)
	{
	MPU6050_raw[i]=accgyroval[i];
	}
	
	for(i = 0; i < 6; i++)
	{
		if(i < 3)
		{
			// Complementary Filter for new accelemeter data from MPU6050
			values[i] = (float) accgyroval[i] * new_weight + lastacc[i] * old_weight ;
			lastacc[i] = values[i];
		}
		else
		{
			// raw to real deg/sec -> *2000/32767 = 1/16.4
			values[i] = ((float) accgyroval[i]) / 16.4f; 
		}
	}
	/* pass the address of values[6], after this function finishes, values[6],[7],[8] are filled with mag data
	*/
	HMC58X3_mgetValues(&values[6]);	//

}

/***************************************************************************
IMU_data is used by all function
*******************************************************************************/
//float acc_z_comp;
float acc_x,acc_y,acc_z;
static float IMU_data[9];
void IMU_getAttitude(float *RPY,float *rate_RPY, float *cf_accZ)
{
		float gx,gy,gz;
    static float rateroll,ratepitch,rateyaw;
    float tong;
    float gyroXrate,gyroYrate;
    double angle_roll_acc,angle_pitch_acc;
    double kalAngleX, kalAngleY;
	/* This function to get the scaled values from GY-86 should be call here once. 
	used by all algorithms
	*/

	IMU_getValues(IMU_data);
	/*
	time elapsed should be calculated right after IMU data is read
	*/
	now = micros(); 
	if(now < lastUpdate) 
	{
		elapsedT =  (float)(now + (0xffffffff - lastUpdate));
	}
	else
	{
		elapsedT =  (now - lastUpdate);
	}
	//convert us to second
	elapsedT = elapsedT * 0.000001f;
	halfT = elapsedT / 2.0f;
	lastUpdate = now;
	
    acc_x=IMU_data[0];
    acc_y=IMU_data[1];
    acc_z=IMU_data[2];
    gx=MPU6050_raw[3] - offset_gx;
    gy=MPU6050_raw[4] - offset_gy;
    gz=MPU6050_raw[5] - offset_gz;
    // Like IMU_getRATERollPitchYaw but rewrite later
    rateroll=LPF(gx,rateroll,10,elapsedT);
    ratepitch=LPF(gy,ratepitch,10,elapsedT);
    rateyaw=LPF(gz,rateyaw,10,elapsedT);
	
		rate_rpy[0] = rate_rpy[0] * 0.85f + rateroll * 0.15f/16.4f;
	  rate_rpy[1] = rate_rpy[1] * 0.85f + ratepitch * 0.15f/16.4f;
	  rate_rpy[2] = rate_rpy[2] * 0.85f + rateyaw * 0.15f/16.4f;
	
    gyroXrate = gx / 16.4f;
    gyroYrate = gy / 16.4f;

	  /*
		angle += rate * elapsedTime * ( sensor conversion coeffient )
		T*2000/32767 = ...
	  */
    RPY[0] += rateroll * elapsedT * 2000/32767;// 
    RPY[1] += ratepitch * elapsedT * 2000/32767;
    RPY[2] += rateyaw * elapsedT * 2000/32767;
    //(1/250/65.5)*pi/180=0.000001066
		// elapsedT * ( sensor conversion coeffient ) * M_PI/180.0f
    RPY[1]-= RPY[0]*sin(rateyaw * elapsedT * (2000/32767) * M_PI/180);
    RPY[0]+= RPY[1]*sin(rateyaw * elapsedT * (2000/32767) * M_PI/180);

    tong = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
    if(abs(acc_x) < tong) {
        angle_pitch_acc=-asin((float)acc_x/tong)*57.296f;
    }
    if(abs(acc_y) < tong) {
        angle_roll_acc=asin((float)acc_y/tong)*57.296f;
    }

    kalAngleX = Kalman_getAngle_roll(angle_roll_acc,gyroXrate, elapsedT);
    kalAngleY = Kalman_getAngle_pitch(angle_pitch_acc,gyroYrate, elapsedT);

    RPY[1] = RPY[1] * 0.975f +  kalAngleY * 0.025f;
    RPY[0] =  RPY[0] * 0.975f +   kalAngleX * 0.025f ;
//		acc_z_comp = acc_z_comp*0.9 + acc_z*0.1;
//		*rate_z += acc_z_comp*elapsedT;
	  *cf_accZ = *cf_accZ*0.974f + 0.036f*(MPU6050_raw[2] - offset_az);
	  *cf_accZ/=2;
}

float LPF(float x, float pre_value, float CUTOFF,float dt)
{
    float RC, alpha, y;
    RC = 1.0f/(CUTOFF*2*3.1416f);
    alpha = dt/(RC+dt);
    y = pre_value + alpha * ( x - pre_value );
    return y;
}

float safe_asin(float v)
{
	if (isnan(v))
	{
		return 0.0f;
	}
	if (v >= 1.0f)
	{
		return M_PI / 2;
	}
	if (v <= -1.0f)
	{
		return -M_PI / 2;
	}
	return asin(v);
}
