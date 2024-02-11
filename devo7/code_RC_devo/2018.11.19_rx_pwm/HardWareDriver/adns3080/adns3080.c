#include "adns3080.h"
#include "math.h"
#define ADNS_NCS_DESELECT GPIO_SetBits(GPIOE, GPIO_Pin_7)
#define ADNS_NCS_SELECT GPIO_ResetBits(GPIOE, GPIO_Pin_7)

unsigned char frame[ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y];
int pid;
void adns3080_reset(void);
void adns3080_spi_config(void) {
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	GPIO_InitTypeDef GPIO_InitStructt;
    SPI_InitTypeDef   SPI_InitStructure;
	// PB1 -> RST
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	  GPIO_InitStructt.GPIO_Pin =GPIO_Pin_1;
    GPIO_InitStructt.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructt.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructt.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructt.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOB, &GPIO_InitStructt);
	// PE7 -> NCS
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
		  GPIO_InitStructt.GPIO_Pin =GPIO_Pin_7;
    GPIO_InitStructt.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructt.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructt.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructt.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOE, &GPIO_InitStructt);
	
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
//	  GPIO_InitStructt.GPIO_Pin =GPIO_Pin_4;
//    GPIO_InitStructt.GPIO_Mode = GPIO_Mode_OUT;
//    GPIO_InitStructt.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructt.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructt.GPIO_PuPd = GPIO_PuPd_NOPULL;
//    GPIO_Init(GPIOC, &GPIO_InitStructt);
  /* SPI_MASTER configuration ------------------------------------------------*/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
  GPIO_InitStructt.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructt.GPIO_Mode = GPIO_Mode_AF;
  GPIO_Init(GPIOA, &GPIO_InitStructt);
  
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_SPI1); // SCK
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_SPI1); // MISO
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_SPI1); // MOSI
  
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	// spi mode 3.
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	
  SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  SPI_Init(SPI1, &SPI_InitStructure);
  SPI_Cmd(SPI1, ENABLE);
}
	int raw_dx, raw_dy;   // raw sensor change in x and y position (i.e. unrotated)
	int surface_quality;  // image quality (below 15 you really can't trust the x,y values returned)
//	int x,y;              // total x,y position
	int dx,dy;            // rotated change in x and y position
    float vlon, vlat;       // position as offsets from original position
	unsigned long last_update;    // millis() time of last update
int num_pixels;
float field_of_view;  // field of view in Radians
float scaler;        // number returned from sensor when moved one pixel
	// temp variables - delete me!
    float exp_change_x, exp_change_y;
    float change_x, change_y;
	float x_cm, y_cm;
/*
return 0 if init successfully
*/
int8_t mousecam_init(void) {
	ADNS_NCS_DESELECT;
	adns3080_reset();
	pid = mousecam_read_reg(ADNS3080_PRODUCT_ID);
  if(pid != ADNS3080_PRODUCT_ID_VAL)
    return -1;
  // turn on sensitive mode
  mousecam_write_reg(ADNS3080_CONFIGURATION_BITS, 0x19);
  return 0;
}
int adns_val;
struct MD adns_md;
/*
Correspond to 
optflow.update(); == AP_OpticalFlow_ADNS3080::update()
*/
void adns3080_getdata(void) {
	uint8_t motion_reg;
//	float adns_x, adns_y, adns_temp_x, adns_temp_y;
/*
The idea is based on http://ardupilot.org/copter/docs/common-mouse-based-optical-flow-sensor-adns3080.html
Note that after the conversion, adns_x will be distance but with unknown unit. But surely it can be fed into PID
+ Should be update only after checking the sensor's quality data values. -> future improvement.

*/
	adns_val = mousecam_read_reg(ADNS3080_PIXEL_SUM);
////  
  mousecam_read_motion(&adns_md);
	surface_quality = adns_md.squal;
	motion_reg = adns_md.motion;
	
//	_overflow = ((motion_reg & 0x10) != 0);  // check if we've had an overflow
	if( (motion_reg & 0x80) != 0 ) {
		raw_dx = adns_md.dx;
	//	delay_ms(50);  // small delay
		raw_dy = adns_md.dy;
	//	_motion = true;
	}else{
	    raw_dx = 0;
		  raw_dy = 0;
	}
	
		//	delay_ms(100);
//	adns_md.x= adns_md.dx*dt;
//	adns_md.y= adns_md.dy*dt;
//	
//	adns_temp_x = -adns_md.dx*dt;
//	adns_temp_y = -adns_md.dy*dt;
//	  // angle compensation
//	adns_temp_x = - rpy[0]*dt;
//	adns_temp_y = - rpy[1]*dt;
//	// altitude compensation
//	adns_temp_x = adns_temp_x*alt_sonar;
//	adns_temp_y = adns_temp_y*alt_sonar;
//	
//	adns_x+=adns_temp_x;
//	adns_y+=adns_temp_y;
}

void adns3080_setup(void) {
  num_pixels = ADNS3080_PIXELS_X;
	field_of_view = AP_OPTICALFLOW_ADNS3080_08_FOV;
	scaler = AP_OPTICALFLOW_ADNS3080_SCALER;
	
//	adns3080_set_orientation(OPTFLOW_ORIENTATION);			// set optical flow sensor's orientation on aircraft
	
	
//	adns3080_set_frame_rate(2000);							// set minimum update rate (which should lead to maximum low light performance
	
		// set specific frame period
	mousecam_write_reg(ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER,0xE0);
	delay_us(50);  // small delay
	mousecam_write_reg(ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER,0x1A);
	adns3080_set_resolution(OPTFLOW_RESOLUTION);				// set optical flow sensor's resolution
	
//	adns3080_set_field_of_view(OPTFLOW_FOV);					// set optical flow sensor's field of view
	
	//field_of_view = AP_OPTICALFLOW_ADNS3080_08_FOV; 
	
	update_conversion_factors();
	

}
void adns3080_reset(void)
{
	GPIO_SetBits(GPIOB, GPIO_Pin_1);
  delay_ms(1); // reset pulse >10us
	GPIO_ResetBits(GPIOB, GPIO_Pin_1);
  delay_ms(35); // 35ms from reset to functional
}



// pdata must point to an array of size ADNS3080_PIXELS_X x ADNS3080_PIXELS_Y
// you must call mousecam_reset() after this if you want to go back to normal operation
int mousecam_frame_capture(unsigned char *pdata)
{
  mousecam_write_reg(ADNS3080_FRAME_CAPTURE,0x83);
  ADNS_NCS_SELECT;
  SPI_transfer(ADNS3080_PIXEL_BURST);
  delay_us(50);
  
  int pix;
  unsigned char started = 0;
  int count;
  int timeout = 0;
  int ret = 0;
  for(count = 0; count < ADNS3080_PIXELS_X * ADNS3080_PIXELS_Y; )
  {
    pix = SPI_transfer(0xff);
    delay_us(10);
    if(started==0)
    {
      if(pix&0x40)
        started = 1;
      else
      {
        timeout++;
        if(timeout==100)
        {
          ret = -1;
          break;
        }
      }
    }
    if(started==1)
    {
      pdata[count++] = (pix & 0x3f)<<2; // scale to normal grayscale byte range
    }
  }
	ADNS_NCS_DESELECT;
  delay_us(14);
  
  return ret;
}

void mousecam_write_reg(int reg, int val)
{  
	ADNS_NCS_SELECT;
	delay_us(5);
	SPI_transfer(reg| 0x80);
	SPI_transfer(val);
	ADNS_NCS_DESELECT;
	delay_us(5);
}
int mousecam_read_reg(int Address)
{
  int val;
	ADNS_NCS_SELECT;
	delay_us(2);
	SPI_transfer(Address);
	delay_us(83);
	val = SPI_transfer(0xff);
	ADNS_NCS_DESELECT;
	delay_us(1);
	return val;
}
unsigned char SPI_transfer(unsigned char data){

	uint32_t val;
	//This function is just exactly the same as void 'SPI_I2S_SendData(SPI_TypeDef* SPIx, uint16_t Data)'
	SPI1->DR = data; // write data to be transmitted to the SPI data register
	while( !(SPI1->SR & SPI_I2S_FLAG_TXE) ); // wait until transmit complete
	while( !(SPI1->SR & SPI_I2S_FLAG_RXNE) ); // wait until receive complete
	while( SPI1->SR & SPI_I2S_FLAG_BSY ); // wait until SPI is not busy anymore
	return SPI1->DR; // return received data from SPI data register
}

void mousecam_read_motion(struct MD *p)
{
	ADNS_NCS_SELECT;
  SPI_transfer(ADNS3080_MOTION_BURST);
  delay_us(75);
  p->motion =  SPI_transfer(0xff);
  p->dx =  SPI_transfer(0xff);
  p->dy =  SPI_transfer(0xff);
  p->squal =  SPI_transfer(0xff);
  p->shutter =  SPI_transfer(0xff)<<8;
  p->shutter |=  SPI_transfer(0xff);
  p->max_pix =  SPI_transfer(0xff);
	ADNS_NCS_DESELECT;
  delay_us(5);
}
/*
AP_OpticalFlow_ADNS3080.cpp

*/
void adns3080_set_resolution(int resolution)
{
    uint8_t regVal = mousecam_read_reg(ADNS3080_CONFIGURATION_BITS);

    if( resolution == ADNS3080_RESOLUTION_400 )	{
	    regVal &= ~0x10;
		scaler = AP_OPTICALFLOW_ADNS3080_SCALER;
	}else if( resolution == ADNS3080_RESOLUTION_1600) {
	    regVal |= 0x10;
		scaler = AP_OPTICALFLOW_ADNS3080_SCALER * 4;
	}

	delay_us(50);  // small delay
	mousecam_write_reg(ADNS3080_CONFIGURATION_BITS, regVal);

	// this will affect conversion factors so update them
	update_conversion_factors();
}
	float conv_factor; // multiply this number by altitude and pixel change to get horizontal move (in same units as altitude)
    float radians_to_pixels;
void update_conversion_factors(void)
{
	conv_factor = (1.0 / (float)(num_pixels * scaler)) * 2.0 * tan(field_of_view / 2.0);	// multiply this number by altitude and pixel change to get horizontal move (in same units as altitude)
	// 0.00615
	radians_to_pixels = (num_pixels * scaler) / field_of_view;
	// 162.99
}

float last_roll,last_pitch,last_altitude ;
void adns3080_update_position(float roll, float pitch, float cos_yaw_x, float sin_yaw_y, float altitude)
{
	//static 
    float diff_roll 	= roll  - last_roll;
    float diff_pitch 	= pitch - last_pitch;

	// only update position if surface quality is good and angle is not over 45 degrees
	if( surface_quality >= 10 && fabs(roll) <= FORTYFIVE_DEGREES && fabs(pitch) <= FORTYFIVE_DEGREES ) {
	//	altitude = max(altitude, 0);
		// calculate expected x,y diff due to roll and pitch change
		exp_change_x = diff_roll * radians_to_pixels;
		exp_change_y = -diff_pitch * radians_to_pixels;

		// real estimated raw change from mouse
		change_x = dx - exp_change_x;
		change_y = dy - exp_change_y;

		float avg_altitude = (altitude + last_altitude)*0.5f;

		// convert raw change to horizontal movement in cm
		x_cm = -change_x * avg_altitude * conv_factor;    // perhaps this altitude should actually be the distance to the ground?  i.e. if we are very rolled over it should be longer?
		y_cm = -change_y * avg_altitude * conv_factor;    // for example if you are leaned over at 45 deg the ground will appear farther away and motion from opt flow sensor will be less

		// convert x/y movements into lon/lat movement
		vlon = x_cm * sin_yaw_y + y_cm * cos_yaw_x;
		vlat = y_cm * sin_yaw_y - x_cm * cos_yaw_x;
	}

	last_altitude = altitude;
	last_roll = roll;
	last_pitch = pitch;
}

//void set_frame_period(unsigned int period)
//{
//    NumericIntType aNum;
//	aNum.uintValue = period;

//	// set frame rate to manual
//	//set_frame_rate_auto(false);
//	delay_us(50);  // small delay

//	// set specific frame period
//	write_register(ADNS3080_FRAME_PERIOD_MAX_BOUND_LOWER,aNum.byteValue[0]);
//	delayMicroseconds(50);  // small delay
//	write_register(ADNS3080_FRAME_PERIOD_MAX_BOUND_UPPER,aNum.byteValue[1]);

//}

int get_resolution(void)
{
    if( (mousecam_read_reg(ADNS3080_CONFIGURATION_BITS) & 0x10) == 0 )
	    return 400;
	  else
	    return 1600;
}
