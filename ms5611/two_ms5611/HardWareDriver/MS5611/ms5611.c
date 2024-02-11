#include "ms5611.h"
#include "common.h"
#include "Fmath.h"
	uint16_t fc[6];
	uint8_t ct;
	uint8_t uosr;
	int32_t TEMP2,timer,timer1;
	
	int64_t OFF2, SENS2;
	int pressureState=0;
  int ut,up;
void begin(void)
{
    reset();
    setOversampling(MS5611_ULTRA_HIGH_RES); // resolution
    delay_ms(100);
    readPROM(); // read calib parameter
}

void update_baro(float *temperature, long *pressure, float *alti) {	
  switch (pressureState){
    case 0:
	  pressureState = 1;
	//  newData = false;
	  timer = mili();
    ms5611WriteByte(MS5611_CMD_CONV_D2 + uosr);// chuyen doi gia tri            
    break;
    case 1:
      if (mili() - timer >= ct) {
        pressureState = 2;
	    	ut = readRawTemperature();	// gia tri tho
        *temperature = readTemperature(0);  // tinh lai gia tri nhiet do      
      }
    break;
    case 2:
      timer1 = mili();
       ms5611WriteByte(MS5611_CMD_CONV_D1 + uosr);       
      pressureState = 3;
    break;    
    case 3:			  
      if (mili() - timer1 >= ct) {	          
         up = readRawPressure(); // do gia tri ADC 
        *pressure = readPressure(0);
        *alti = getAltitude(*pressure,101325);
	//	newData = true;
				pressureState = 0;
      }
    break;
	default:
	break;
  }
}
// Set oversampling value
void setOversampling(int osr)
{
    switch (osr)
    {
	case MS5611_ULTRA_LOW_POWER:
	    ct = 1;
	    break;
	case MS5611_LOW_POWER:
	    ct = 2;
	    break;
	case MS5611_STANDARD:
	    ct = 3;
	    break;
	case MS5611_HIGH_RES:
	    ct = 5;
	    break;
	case MS5611_ULTRA_HIGH_RES:
	    ct = 10;
	    break;
    }

    uosr = osr;
}

// Get oversampling value
ms5611_osr_t getOversampling(void)
{
    return (ms5611_osr_t)uosr;
}

void reset(void)
{
    ms5611WriteByte(MS5611_CMD_RESET);
}

void readPROM(void)
{  uint8_t offset;
    for (offset = 0; offset < 6; offset++)
    {
	fc[offset] = readRegister16(MS5611_CMD_READ_PROM + (offset * 2));
    }
}


uint32_t readRawTemperature(void)
{
  return readRegister24(MS5611_CMD_ADC_READ);
}

uint32_t readRawPressure(void)
{
    return readRegister24(MS5611_CMD_ADC_READ);
}

int32_t readPressure(int compensation)
{   
		uint32_t P;
    uint32_t D1 = up;

    uint32_t D2 = ut;
    int32_t dT = D2 - (uint32_t)fc[4] * 256;

    int64_t OFF = (int64_t)fc[1] * 65536 + (int64_t)fc[3] * dT / 128;
    int64_t SENS = (int64_t)fc[0] * 32768 + (int64_t)fc[2] * dT / 256;

    if (compensation==1)
    {
	int32_t TEMP = 2000 + ((int64_t) dT * fc[5]) / 8388608;

	OFF2 = 0;
	SENS2 = 0;

	if (TEMP < 2000)
	{
	    OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
	    SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
	}

	if (TEMP < -1500)
	{
	    OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
	    SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
	}

	OFF = OFF - OFF2;
	SENS = SENS - SENS2;
    }

     P = (D1 * SENS / 2097152 - OFF) / 32768;

    return P;
}

double readTemperature(int compensation)
{
    uint32_t D2 = ut;
    int32_t dT = D2 - (uint32_t)fc[4] * 256;

    int32_t TEMP = 2000 + ((int64_t) dT * fc[5]) / 8388608;

    TEMP2 = 0;

    if (compensation==1)
    {
	if (TEMP < 2000)
	{
	    TEMP2 = (dT * dT) / (2 << 30);
	}
    }

    TEMP = TEMP - TEMP2;

    return ((double)TEMP/100);
}

// Calculate altitude from Pressure & Sea level pressure
double getAltitude(double pressure, double seaLevelPressure)
{
    return (44330.0f * (1.0f - pow((double)pressure / (double)seaLevelPressure, 0.1902949f)));
}

// Calculate sea level from Pressure given on specific altitude
double getSeaLevel(double pressure, double altitude)
{
    return ((double)pressure / pow(1.0f - ((double)altitude / 44330.0f), 5.255f));
}

// Read 16-bit from register (oops MSB, LSB)
uint16_t readRegister16(uint8_t reg)
{
    uint16_t value;
		u8 data[2];
	//uint17_t value;
	IIC2readBytes(MS5611_ADDRESS,reg,2,data);
     unsigned short msb=0;
  unsigned short lsb=0;
//  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
//  I2C_AcknowledgeConfig(I2C1,ENABLE);
//  I2C_GenerateSTART(I2C1,ENABLE);

//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
//  I2C_Send7bitAddress(I2C1, MS5611_ADDRESS, I2C_Direction_Transmitter);

//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

//  I2C_SendData(I2C1,reg);
//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

//  I2C_GenerateSTART(I2C1,ENABLE);
//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

//  I2C_Send7bitAddress(I2C1, MS5611_ADDRESS, I2C_Direction_Receiver);
//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
//  msb = I2C_ReceiveData(I2C1);

//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
//  lsb = I2C_ReceiveData(I2C1);

//  I2C_GenerateSTOP(I2C1,ENABLE);
//  I2C_AcknowledgeConfig(I2C1,DISABLE);

//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
//  I2C_ReceiveData(I2C1);
	msb=data[0];
	lsb=data[1];
  value= (msb << 8) | lsb;

    return value;
}

// Read 24-bit from register (oops XSB, MSB, LSB)
uint32_t readRegister24(uint8_t reg)
{
	   unsigned long msb=0;
  unsigned long lsb=0;
  unsigned long xsb=0;
	u8 data[3];
	uint32_t value;
	IIC2readBytes(MS5611_ADDRESS,reg,3,data);
    
//   unsigned long msb=0;
//  unsigned long lsb=0;
//  unsigned long xsb=0;
//  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
//  I2C_AcknowledgeConfig(I2C1,ENABLE);
//  I2C_GenerateSTART(I2C1,ENABLE);
//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

//  I2C_Send7bitAddress(I2C1, MS5611_ADDRESS, I2C_Direction_Transmitter);
//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

//  I2C_SendData(I2C1,reg);
//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

//  I2C_GenerateSTART(I2C1,ENABLE);
//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

//  I2C_Send7bitAddress(I2C1, MS5611_ADDRESS, I2C_Direction_Receiver);
//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
//  msb = I2C_ReceiveData(I2C1);

//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
//  lsb = I2C_ReceiveData(I2C1);

//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
//  xsb = I2C_ReceiveData(I2C1);

//  I2C_GenerateSTOP(I2C1,ENABLE);
//  I2C_AcknowledgeConfig(I2C1,DISABLE);

//  while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));
//  I2C_ReceiveData(I2C1);
	msb=data[0];
	lsb=data[1];
	xsb=data[2];
  value = (msb << 16) | (lsb << 8) | xsb;

    return value;
}
void ms5611WriteByte(unsigned char data)
{
	IICwriteByte(MS5611_ADDRESS, 0, data);
}

// ms5611 for second I2C. PB6, PB7.

	uint16_t fc0[6];
	uint8_t ct0;
	uint8_t uosr0;

	//int32_t TEMP2,timer,timer1;
	
	int64_t OFF2_0, SENS2_0;
//	int pressureState=0;
  int ut0,up0;
void begin0(void)
{
    reset0();
    setOversampling0(MS5611_ULTRA_HIGH_RES); // resolution
    delay_ms(100);
    readPROM0(); // read calib parameter
}

// Set oversampling value
void setOversampling0(int osr)
{
    switch (osr)
    {
	case MS5611_ULTRA_LOW_POWER:
	    ct0 = 1;
	    break;
	case MS5611_LOW_POWER:
	    ct0 = 2;
	    break;
	case MS5611_STANDARD:
	    ct0 = 3;
	    break;
	case MS5611_HIGH_RES:
	    ct0 = 5;
	    break;
	case MS5611_ULTRA_HIGH_RES:
	    ct0 = 10;
	    break;
    }

    uosr0 = osr;
}

// Get oversampling value
//ms5611_osr_t getOversampling(void)
//{
//    return (ms5611_osr_t)uosr;
//}

void reset0(void)
{
    ms5611WriteByte(MS5611_CMD_RESET);
}

void readPROM0(void)
{  uint8_t offset;
    for (offset = 0; offset < 6; offset++)
    {
	fc0[offset] = readRegister16_0(MS5611_CMD_READ_PROM + (offset * 2));
    }
}


uint32_t readRawTemperature0(void)
{
  return readRegister24_0(MS5611_CMD_ADC_READ);
}

uint32_t readRawPressure0(void)
{
    return readRegister24_0(MS5611_CMD_ADC_READ);
}

int32_t readPressure0(int compensation)
{   
		uint32_t P;
    uint32_t D1 = up0;

    uint32_t D2 = ut0;
    int32_t dT = D2 - (uint32_t)fc0[4] * 256;

    int64_t OFF = (int64_t)fc0[1] * 65536 + (int64_t)fc0[3] * dT / 128;
    int64_t SENS = (int64_t)fc0[0] * 32768 + (int64_t)fc0[2] * dT / 256;

    if (compensation==1)
    {
	int32_t TEMP = 2000 + ((int64_t) dT * fc0[5]) / 8388608;

	OFF2_0 = 0;
	SENS2_0 = 0;

	if (TEMP < 2000)
	{
	    OFF2_0 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
	    SENS2_0 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
	}

	if (TEMP < -1500)
	{
	    OFF2_0 = OFF2_0 + 7 * ((TEMP + 1500) * (TEMP + 1500));
	    SENS2_0 = SENS2_0 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
	}

	OFF = OFF - OFF2_0;
	SENS = SENS - SENS2_0;
    }

     P = (D1 * SENS / 2097152 - OFF) / 32768;

    return P;
}

uint16_t readRegister16_0(uint8_t reg)
{
    uint16_t value;
		u8 data[2];
	IICreadBytes(MS5611_ADDRESS,reg,2,data);
     unsigned short msb=0;
  unsigned short lsb=0;
	msb=data[0];
	lsb=data[1];
  value= (msb << 8) | lsb;

    return value;
}

// Read 24-bit from register (oops XSB, MSB, LSB)
uint32_t readRegister24_0(uint8_t reg)
{
	   unsigned long msb=0;
  unsigned long lsb=0;
  unsigned long xsb=0;
	u8 data[3];
	uint32_t value;
	IICreadBytes(MS5611_ADDRESS,reg,3,data);
	msb=data[0];
	lsb=data[1];
	xsb=data[2];
  value = (msb << 16) | (lsb << 8) | xsb;

    return value;
}
void ms5611WriteByte0(unsigned char data)
{
	IICwriteByte(MS5611_ADDRESS, 0, data);
}