
#include "mpu9250.h"
# define F_CPU 20000000
#include <util/delay.h>
#include <math.h>
#include <stdbool.h>


#define RTC_ADDR 0x68
#define CH_BIT 7 // clock halt bit

struct tm {
	int sec;      // 0 to 59
	int min;      // 0 to 59
	int hour;     // 0 to 23
	int mday;     // 1 to 31
	int mon;      // 1 to 12
	int year;     // year-99
	int wday;     // 1-7

	// 12-hour clock data
	bool am; // true for AM, false for PM
	int twelveHour; // 12 hour clock time
};




/*************************************************************************	
  Issues a start condition and sends address and transfer direction.
  return 0 = device accessible, 1= failed to access device
*************************************************************************/
unsigned char i2c_start(unsigned char address){
	uint8_t twst;
	
	TWCR= (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);	// Start condition
	
	while(!(TWCR & (1<<TWINT)));	// (acabar START) - ESPERA ATÉ TWINT==0. Quando TWINT ==0, INICIA O PROXIMO PASSO- TRANMISSÃO DE DADOS
	
	twst= TW_STATUS & 0XF8;	// TWI STATUS REGISTER 
	if((twst != TW_START) && (twst != TW_REP_START))
		return 1;	// return 1= failed to access device
		
	TWDR=address;
	TWCR=(1<<TWINT)|(1<<TWEN);
	
	while(!(TWCR & (1<<TWINT)));// espera para que TWINT==0 =>TWINT Flag==1. indica que o endereço foi transmitido com sucesso e o ACK/NACK foi recebido.
	
	twst = TW_STATUS & 0xF8;	// Check value of TWI Status	Register. Mask prescaler bits. If status different from MT_SLA_ACK return 1 = failed to access device
	if ((twst != TW_MT_SLA_ACK) && (twst != TW_MR_SLA_ACK) ) 
		return 1;
	return 0;	
}

/*************************************************************************
 Issues a start condition and sends address and transfer direction.
 If device is busy, use ack polling to wait until device is ready
 
 Input:   address and transfer direction of I2C device
*************************************************************************/

void i2c_start_wait(unsigned char address){
	uint8_t   twst;
	while (1){
		TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);		// send START condition
		
		while(!(TWCR & (1<<TWINT)));	// wait until transmission completed
		
		twst = TW_STATUS & 0xF8;		// check value of TWI Status Register. Mask prescaler bits.
		if ( (twst != TW_START) && (twst != TW_REP_START)) 
			continue;
		
		TWDR = address;						// send device address
		TWCR = (1<<TWINT) | (1<<TWEN);
		
		while(!(TWCR & (1<<TWINT)));		// wail until transmission completed
				
		twst = TW_STATUS & 0xF8;			// check value of TWI Status Register. Mask prescaler bits.
		if ( (twst == TW_MT_SLA_NACK )||(twst ==TW_MR_DATA_NACK) ){	// device busy, send stop condition to terminate write operation //
			TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
			while(TWCR & (1<<TWSTO));// wait until stop condition is executed and bus released
			continue;
		}
	break;
	}
}

/*************************************************************************
 Issues a repeated start condition and sends address and transfer direction 

 Input:   address and transfer direction of I2C device
 
 Return:  0 device accessible
          1 failed to access device
*************************************************************************/
unsigned char i2c_rep_start(unsigned char address){
    
	return i2c_start(address);
}

/*************************************************************************
 Terminates the data transfer and releases the I2C bus
*************************************************************************/
void i2c_stop(void){
    
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);		//send stop condition //	
	while(TWCR & (1<<TWSTO));				// wait until stop condition is executed and bus released
}

/*************************************************************************
  Send one byte to I2C device
  
  Input:    byte to be transfered
  Return:   0 write successful 
            1 write failed
*************************************************************************/
unsigned char i2c_write(unsigned char data){	
    uint8_t   twst;
    
	TWDR = data;						// send data to the previously addressed device
	TWCR = (1<<TWINT) | (1<<TWEN);

	while(!(TWCR & (1<<TWINT)));		// wait until transmission completed
	
	twst = TW_STATUS & 0xF8;			// check value of TWI Status Register. Mask prescaler bits
	if(twst != TW_MT_DATA_ACK) 
		return 1;
	return 0;
}

/*************************************************************************
 Read one byte from the I2C device, request more data from device 
 
 Return:  byte read from I2C device
*************************************************************************/
unsigned char i2c_readAck(void){
	
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);	// TWEAN - ENABLE ACKNOWLEDGE
	while(!(TWCR & (1<<TWINT)));				// wait until transmission completed   

    return TWDR;
}

/*************************************************************************
 Read one byte from the I2C device, read is followed by a stop condition 
 
 Return:  byte read from I2C device
*************************************************************************/
unsigned char i2c_readNak(void){
	
	TWCR = (1<<TWINT) | (1<<TWEN);				// TWEAN - ENABLE ACKNOWLEDGE
	while(!(TWCR & (1<<TWINT)));				// wait until transmission completed
	
    return TWDR;
}

void readAccelxyz(float *ax, float *ay, float *az){
	
	int16_t dadosaccel[6];
	i2c_start_wait(MPU9250_ADDR + I2C_WRITE);
	i2c_write(MPU9250_ACCEL_XOUT_H);
	i2c_rep_start(MPU9250_ADDR + I2C_READ);
	dadosaccel[0] = i2c_readAck();
	dadosaccel[1] = i2c_readAck();
	dadosaccel[2] = i2c_readAck();
	dadosaccel[3] = i2c_readAck();
	dadosaccel[4] = i2c_readAck();
	dadosaccel[5] = i2c_readNak();

	i2c_stop();
	
	axc = ((int16_t)dadosaccel[0] << 8) | dadosaccel[1];
	ayc = ((int16_t)dadosaccel[2] << 8) | dadosaccel[3];
	azc = ((int16_t)dadosaccel[4] << 8) | dadosaccel[5];
	
	*ax = axc * _ares*9.807;	//resultado vem em 1G. Multiplicar por 9.81, faz com que passe para m/s^2
	*ay = ayc * _ares*9.807;	//resultado vem em 1G. Multiplicar por 9.81, faz com que passe para m/s^2
	*az = azc * _ares*9.807;	//resultado vem em 1G. Multiplicar por 9.81, faz com que passe para m/s^2
}

void readGyroxyz(float *gx, float *gy, float *gz){
	int16_t dadosgyro[6];
	
	i2c_start_wait(MPU9250_ADDR + I2C_WRITE);
	i2c_write(MPU9250_GYRO_XOUT_H);
	i2c_rep_start(MPU9250_ADDR + I2C_READ);
	dadosgyro[0]=i2c_readAck();
	dadosgyro[1]=i2c_readAck();
	dadosgyro[2]=i2c_readAck();
	dadosgyro[3]=i2c_readAck();
	dadosgyro[4]=i2c_readAck();
	dadosgyro[5]=i2c_readNak();
	
	i2c_stop();
	
	gxc = ((int16_t)dadosgyro[0] << 8) | dadosgyro[1];
	gyc = ((int16_t)dadosgyro[2] << 8) | dadosgyro[3];
	gzc = ((int16_t)dadosgyro[4] << 8) | dadosgyro[5];
	
	*gx = (float)gxc * _gres;	//retorna um valor em em º/s
	*gy = (float)gyc * _gres;	//retorna um valor em em º/s
	*gz = (float)gzc * _gres;	//retorna um valor em em º/s
}

/*************************************************************************
 RTC
*************************************************************************/

uint8_t rtc_read_byte(uint8_t offset)
{
	uint8_t ret;
	i2c_start(RTC_ADDR);
	i2c_write(offset);
	i2c_rep_start(RTC_ADDR + I2C_READ);
	ret = i2c_readAck();
	i2c_stop();
	return ret;
}

void rtc_write_byte(uint8_t b, uint8_t offset)
{
	i2c_start(RTC_ADDR);
	i2c_write(offset);
	i2c_write(b);
	i2c_stop();
}

static bool s_is_ds1307 = false;
static bool s_is_ds3231 = false;
struct tm _tm;

void rtc_init(void)
{
	// Attempt autodetection:
	// 1) Read and save temperature register
	// 2) Write a value to temperature register
	// 3) Read back the value
	//   equal to the one written: DS1307, write back saved value and return
	//   different from written:   DS3231
	
	uint8_t temp1 = rtc_read_byte(0x11);
	uint8_t temp2 = rtc_read_byte(0x12);
	
	rtc_write_byte(0xee, 0x11);
	rtc_write_byte(0xdd, 0x12);

	if (rtc_read_byte(0x11) == 0xee && rtc_read_byte(0x12) == 0xdd) {
		s_is_ds1307 = true;
		// restore values
		rtc_write_byte(temp1, 0x11);
		rtc_write_byte(temp2, 0x12);
	}
	else {
		s_is_ds3231 = true;
	}
}

uint8_t bcd2dec(uint8_t b)
{
	return ((b/16 * 10) + (b % 16));
}

struct tm* rtc_get_time(void)
{
	uint8_t rtc[9];
	uint8_t century = 0;

	// read 7 bytes starting from register 0
	// sec, min, hour, day-of-week, date, month, year
	i2c_start(RTC_ADDR);
	i2c_write(0x0);
	
	i2c_rep_start(RTC_ADDR + I2C_READ);
	rtc_write_byte(RTC_ADDR, 7);

	for (uint8_t i = 0; i < 7; i++) {
		rtc[i] = i2c_readAck();
	}

	i2c_stop();

	// Clear clock halt bit from read data
	// This starts the clock for a DS1307, and has no effect for a DS3231
	rtc[0] &= ~(_BV(CH_BIT)); // clear bit

	_tm.sec = bcd2dec(rtc[0]);
	_tm.min = bcd2dec(rtc[1]);
	_tm.hour = bcd2dec(rtc[2]);
	_tm.mday = bcd2dec(rtc[4]);
	_tm.mon = bcd2dec(rtc[5] & 0x1F); // returns 1-12
	century = (rtc[5] & 0x80) >> 7;
	_tm.year = century == 1 ? 2000 + bcd2dec(rtc[6]) : 1900 + bcd2dec(rtc[6]); // year 0-99
	_tm.wday = bcd2dec(rtc[3]); // returns 1-7

	if (_tm.hour == 0) {
		_tm.twelveHour = 0;
		_tm.am = 1;
		} else if (_tm.hour < 12) {
		_tm.twelveHour = _tm.hour;
		_tm.am = 1;
		} else {
		_tm.twelveHour = _tm.hour - 12;
		_tm.am = 0;
	}

	return &_tm;
}