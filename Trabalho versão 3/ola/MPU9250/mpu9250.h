#ifndef mpu9250_H_
#define mpu9250_H_

#include <compat/twi.h>
#include <avr/io.h>

#define MPU9250_ADDR	0xD0
#define MPU9250_GFS_250  0x0 ///< +250dps
#define MPU9250_ACCEL_XOUT_H 0x3B
#define MPU9250_ACCEL_XOUT_L 0x3C
#define MPU9250_ACCEL_YOUT_H 0x3D
#define MPU9250_ACCEL_YOUT_L 0x3E
#define MPU9250_ACCEL_ZOUT_H 0x3F
#define MPU9250_ACCEL_ZOUT_L 0x40
#define MPU9250_GFS_250	0x0 ///< +250dps - giroscopio
#define MPU9250_AFS_2G	0x0 ///< 2g - acelerometro
#define AK8963_MODE_C8HZ 0x02 ///< Continous data output 8Hz
#define AK8963_BIT_16 0x1 ///< 16bit output
#define AK8963_CNTL1 0x0A /// AK8963 Register Addresses
#define AK8963_ADDRESS 0x0C ///< AK8963 I2C slave address
#define MPU9250_PWR_MGMT_1 0x6B
#define MPU9250_CONFIG 0x1A
#define MPU9250_SMPLRT_DIV 0x19
#define MPU9250_GYRO_CONFIG 0x1B
#define MPU9250_ACCEL_CONFIG 0x1C
#define MPU9250_ACCEL_CONFIG_2 0x1D
#define MPU9250_INT_PIN_CFG 0x37
#define AK8963_ASAX 0x10
#define MPU9250_INT_STATUS 0x3A

#define MPU9250_GYRO_XOUT_H 0x43
#define MPU9250_GYRO_XOUT_L 0x44
#define MPU9250_GYRO_YOUT_H 0x45
#define MPU9250_GYRO_YOUT_L 0x46
#define MPU9250_GYRO_ZOUT_H 0x47
#define MPU9250_GYRO_ZOUT_L 0x48



int16_t axc, ayc, azc;
float _gres, _ares;
int16_t gxc, gyc, gzc;


#define I2C_READ	1	//LEITURA = 1
#define I2C_WRITE	0	//ESCRITA = 0

 /**********************FUN�OES**********************/
unsigned char i2c_start(unsigned char address);
void i2c_start_wait(unsigned char address);
unsigned char i2c_rep_start(unsigned char address);
void i2c_stop(void);
unsigned char i2c_write(unsigned char data);
unsigned char i2c_readAck(void);
unsigned char i2c_readNak(void);
void readAccelxyz(float *ax, float *ay, float *az);
void readGyroxyz(float *gx, float *gy, float *gz);
void rotincl(void);

/*********************************************************************************
 * 
 *									AUX 2
 *  
*********************************************************************************/
#ifndef CPU_FREQ
#define CPU_FREQ 16000000L
#endif

#ifndef TWI_FREQ
#define TWI_FREQ 100000L
#endif

#ifndef TWI_BUFFER_LENGTH
#define TWI_BUFFER_LENGTH 32
#endif

#define TWI_READY 0
#define TWI_MRX   1
#define TWI_MTX   2
#define TWI_SRX   3
#define TWI_STX   4

void twi_init(void);
void twi_setAddress(uint8_t);
uint8_t twi_readFrom(uint8_t, uint8_t*, uint8_t);
uint8_t twi_writeTo(uint8_t, uint8_t*, uint8_t, uint8_t);
uint8_t twi_transmit(uint8_t*, uint8_t);
void twi_attachSlaveRxEvent( void (*)(uint8_t*, int) );
void twi_attachSlaveTxEvent( void (*)(void) );
void twi_reply(uint8_t);
void twi_stop(void);
void twi_releaseBus(void);

/*********************************************************************************
 * 
 *									AUX
 *  
*********************************************************************************/
#include <inttypes.h>

#define BUFFER_LENGTH 32

void twi_begin_transmission(uint8_t);
void twi_send_byte(uint8_t);
uint8_t twi_end_transmission(void);

uint8_t twi_request_from(uint8_t, uint8_t);
uint8_t twi_receive(void);

void twi_init_master(void);
void twi_init_slave(uint8_t);
void twi_send(uint8_t*, uint8_t);
void twi_send_char(char*);
uint8_t twi_available(void);
void twi_set_on_receive( void (*)(int) );
void twi_set_on_request( void (*)(void) );



/*********************************************************************************
 * 
 *									RTC
 *  
*********************************************************************************/

#include <stdbool.h>
#include <avr/io.h>

#define DS1307_SLAVE_ADDR 0b11010000
/** Time structure
 * 
 * Both 24-hour and 12-hour time is stored, and is always updated when rtc_get_time is called.
 * 
 * When setting time and alarm, 24-hour mode is always used.
 *
 * If you run your clock in 12-hour mode:
 * - set time hour to store in twelveHour and set am to true or false.
 * - call rtc_12h_translate (this will put the correct value in hour, so you don't have to
 *   calculate it yourself.
 * - call rtc_set_alarm or rtc_set_clock
 *
 * Note that rtc_set_clock_s, rtc_set_alarm_s, rtc_get_time_s, rtc_set_alarm_s always operate in 24-hour mode
 * and translation has to be done manually (you can call rtc_24h_to_12h to perform the calculation)
 *
 */
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

// statically allocated 
extern struct tm _tm;

// Initialize the RTC and autodetect type (DS1307 or DS3231)
void rtc_init(void);

// Autodetection
bool rtc_is_ds1307(void);
bool rtc_is_ds3231(void);

void rtc_set_ds1307(void);
void rtc_set_ds3231(void);

// Get/set time
// Gets the time: Supports both 24-hour and 12-hour mode
struct tm* rtc_get_time(void);
// Gets the time: 24-hour mode only
void rtc_get_time_s(uint8_t* hour, uint8_t* min, uint8_t* sec);
// Sets the time: Supports both 24-hour and 12-hour mode
void rtc_set_time(struct tm* tm_);
// Sets the time: Supports 12-hour mode only
void rtc_set_time_s(uint8_t hour, uint8_t min, uint8_t sec);

#endif