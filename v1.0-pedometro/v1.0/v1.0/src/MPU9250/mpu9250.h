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

 /**********************FUNÇOES**********************/
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

#endif
