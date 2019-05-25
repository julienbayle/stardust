#ifndef SD_SENSOR_MPU6050_H
#define SD_SENSOR_MPU6050_H

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>

#include "sd_sensor/i2c.h"

// MPU 6050 configuration words
#define MPU_ACCEL_XOUT1 0x3b
#define MPU_ACCEL_XOUT2 0x3c
#define MPU_ACCEL_YOUT1 0x3d
#define MPU_ACCEL_YOUT2 0x3e
#define MPU_ACCEL_ZOUT1 0x3f
#define MPU_ACCEL_ZOUT2 0x40

#define MPU_GYRO_XOUT1 0x43
#define MPU_GYRO_XOUT2 0x44
#define MPU_GYRO_YOUT1 0x45
#define MPU_GYRO_YOUT2 0x46
#define MPU_GYRO_ZOUT1 0x47
#define MPU_GYRO_ZOUT2 0x48

#define MPU_TEMP1 0x41
#define MPU_TEMP2 0x42

#define MPU_POWER1 0x6b
#define MPU_POWER2 0x6c

#define ACCEL_CONFIG 0x1c

// Accelerometer Sensitivity Ranges
#define ACCEL_SCALE_RANGE_2G 0x00
#define ACCEL_SCALE_RANGE_4G 0x08
#define ACCEL_SCALE_RANGE_8G 0x10
#define ACCEL_SCALE_RANGE_16G 0x18

// Accelerometer Sensitivity Scaling Factors (LSB/g)
#define ACCEL_SCALE_FACTOR_2G   16384.0 * 9.807
#define ACCEL_SCALE_FACTOR_4G   8192.0  * 9.807
#define ACCEL_SCALE_FACTOR_8G   4096.0  * 9.807
#define ACCEL_SCALE_FACTOR_16G  2048.0  * 9.807

#define GYRO_CONFIG 0x1b

// Gyroscope Sensitivity Ranges
#define GYRO_SCALE_RANGE_250DEG 0x00
#define GYRO_SCALE_RANGE_500DEG 0x08
#define GYRO_SCALE_RANGE_1000DEG 0x10
#define GYRO_SCALE_RANGE_2000DEG 0x18

// Gyroscope Sensitivity Scaling Factors (LSB/deg/second)
#define GYRO_SCALE_FACTOR_250DEG 131.0 * 180.0 / M_PI
#define GYRO_SCALE_FACTOR_500DEG 65.5  * 180.0 / M_PI
#define GYRO_SCALE_FACTOR_1000DEG 32.8 * 180.0 / M_PI
#define GYRO_SCALE_FACTOR_2000DEG 16.4 * 180.0 / M_PI

#define GFORCE 9.80665

// FIFO Enable Register
#define FIFO_EN 0x23

// FIFO Count Register
#define FIFO_COUNT 0x72
#define FIFO_RW    0x74
#define SAMPLE_RATE 0x19


float set_accel_scale_factor(int fd, unsigned char acc_scale)
{
    float scale_factor;
    unsigned char  scaleRange;
    switch(acc_scale)
    {
     case 2:
              scaleRange= ACCEL_SCALE_RANGE_2G;
              scale_factor = ACCEL_SCALE_FACTOR_2G;
              break;
     case 4:
              scaleRange= ACCEL_SCALE_RANGE_4G;
              scale_factor = ACCEL_SCALE_FACTOR_4G;
              break;
     case 8:
              scaleRange= ACCEL_SCALE_RANGE_8G;
              scale_factor = ACCEL_SCALE_FACTOR_8G;

              break;
     default:
              scaleRange= ACCEL_SCALE_RANGE_16G;
              scale_factor = ACCEL_SCALE_FACTOR_16G;

              break;
   }

   i2c_smbus_write_byte_data(fd,ACCEL_CONFIG,scaleRange);
   return scale_factor;
}

float set_gyro_scale_factor(int fd, int gyroScale)
{

        int16_t gyro_config = read_byte(fd, GYRO_CONFIG) & 0x18;

        float scale_factor;
        int  scaleRange;

        switch(gyroScale)
          {
            case 250:
            case GYRO_SCALE_RANGE_250DEG:  scaleRange = GYRO_SCALE_RANGE_250DEG;
                                           scale_factor = GYRO_SCALE_FACTOR_250DEG;
                                           break;
            case 500:
            case GYRO_SCALE_RANGE_500DEG:  scaleRange = GYRO_SCALE_RANGE_500DEG;
                                           scale_factor = GYRO_SCALE_FACTOR_500DEG;
                                           break;
            case 1000:
            case GYRO_SCALE_RANGE_1000DEG: scaleRange = GYRO_SCALE_RANGE_1000DEG;
                                           scale_factor = GYRO_SCALE_FACTOR_1000DEG;
                                           break;
            default:                       scaleRange = GYRO_SCALE_RANGE_2000DEG;
                                           scale_factor = GYRO_SCALE_FACTOR_2000DEG;
           }

         i2c_smbus_write_byte_data(fd,GYRO_CONFIG,scaleRange);
         return scale_factor;
}

float read_temperature_register(int fd)
{
        int16_t raw_temp = read_word(fd, MPU_TEMP1);
        return (float)(raw_temp / 340.0f + 36.53);
}

void read_accelerometer_registers(int fd, float data[3], float scale_factor)
{
        int16_t x = read_word(fd, MPU_ACCEL_XOUT1);
        int16_t y = read_word(fd, MPU_ACCEL_YOUT1);
        int16_t z = read_word(fd, MPU_ACCEL_ZOUT1);

        data[0] = (float)(x / scale_factor * GFORCE);
        data[1] = (float)(y / scale_factor * GFORCE);
        data[2] = (float)(z / scale_factor * GFORCE);
}

void enable_fifo(int fd)
{
        // User config register
        int8_t user_control = read_byte(fd, 0x6a);
        // Enables FIFO operations
        write_bit(fd, 0x6a, 6, 1);
        // Enables temp, gyroscope and accelerometer registers 14 bytes data
        i2c_smbus_write_byte_data(fd, FIFO_EN, 0xf8);
        // Enables overflow interrupts
        i2c_smbus_write_byte_data(fd, 0x38, 0x10);
}

int overflow_interrupt(int fd)
{
        int8_t interrupt_state = read_byte(fd, 0x3a);
        if (interrupt_state == 0x10)
                return 1;
        else
                return 0;
}

void reset_fifo(int fd)
{
        write_bit(fd, 0x6a, 2, 1);
}

void wake_up_device(int fd)
{
        int8_t power = i2c_smbus_read_byte_data(fd, MPU_POWER1);

        // force 8Mhz oscillatore
        i2c_smbus_write_byte_data(fd, MPU_POWER1,2); // use gyro 1 pll
        i2c_smbus_write_byte_data(fd, MPU_POWER2,0);
}

short get_fifo_count(int fd)
{
   return read_word(fd,FIFO_COUNT);
}

int  read_fifo(int fd, unsigned char * Fifo,int Count)
{

  return i2c_smbus_read_i2c_block_data(fd,FIFO_RW ,Count, Fifo);
}

void set_sample_rate(int fd,int value)
{
   unsigned char _irate = (8000 / value) - 1;
   i2c_smbus_write_byte_data(fd, SAMPLE_RATE,_irate);
}

void read_fifo_buffer(int fd, float temps, float accel_data[3], float accel_scale_factor, float gyro_data[3], float gyro_scale_factor)
{
    int16_t FifoBuffer[7];
    int i,j;

    // ok let's read 14 bytes in one command
    // because is way faster that way
    // we only need to send the command once
    read_fifo(fd,(unsigned char *)  FifoBuffer, 14);

    // swap byte from big endian to small endian
    for(j=0;j<7;j++)
    FifoBuffer[j]=__bswap_16(FifoBuffer[j]);

    temps = (float)(FifoBuffer[3] / 340.0f + 36.53);

    // Gyro X (2 bytes)
    gyro_data[0] = (float)(FifoBuffer[4] / gyro_scale_factor);
    // Gyro y (2 bytes)
    gyro_data[1] = (float)(FifoBuffer[5] / gyro_scale_factor);
    // Gyro z (2 bytes)
    gyro_data[2] = (float)(FifoBuffer[6] / gyro_scale_factor);

    // Accel x (2 bytes)
    accel_data[0] = (float)(FifoBuffer[0] / accel_scale_factor * GFORCE);
    // Accel y (2 bytes)
    accel_data[1] = (float)(FifoBuffer[1] / accel_scale_factor * GFORCE);
    // Accel z (2 bytes)
    accel_data[2] = (float)(FifoBuffer[2] / accel_scale_factor * GFORCE);
   
}

#endif // SD_SENSOR_MPU6050_H