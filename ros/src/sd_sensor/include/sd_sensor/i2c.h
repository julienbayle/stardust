#ifndef SD_SENSOR_I2C_H
#define SD_SENSOR_I2C_H

#include <byteswap.h>

extern "C" {
        #include <linux/i2c-dev.h>
        #include <i2c/smbus.h>
}

int16_t read_word(int fd, int address)
{
        int16_t value = (unsigned short )i2c_smbus_read_word_data(fd, address);
        return __bswap_16(value);
}

int8_t read_byte(int fd, int address)
{
        return i2c_smbus_read_byte_data(fd, address);
}

void write_bit(int fd, int register_address, int nth_bit, int value)
{
        int8_t reg = read_byte(fd, register_address);
        if ( value == 1 ) {
                reg = reg | (1 << nth_bit);
        }
        else {
                reg = reg & ~(1 << nth_bit);
        }
        i2c_smbus_write_byte_data(fd, register_address, reg);
}

#endif // SD_SENSOR_I2C_H