#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "icm42688p.h"

#define WHO_AM_I 0x47

#define REG_DEVICE_CONFIG 0x11
#define REG_WHO_AM_I 0x75

#define REG_INT_CONFIG 0x14
#define REG_TEMP_DATA1 0x1D
#define REG_PWR_MGMT0 0x4E
#define REG_GYRO_CONFIG0 0x4F
#define REG_ACCEL_CONFIG0 0x50
#define REG_GYRO_CONFIG1 0x51
#define REG_ACCEL_CONFIG1 0x53
#define REG_GYRO_ACCEL_CONFIG0 0x52
#define REG_INT_CONFIG1 0x64
#define REG_INT_SOURCE0 0x65
#define REG_BANK_SEL 0x76

int write_data(struct icm42688p icm, uint8_t byte, uint8_t reg)
{
    uint8_t buf[2] = {reg, byte};

    if (write(icm.file, buf, 2) != 2)
        return 1;

    usleep(1000);

    return 0;
}

int read_data(struct icm42688p icm, uint8_t reg)
{
    if (write(icm.file, &reg, 1) != 1)
        return -1;

    uint8_t buf[1];

    if (read(icm.file, buf, 1) != 1)
        return -1;

    return buf[0];
}

int read_bytes(struct icm42688p icm, uint8_t reg, uint8_t *buf, int len)
{
    if (icm.file < 0)
    {
        fprintf(stderr, "Wrong file descriptor: %d.\n", icm.file);
        return 1;
    }

    int ret = write(icm.file, &reg, 1);
    if (ret != 1)
    {
        fprintf(stderr, "Failed to write 0x1 to register 0x%02X of file descriptor %d (%d).\n", reg, icm.file, ret);
        return 1;
    }

    ret = read(icm.file, buf, len);
    if (ret != len)
    {
        fprintf(stderr, "Failed to read %d bytes from file descriptor %d (%d).\n", len, icm.file, ret);
        return 1;
    }

    return 0;
}

int read_config(struct icm42688p *icm)
{
    icm->gyro_config0 = read_data(*icm, REG_GYRO_CONFIG0);
    icm->gyro_config1 = read_data(*icm, REG_GYRO_CONFIG1);
    icm->accel_config0 = read_data(*icm, REG_ACCEL_CONFIG0);
    icm->accel_config1 = read_data(*icm, REG_ACCEL_CONFIG1);
    icm->gyro_accel_config0 = read_data(*icm, REG_GYRO_ACCEL_CONFIG0);

    return 0;
}

int set_accel_scale(struct icm42688p *icm, uint8_t scale)
{
    uint8_t accel_config = (scale << 5) | (icm->accel_config0 & 0x1F);
    write_data(*icm, accel_config, REG_ACCEL_CONFIG0);
    icm->accel_config0 = accel_config;
    icm->accel_res = (float)(1 << (4 - scale)) / 32768;

    return 0;
}

void set_accel_odr(struct icm42688p *icm, uint8_t odr)
{
    uint8_t accel_config = odr | (icm->accel_config0 & 0xF0);
    write_data(*icm, accel_config, REG_ACCEL_CONFIG0);
    icm->accel_config0 = accel_config;
}

int set_gyro_scale(struct icm42688p *icm, uint8_t scale)
{
    uint8_t gyro_config = (scale << 5) | (icm->gyro_config0 & 0x1F);
    write_data(*icm, gyro_config, REG_GYRO_CONFIG0);
    icm->gyro_config0 = gyro_config;
    icm->gyro_res = (2000 / (float)(1 << scale)) / 32768;

    return 0;
}

void set_gyro_odr(struct icm42688p *icm, uint8_t odr)
{
    if (odr >= ODR_6_25 && odr <= ODR_1_5625)
        return; // Invalid ODR for gyroscope, must be >= 12.5 Hz

    uint8_t gyro_config = odr | (icm->gyro_config0 & 0xF0);
    write_data(*icm, gyro_config, REG_GYRO_CONFIG0);
    icm->gyro_config0 = gyro_config;
}

void enable_data_ready_int(struct icm42688p *icm)
{
    // 0x1b = 00011011 (active high, push pull, pulsed for int1 and int2)
    write_data(*icm, REG_INT_CONFIG, 0x1B);

    // set INT_ASYNC_RESET to 0 for proper int1 and int2 operation
    uint8_t byte = read_data(*icm, REG_INT_CONFIG1);
    write_data(*icm, byte & ~0x10, REG_INT_CONFIG1);

    // reset done and UI data ready routed to int1
    write_data(*icm, 0x18, REG_INT_SOURCE0);
}

int init(const char *i2c_bus, int i2c_addr, struct icm42688p *icm)
{
    if ((icm->file = open(i2c_bus, O_RDWR)) < 0)
        return 1;

    if (ioctl(icm->file, I2C_SLAVE, i2c_addr) < 0)
        return 1;

    // Reset the device to default values
    if (write_data(*icm, 0x01, REG_DEVICE_CONFIG))
        return 1;
    usleep(1000);

    // Select the user bank 0
    if (write_data(*icm, 0x00, REG_BANK_SEL))
        return 1;

    // Check the WHO_AM_I register
    if (read_data(*icm, REG_WHO_AM_I) != WHO_AM_I)
        return 1;

    // Set IMU to low noise mode
    if (write_data(*icm, 0x0F, REG_PWR_MGMT0))
        return 1;

    read_config(icm);

    return 0;
}

int measure(struct icm42688p icm, struct imu_data *imu_data)
{
    // Message starts at REG_TEMP_DATA1 and is 14 bytes long:
    // temp1 + temp0 + accel_x1 + accel_x0 + ... + gyro_z1 + gyro_z0

    unsigned char data[14] = {0};
    if (read_bytes(icm, REG_TEMP_DATA1, data, 14))
        return 1;

    imu_data->temperature = (int16_t)((data[0] << 8) | data[1]) / 132.48 + 25;
    imu_data->accel[0] = (int16_t)(data[2] << 8 | data[3]) * icm.accel_res;
    imu_data->accel[1] = (int16_t)(data[4] << 8 | data[5]) * icm.accel_res;
    imu_data->accel[2] = (int16_t)(data[6] << 8 | data[7]) * icm.accel_res;
    imu_data->gyro[0] = (int16_t)(data[8] << 8 | data[9]) * icm.gyro_res;
    imu_data->gyro[1] = (int16_t)(data[10] << 8 | data[11]) * icm.gyro_res;
    imu_data->gyro[2] = (int16_t)(data[12] << 8 | data[13]) * icm.gyro_res;

    return 0;
}

void print_config(struct icm42688p icm)
{
    printf("Gyro Config 0: 0x%02X\n", icm.gyro_config0);
    printf("Gyro Config 1: 0x%02X\n", icm.gyro_config1);
    printf("Accel Config 0: 0x%02X\n", icm.accel_config0);
    printf("Accel Config 1: 0x%02X\n", icm.accel_config1);
    printf("Gyro/Accel Config 0: 0x%02X\n", icm.gyro_accel_config0);
    printf("Accel Resolution: %f\n", icm.accel_res);
    printf("Gyro Resolution: %f\n", icm.gyro_res);
}