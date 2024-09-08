#include <stdint.h>

#define ACCEL_FS_16g 0x00
#define ACCEL_FS_8g 0x01
#define ACCEL_FS_4g 0x02
#define ACCEL_FS_2g 0x03

#define GYRO_FS_2000dps 0x00
#define GYRO_FS_1000dps 0x01
#define GYRO_FS_500dps 0x02
#define GYRO_FS_250dps 0x03
#define GYRO_FS_125dps 0x04
#define GYRO_FS_62_5dps 0x05
#define GYRO_FS_31_25dps 0x06
#define GYRO_FS_15_625dps 0x07

#define ODR_32k 0x01
#define ODR_16k 0x02
#define ODR_8k 0x03
#define ODR_4k 0x04
#define ODR_2k 0x05
#define ODR_1k 0x06
#define ODR_200 0x07
#define ODR_100 0x08
#define ODR_50 0x09
#define ODR_25 0x0A
#define ODR_12_5 0x0B
#define ODR_6_25 0x0C
#define ODR_3_125 0x0D
#define ODR_1_5625 0x0E
#define ODR_500 0x0F

struct icm42688p
{
    int file;
    uint8_t gyro_config0;
    uint8_t gyro_config1;
    uint8_t accel_config0;
    uint8_t accel_config1;
    uint8_t gyro_accel_config0;
    float accel_res;
    float gyro_res;
};

int init(const char *i2c_bus, int i2c_addr, struct icm42688p *icm);
int set_accel_scale(struct icm42688p *icm, uint8_t scale);
void set_accel_odr(struct icm42688p *icm, uint8_t odr);
int set_gyro_scale(struct icm42688p *icm, uint8_t scale);
void set_gyro_odr(struct icm42688p *icm, uint8_t odr);
void enable_data_ready_int(struct icm42688p *icm);

void print_config(struct icm42688p icm);
