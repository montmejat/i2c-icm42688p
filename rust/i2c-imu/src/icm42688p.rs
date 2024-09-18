use i2cdev::core::*;
use i2cdev::linux::LinuxI2CDevice;

use std::thread;
use std::time::Duration;

const WHO_AM_I: u8 = 0x47;

const REG_DEVICE_CONFIG: u8 = 0x11;
const REG_INT_CONFIG: u8 = 0x14;
const REG_TEMP_DATA1: u8 = 0x1D;
const REG_PWR_MGMT0: u8 = 0x4E;
const REG_GYRO_CONFIG0: u8 = 0x4F;
const REG_ACCEL_CONFIG0: u8 = 0x50;
const REG_INT_SOURCE0: u8 = 0x65;
const REG_WHO_AM_I: u8 = 0x75;
const REG_BANK_SEL: u8 = 0x76;

#[allow(dead_code)]
pub enum AccelScale {
    Fs2g,
    Fs4g,
    Fs8g,
    Fs16g,
}

#[allow(dead_code)]
pub enum GyroScale {
    Fs125dps,
    Fs250dps,
    Fs500dps,
    Fs1000dps,
    Fs2000dps,
    Fs62_4dps,
    Fs31_2dps,
    Fs15_6dps,
}

#[allow(dead_code)]
pub enum SamplingRate {
    Odr32k,
    Odr16k,
    Odr8k,
    Odr4k,
    Odr2k,
    Odr1k,
    Odr200,
    Odr100,
    Odr50,
    Odr25,
    Odr12_5,
    Odr6_25,
    Odr3_125,
    Odr1_5625,
    Odr500,
}

pub struct Icm42688p {
    device: LinuxI2CDevice,
    accel_config_0: u8,
    accel_res: f32,
    gyro_config_0: u8,
    gyro_res: f32,
    pub temperature: f32,
    pub gyro: [f32; 3],
    pub accel: [f32; 3],
}

impl Icm42688p {
    pub fn new(bus: &str, address: u16) -> Self {
        let mut device = LinuxI2CDevice::new(bus, address).unwrap();
        let accel_config_0 = device.smbus_read_byte_data(REG_ACCEL_CONFIG0).unwrap();
        let gyro_config_0 = device.smbus_read_byte_data(REG_GYRO_CONFIG0).unwrap();

        let mut icm = Icm42688p {
            device,
            accel_config_0,
            accel_res: 1.0, // TODO: read from sensor
            gyro_config_0,
            gyro_res: 1.0,    // TODO: read from sensor
            temperature: 0.0, // TODO: read from sensor
            gyro: [0.0; 3],
            accel: [0.0; 3],
        };

        assert!(icm.device.smbus_read_byte_data(REG_WHO_AM_I).unwrap() == WHO_AM_I);

        icm.device
            .smbus_write_byte_data(REG_DEVICE_CONFIG, 0x01)
            .unwrap();
        thread::sleep(Duration::from_millis(1));

        icm.device
            .smbus_write_byte_data(REG_BANK_SEL, 0x00)
            .unwrap();

        icm.device
            .smbus_write_byte_data(REG_PWR_MGMT0, 0x0F)
            .unwrap();

        icm
    }

    pub fn enable_data_interuption(&mut self) {
        // 0x1b = 00011011 (active high, push pull, pulsed for int1 and int2)
        self.device
            .smbus_write_byte_data(REG_INT_CONFIG, 0x1B)
            .unwrap();

        // set INT_ASYNC_RESET to 0 for proper int1 and int2 operation
        let int_config = self.device.smbus_read_byte_data(REG_INT_CONFIG).unwrap();
        self.device
            .smbus_write_byte_data(REG_INT_CONFIG, int_config & !0x10)
            .unwrap();

        // reset done and UI data ready routed to int1
        self.device
            .smbus_write_byte_data(REG_INT_SOURCE0, 0x18)
            .unwrap();
    }

    pub fn change_accel_scale(&mut self, scale: AccelScale) {
        let accel_scale = scale as u8;

        let accel_config = (accel_scale << 5) | (self.accel_config_0 & 0x1F);
        self.device
            .smbus_write_byte_data(REG_ACCEL_CONFIG0, accel_config)
            .unwrap();
        self.accel_config_0 = accel_config;
        self.accel_res = (1 << (4 - accel_scale)) as f32 / 32768.0;
    }

    pub fn change_accel_odr(&mut self, odr: SamplingRate) {
        let accel_config = (odr as u8 + 1) | (self.accel_config_0 & 0xF0);
        self.device
            .smbus_write_byte_data(REG_ACCEL_CONFIG0, accel_config)
            .unwrap();
        self.accel_config_0 = accel_config;
    }

    pub fn change_gyro_scale(&mut self, scale: GyroScale) {
        let gyro_scale = scale as u8;

        let gyro_config = (gyro_scale << 5) | (self.gyro_config_0 & 0x1F);
        self.device
            .smbus_write_byte_data(REG_GYRO_CONFIG0, gyro_config)
            .unwrap();
        self.gyro_config_0 = gyro_config;
        self.gyro_res = (2000. / (1 << gyro_scale) as f32) / 32768.0;
    }

    pub fn change_gyro_odr(&mut self, odr: SamplingRate) {
        let sampling_rate = odr as u8;
        if sampling_rate >= SamplingRate::Odr6_25 as u8
            && sampling_rate <= SamplingRate::Odr1_5625 as u8
        {
            return; // invalid odr for gyroscope, must be >= 12.5 hz
        }

        let gyro_config = (sampling_rate + 1) | (self.gyro_config_0 & 0xF0);
        self.device
            .smbus_write_byte_data(REG_GYRO_CONFIG0, gyro_config)
            .unwrap();
        self.gyro_config_0 = gyro_config;
    }

    pub fn read_data(&mut self) {
        let data = self
            .device
            .smbus_read_i2c_block_data(REG_TEMP_DATA1, 14)
            .unwrap();

        self.temperature = (((data[0] as i16) << 8) | data[1] as i16) as f32 / 132.48 + 25.0;
        self.accel[0] = ((data[2] as i16) << 8 | data[3] as i16) as f32 * self.accel_res;
        self.accel[1] = ((data[4] as i16) << 8 | data[5] as i16) as f32 * self.accel_res;
        self.accel[2] = ((data[6] as i16) << 8 | data[7] as i16) as f32 * self.accel_res;
        self.gyro[0] = ((data[8] as i16) << 8 | data[9] as i16) as f32 * self.gyro_res;
        self.gyro[1] = ((data[10] as i16) << 8 | data[11] as i16) as f32 * self.gyro_res;
        self.gyro[2] = ((data[12] as i16) << 8 | data[13] as i16) as f32 * self.gyro_res;
    }
}
