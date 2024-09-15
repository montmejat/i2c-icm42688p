use i2cdev::core::*;
use i2cdev::linux::LinuxI2CDevice;

use std::thread;
use std::time::Duration;

const WHO_AM_I: u8 = 0x47;

const REG_DEVICE_CONFIG: u8 = 0x11;
const REG_INT_CONFIG: u8 = 0x14;
const REG_PWR_MGMT0: u8 = 0x4E;
const REG_INT_SOURCE0: u8 = 0x65;
const REG_WHO_AM_I: u8 = 0x75;
const REG_BANK_SEL: u8 = 0x76;

pub enum AccelScale {
    Fs2g = 0x00,
    Fs4g = 0x01,
    Fs8g = 0x02,
    Fs16g = 0x03,
}

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

pub struct Icm42688p {
    device: LinuxI2CDevice,
}

impl Icm42688p {
    pub fn new(bus: &str, address: u16) -> Self {
        let device = LinuxI2CDevice::new(bus, address).unwrap();
        let mut icm = Icm42688p { device };
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
        let accel_config = (scale as u8) << 5;
    }
}
