import time

import numpy as np
from smbus2 import SMBus

ACCEL_FS_16g = 0x00
ACCEL_FS_8g = 0x01
ACCEL_FS_4g = 0x02
ACCEL_FS_2g = 0x03

GYRO_FS_2000dps = 0x00
GYRO_FS_1000dps = 0x01
GYRO_FS_500dps = 0x02
GYRO_FS_250dps = 0x03
GYRO_FS_125dps = 0x04
GYRO_FS_62_5dps = 0x05
GYRO_FS_31_25dps = 0x06
GYRO_FS_15_625dps = 0x07

ODR_32k = 0x01
ODR_16k = 0x02
ODR_8k = 0x03
ODR_4k = 0x04
ODR_2k = 0x05
ODR_1k = 0x06
ODR_200 = 0x07
ODR_100 = 0x08
ODR_50 = 0x09
ODR_25 = 0x0A
ODR_12_5 = 0x0B
ODR_6_25 = 0x0C
ODR_3_125 = 0x0D
ODR_1_5625 = 0x0E
ODR_500 = 0x0F

WHO_AM_I = 0x47

REG_DEVICE_CONFIG = 0x11
REG_INT_CONFIG = 0x14
REG_TEMP_DATA1 = 0x1D
REG_PWR_MGMT0 = 0x4E
REG_GYRO_CONFIG0 = 0x4F
REG_GYRO_CONFIG1 = 0x51
REG_ACCEL_CONFIG0 = 0x50
REG_ACCEL_CONFIG1 = 0x53
REG_GYRO_ACCEL_CONFIG0 = 0x52
REG_INT_CONFIG1 = 0x64
REG_INT_SOURCE0 = 0x65
REG_WHO_AM_I = 0x75
REG_BANK_SEL = 0x76


class ICM42688P:
    def __init__(
        self,
        bus: int = 1,
        addr: int = 0x68,
        accel_fs: int = ACCEL_FS_16g,
        gyro_fs: int = GYRO_FS_2000dps,
        accel_odr: int = ODR_50,
        gyro_odr: int = ODR_50,
    ) -> None:
        self.bus = SMBus(bus)
        self.addr = addr
        self.read_imu_config()

        self.reset()
        self.verify_id()
        self.low_noise_mode()
        self.set_accel_full_scale(accel_fs)
        self.set_gyro_full_scale(gyro_fs)
        self.set_accel_odr(accel_odr)
        self.set_gyro_odr(gyro_odr)

        self.frametype = np.dtype(np.int16).newbyteorder("big")

    def read_imu_config(self):
        self.bank = self.bus.read_byte_data(self.addr, REG_BANK_SEL)
        self.accel_config0 = self.bus.read_byte_data(self.addr, REG_ACCEL_CONFIG0)
        self.accel_config1 = self.bus.read_byte_data(self.addr, REG_ACCEL_CONFIG1)
        self.gyro_config0 = self.bus.read_byte_data(self.addr, REG_GYRO_CONFIG0)
        self.gyro_config1 = self.bus.read_byte_data(self.addr, REG_GYRO_CONFIG1)
        self.gyro_accel_config0 = self.bus.read_byte_data(
            self.addr, REG_GYRO_ACCEL_CONFIG0
        )

    def write(self, reg: int, byte: int) -> None:
        self.bus.write_byte_data(self.addr, reg, byte)
        time.sleep(0.01)

        # TODO: check if write was successful

    def reset(self) -> None:
        """Reset the device. Set bank to 0 and perform a soft reset."""

        self.set_bank(0)
        self.write(REG_DEVICE_CONFIG, 0x01)

    def set_bank(self, bank: int) -> None:
        """Set the register bank."""

        self.write(REG_BANK_SEL, bank)
        self.bank = bank

    def verify_id(self) -> None:
        """Verify the device ID."""

        byte = self.bus.read_byte_data(self.addr, REG_WHO_AM_I)
        if byte != WHO_AM_I:
            raise Exception(f"Device ID ({self.addr}) doesn't match with the ICM42688P")

    def low_noise_mode(self) -> None:
        """Turn on gyro and accel in low noise mode."""
        self.write(REG_PWR_MGMT0, 0x0F)

    def set_accel_full_scale(self, scale: int) -> None:
        """Set the accelerometer full scale range."""

        accel_config = (scale << 5) | (self.accel_config0 & 0x1F)
        self.write(REG_ACCEL_CONFIG0, accel_config)
        self.accel_config0 = accel_config
        self.accel_res = (1 << (4 - scale)) / 32_768

    def set_gyro_full_scale(self, scale: int) -> None:
        """Set the gyroscope full scale range."""

        gyro_config = (scale << 5) | (self.gyro_config0 & 0x1F)
        self.write(REG_GYRO_CONFIG0, gyro_config)
        self.gyro_config0 = gyro_config
        self.gyro_res = (2_000 / (1 << scale)) / 32_768

    def measure(self, degrees: bool = False, gravity_unit: bool = False) -> None:
        """Read the last and currrent data from the IMU."""

        # Message starts at REG_TEMP_DATA1 and is 14 bytes long:
        # temp1 + temp0 + accel_x1 + accel_x0 + ... + gyro_z1 + gyro_z0
        measure = self.bus.read_i2c_block_data(self.addr, REG_TEMP_DATA1, 14)
        measure = np.frombuffer(bytes(measure), dtype=self.frametype).astype(np.float64)

        self.temp = (measure[0] / 132.48) + 25
        self.accel = measure[1:4] * self.accel_res
        self.gyro = measure[4:7] * self.gyro_res

        if not degrees:
            self.gyro *= np.pi / 180
        if not gravity_unit:
            self.accel *= 9.80665

    def enable_data_ready_int(self) -> None:
        # 0x1b = 00011011 (active high, push pull, pulsed for int1 and int2)
        self.write(REG_INT_CONFIG, 0x1B)

        # set INT_ASYNC_RESET to 0 for proper int1 and int2 operation
        byte = self.bus.read_byte_data(self.addr, REG_INT_CONFIG1)
        self.write(REG_INT_CONFIG1, byte & ~0x10)

        # reset done and UI data ready routed to int1
        self.write(REG_INT_SOURCE0, 0x18)

    def set_accel_odr(self, odr: int) -> None:
        """Set the accelerometer output data rate."""

        accel_config = odr | (self.accel_config0 & 0xF0)
        self.write(REG_ACCEL_CONFIG0, accel_config)
        self.accel_config0 = accel_config

    def set_gyro_odr(self, odr: int) -> None:
        """Set the gyroscope output data rate."""

        if ODR_6_25 <= odr <= ODR_1_5625:
            raise Exception("Gyroscope ODR must be >= 12.5 Hz")

        gyro_config = odr | (self.gyro_config0 & 0xF0)
        self.write(REG_GYRO_CONFIG0, gyro_config)
        self.gyro_config0 = gyro_config

    def __repr__(self) -> str:
        accel_order = ((self.accel_config1 & 0x18) >> 3) + 1
        gyro_order = ((self.gyro_config1 & 0x30) >> 4) + 1

        return (
            "ICM42688P(\n"
            "    -- IMU --\n"
            f"    addr={self.addr},\n"
            f"    bank={self.bank},\n"
            "    -- Accelerometer --\n"
            f"    accel_config0={self.accel_config0:08b},\n"
            f"    accel_config1={self.accel_config1:08b}, (filter order: {accel_order})\n"
            f"    accel_scale={self.accel_res},\n"
            "    -- Gyroscope --\n"
            f"    gyro_config0={self.gyro_config0:08b},\n"
            f"    gyro_config1={self.gyro_config1:08b}, (filter order: {gyro_order})\n"
            f"    gyro_scale={self.gyro_res},\n"
            "    -- Gyroscope and Accelerometer --\n"
            f"    gyro_accel_config0={self.gyro_accel_config0:08b},\n"
            ")"
        )
