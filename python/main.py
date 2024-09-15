import signal
from math import tanh

import gpiod
import rerun as rr
from gpiod.line import Edge
from icm42688p import ICM42688P


def terminate(s, f):
    print("Exiting...")
    exit()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, terminate)

    imu = ICM42688P()
    imu.enable_data_ready_int()

    rr.init("imu")
    rr.connect()

    print("Starting...")

    gyro_roll = 0
    gyro_pitch = 0
    prev_time = None

    with gpiod.request_lines(
        "/dev/gpiochip0", config={4: gpiod.LineSettings(edge_detection=Edge.RISING)}
    ) as request:
        while True:
            for event in request.read_edge_events():
                imu.measure()

                rr.log("imu/accel/x", rr.Scalar(imu.accel[0]))
                rr.log("imu/accel/y", rr.Scalar(imu.accel[1]))
                rr.log("imu/accel/z", rr.Scalar(imu.accel[2]))

                rr.log("imu/gyro/x", rr.Scalar(imu.gyro[0]))
                rr.log("imu/gyro/y", rr.Scalar(imu.gyro[1]))
                rr.log("imu/gyro/z", rr.Scalar(imu.gyro[2]))

                roll = tanh(imu.accel[1] / imu.accel[2])
                pitch = tanh(-imu.accel[0] / 9.81)

                rr.log("accel_estimate/roll", rr.Scalar(roll))
                rr.log("accel_estimate/pitch", rr.Scalar(pitch))

                if prev_time is None:
                    prev_time = event.timestamp_ns
                    continue

                dt = (event.timestamp_ns - prev_time) / 1e9
                prev_time = event.timestamp_ns

                gyro_roll += imu.gyro[0] * dt
                gyro_pitch += imu.gyro[1] * dt

                rr.log("gyro_estimate/roll", rr.Scalar(gyro_roll))
                rr.log("gyro_estimate/pitch", rr.Scalar(gyro_pitch))
