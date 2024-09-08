import signal

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
