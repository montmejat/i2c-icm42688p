#include <errno.h>
#include <gpiod.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gpio/gpio.h"
#include "icm42688p/icm42688p.h"

int main()
{
    struct icm42688p *icm;
    static const char *const i2c_bus = "/dev/i2c-1";
    static const int i2c_addr = 0x68;

    init(i2c_bus, i2c_addr, icm);
    set_accel_scale(icm, ACCEL_FS_4g);
    set_accel_odr(icm, ODR_12_5);
    set_gyro_scale(icm, GYRO_FS_2000dps);
    set_gyro_odr(icm, ODR_12_5);
    enable_data_ready_int(icm);

    static const char *const chip_path = "/dev/gpiochip0";
    static const unsigned int line_offset = 4;

    struct gpiod_edge_event_buffer *event_buffer;
    struct gpiod_line_request *request;
    struct gpiod_edge_event *event;
    int i, ret, event_buf_size;

    request = request_input_line_rising(chip_path, line_offset, "watch-line-value");
    if (!request)
    {
        fprintf(stderr, "failed to request line: %s\n", strerror(errno));
        return EXIT_FAILURE;
    }

    event_buf_size = 1;
    event_buffer = gpiod_edge_event_buffer_new(event_buf_size);
    if (!event_buffer)
    {
        fprintf(stderr, "failed to create event buffer: %s\n", strerror(errno));
        return EXIT_FAILURE;
    }

    for (;;)
    {
        ret = gpiod_line_request_read_edge_events(request, event_buffer, event_buf_size);
        if (ret == -1)
        {
            fprintf(stderr, "error reading edge events: %s\n", strerror(errno));
            return EXIT_FAILURE;
        }

        for (i = 0; i < ret; i++)
        {
            event = gpiod_edge_event_buffer_get_event(event_buffer, i);

            struct imu_data imu_data;
            measure(*icm, &imu_data);

            printf("timestamp_ns: %llu, temperature: %f, accel: %f %f %f, gyro: %f, %f, %f\n",
                   gpiod_edge_event_get_timestamp_ns(event), imu_data.temperature,
                   imu_data.accel[0], imu_data.accel[1], imu_data.accel[2],
                   imu_data.gyro[0], imu_data.gyro[1], imu_data.gyro[2]);
        }
    }

    return 0;
}
