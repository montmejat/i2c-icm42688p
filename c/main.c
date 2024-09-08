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
    set_accel_odr(icm, ODR_100);
    set_gyro_scale(icm, GYRO_FS_2000dps);
    set_accel_odr(icm, ODR_100);
    enable_data_ready_int(icm);

    print_config(*icm);

    static const char *const chip_path = "/dev/gpiochip0";
    static const unsigned int line_offset = 4;

    struct gpiod_edge_event_buffer *event_buffer;
    struct gpiod_line_request *request;
    struct gpiod_edge_event *event;
    int i, ret, event_buf_size;

    request = request_input_line(chip_path, line_offset,
                                 "watch-line-value");
    if (!request)
    {
        fprintf(stderr, "failed to request line: %s\n",
                strerror(errno));
        return EXIT_FAILURE;
    }

    event_buf_size = 1;
    event_buffer = gpiod_edge_event_buffer_new(event_buf_size);
    if (!event_buffer)
    {
        fprintf(stderr, "failed to create event buffer: %s\n",
                strerror(errno));
        return EXIT_FAILURE;
    }

    for (;;)
    {
        /* Blocks until at least one event is available. */
        ret = gpiod_line_request_read_edge_events(request, event_buffer,
                                                  event_buf_size);
        if (ret == -1)
        {
            fprintf(stderr, "error reading edge events: %s\n",
                    strerror(errno));
            return EXIT_FAILURE;
        }
        for (i = 0; i < ret; i++)
        {
            event = gpiod_edge_event_buffer_get_event(event_buffer,
                                                      i);
            printf("offset: %d  type: %-7s  event #%ld\n",
                   gpiod_edge_event_get_line_offset(event),
                   edge_event_type_str(event),
                   gpiod_edge_event_get_line_seqno(event));
        }
    }

    return 0;
}
