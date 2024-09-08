#include <errno.h>
#include <gpiod.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

struct gpiod_line_request *request_input_line_rising(const char *chip_path, unsigned int offset, const char *consumer);
const char *edge_event_type_str(struct gpiod_edge_event *event);
