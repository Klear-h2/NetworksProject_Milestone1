#pragma once
#include <stdint.h>
#include <stddef.h>
#include "cJSON.h"

#define MCT_MAX_ATR 1600

typedef struct {
    uint8_t data[MCT_MAX_ATR];
    size_t len;
    int started;
    int escape;
} slip_parser_t;

// Returns an owned cJSON object on a complete, verified SLIP+ATR message.
cJSON *mct_feed(slip_parser_t *p, uint8_t byte);
void mct_send(cJSON *object);
