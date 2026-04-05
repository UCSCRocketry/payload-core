/**
 * @file prebuf.h
 * @brief Pre-launch circular buffer
 */

#ifndef __PREBUF_H__
#define __PREBUF_H__

#include "payload.h"
#include "../lib/spif/spif.h"
#include <stdint.h>

#define PREBUF_DEPTH 64

struct prebuf
{
    struct payload_sample prebuf[PREBUF_DEPTH];
    uint32_t head;
    uint32_t count;
};

void prebuf_push(struct prebuf *pb, const struct payload_sample *s);

uint32_t prebuf_flush(struct prebuf *pb, SPIF_HandleTypeDef *spif);

#endif // __PREBUF_H__
