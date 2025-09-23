#pragma once
#include <stdint.h>

typedef struct {
    float   accum_pos;
    float   raw_pos;
    int8_t accum_sector;
} rotation_accumulator_t;

void rotation_accumulator_init(rotation_accumulator_t *acc, int8_t sector);
void rotation_accumulator_update_raw_pos(rotation_accumulator_t *acc, float raw_pos);
void reset_sector(rotation_accumulator_t *acc, float pos);
