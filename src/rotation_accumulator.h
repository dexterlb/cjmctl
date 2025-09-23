#pragma once
#include <stdint.h>

typedef struct {
    float   cum_pos;
    float   raw_pos;
    uint8_t cum_sector;
} rotation_accumulator_t;

void rotation_accumulator_init(rotation_accumulator_t *acc, uint8_t sector);
void rotation_accumulator_update_raw_pos(rotation_accumulator_t *acc, float raw_pos);
