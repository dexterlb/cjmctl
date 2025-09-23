#include "rotation_accumulator.h"
#include "math_utils.h"

void reset_sector(rotation_accumulator_t *acc, float pos) {
    if(is_in(pos, 0, 0.25)) {
        acc->accum_sector = 0;
    } else if (is_in(pos, 0.25, 0.5)) {
        acc->accum_sector = 1;
    } else if (is_in(pos, 0.5, 0.75)) {
        acc->accum_sector = 2;
    } else if (is_in(pos, 0.75, 1.0)) {
        acc->accum_sector = 3;
    }
}

int raw_sector(const rotation_accumulator_t *acc) {
    return positive_mod((int)acc->accum_sector, 4);
}

void calc_sector_transition_zones(const rotation_accumulator_t *acc,
                                         float *zone_left_start, float *zone_left_end,
                                         float *zone_right_start, float *zone_right_end)
{
    switch (raw_sector(acc)) {
        case 0:
            *zone_left_start = 0.5f;  *zone_left_end = 0.75f;
            *zone_right_start = 0.25f;*zone_right_end = 0.5f;
            break;
        case 1:
            *zone_left_start = 0.75f; *zone_left_end = 1.0f;
            *zone_right_start = 0.5f; *zone_right_end = 0.75f;
            break;
        case 2:
            *zone_left_start = 0.0f;  *zone_left_end = 0.25f;
            *zone_right_start = 0.75f;*zone_right_end = 1.0f;
            break;
        case 3:
            *zone_left_start = 0.25f; *zone_left_end = 0.5f;
            *zone_right_start = 0.0f; *zone_right_end = 0.25f;
            break;
        default:
            *zone_left_start = *zone_left_end = *zone_right_start = *zone_right_end = 0.0f;
            break;
    }
}

void update_cum_sector(rotation_accumulator_t *acc) {
    float zl_start, zl_end, zr_start, zr_end;
    calc_sector_transition_zones(acc, &zl_start, &zl_end, &zr_start, &zr_end);

    if (is_in(acc->raw_pos, zr_start, zr_end)) {
        acc->accum_sector++;
    }
    else if (is_in(acc->raw_pos, zl_start, zl_end)) {
        acc->accum_sector--;
    }
}

void update_cum_pos(rotation_accumulator_t *acc) {
    int jumpovers = acc->accum_sector / 4;
    if (raw_sector(acc) == 0 && acc->raw_pos > 0.75f) {
        jumpovers -= 1;
    }
    acc->accum_pos = (float)jumpovers + acc->raw_pos;
}

void rotation_accumulator_init(rotation_accumulator_t *acc, int8_t sector) {
    if (!acc) return;
    acc->accum_sector = sector;
    acc->accum_pos    = 0;
    acc->raw_pos    = 0;
}

void rotation_accumulator_update_raw_pos(rotation_accumulator_t *acc, float raw_pos) {
    if (!acc) return;
    acc->raw_pos = raw_pos;
    update_cum_sector(acc);
    update_cum_pos(acc);
}
