#include "rotation_accumulator.h"
#include "math_utils.h"

int positive_mod(int x, int m) {
    int r = x % m;
    return (r < 0) ? r + m : r;
}

int raw_sector(const rotation_accumulator_t *acc) {
    return positive_mod((int)acc->cum_sector, 4);
}

void calc_sector_transition_zones(const rotation_accumulator_t *acc,
                                         float *zone_left_a, float *zone_left_b,
                                         float *zone_right_a, float *zone_right_b)
{
    switch (raw_sector(acc)) {
        case 0:
            *zone_left_a = 0.5f;  *zone_left_b = 0.75f;
            *zone_right_a = 0.25f;*zone_right_b = 0.5f;
            break;
        case 1:
            *zone_left_a = 0.75f; *zone_left_b = 1.0f;
            *zone_right_a = 0.5f; *zone_right_b = 0.75f;
            break;
        case 2:
            *zone_left_a = 0.0f;  *zone_left_b = 0.25f;
            *zone_right_a = 0.75f;*zone_right_b = 1.0f;
            break;
        case 3:
            *zone_left_a = 0.25f; *zone_left_b = 0.5f;
            *zone_right_a = 0.0f; *zone_right_b = 0.25f;
            break;
        default:
            *zone_left_a = *zone_left_b = *zone_right_a = *zone_right_b = 0.0f;
            break;
    }
}

void update_cum_sector(rotation_accumulator_t *acc) {
    float zl_a, zl_b, zr_a, zr_b;
    calc_sector_transition_zones(acc, &zl_a, &zl_b, &zr_a, &zr_b);

    if (is_in(acc->raw_pos, zr_a, zr_b)) {
        acc->cum_sector += 1.0f;
    }
    else if (is_in(acc->raw_pos, zl_a, zl_b)) {
        acc->cum_sector -= 1.0f;
    }
}

void update_cum_pos(rotation_accumulator_t *acc) {
    int jumpovers = acc->cum_sector / 4;
    if (raw_sector(acc) == 0 && acc->raw_pos > 0.75f) {
        jumpovers -= 1;
    }
    acc->cum_pos = (float)jumpovers + acc->raw_pos;
}

void rotation_accumulator_init(rotation_accumulator_t *acc, uint8_t sector) {
    if (!acc) return;
    acc->cum_sector = sector;
    acc->cum_pos    = 0;
    acc->raw_pos    = 0;
}

void rotation_accumulator_update_raw_pos(rotation_accumulator_t *acc, float raw_pos) {
    if (!acc) return;
    acc->raw_pos = raw_pos;
    update_cum_sector(acc);
    update_cum_pos(acc);
}
