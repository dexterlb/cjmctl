#include "rotation_accumulator.h"
#include "math_utils.h"

static const float zone_left_start_per_sector[4]  = {0.5f, 0.75f, 0.0f, 0.25f};
static const float zone_left_end_per_sector[4]    = {0.75f, 1.0f, 0.25f, 0.5f};
static const float zone_right_start_per_sector[4] = {0.25f, 0.5f, 0.75f, 0.0f};
static const float zone_right_end_per_sector[4]   = {0.5f, 0.75f, 1.0f, 0.25f};

void reset_sector(rotation_accumulator_t* acc, float pos) {
	// if (is_in(pos, 0, 0.25)) {
	// 	acc->accum_sector = 0;
	// } else if (is_in(pos, 0.25, 0.5)) {
	// 	acc->accum_sector = 1;
	// } else if (is_in(pos, 0.5, 0.75)) {
	// 	acc->accum_sector = 2;
	// } else if (is_in(pos, 0.75, 1.0)) {
	// 	acc->accum_sector = 3;
	// }

	// test this tomorrow
	acc->accum_sector = 0;
	update_accum_sector(acc);
}

int16_t raw_sector(const rotation_accumulator_t* acc) {
	return positive_mod((int)acc->accum_sector, 4);
}

void calc_sector_transition_zones(const rotation_accumulator_t* acc, float* zone_left_start, float* zone_left_end, float* zone_right_start, float* zone_right_end) {
	int16_t sector    = raw_sector(acc);
	*zone_left_start  = zone_left_start_per_sector[sector];
	*zone_left_end    = zone_left_end_per_sector[sector];
	*zone_right_start = zone_right_start_per_sector[sector];
	*zone_right_end   = zone_right_end_per_sector[sector];
}

void update_accum_sector(rotation_accumulator_t* acc) {
	float zl_start, zl_end, zr_start, zr_end;
	calc_sector_transition_zones(acc, &zl_start, &zl_end, &zr_start, &zr_end);

	if (is_in(acc->raw_pos, zr_start, zr_end)) {
		acc->accum_sector++;
	} else if (is_in(acc->raw_pos, zl_start, zl_end)) {
		acc->accum_sector--;
	}
}

void update_accum_pos(rotation_accumulator_t* acc) {
	int jumpovers = acc->accum_sector / 4;
	if (raw_sector(acc) == 0 && acc->raw_pos > 0.75f) {
		jumpovers -= 1;
	}
	acc->accum_pos = (float)jumpovers + acc->raw_pos;
}

void rotation_accumulator_init(rotation_accumulator_t* acc, int16_t sector) {
	if (!acc) {
		return;
	}
	acc->accum_sector = sector;
	acc->accum_pos    = 0;
	acc->raw_pos      = 0;
}

void rotation_accumulator_update_raw_pos(rotation_accumulator_t* acc, float raw_pos) {
	if (!acc) {
		return;
	}
	acc->raw_pos = raw_pos;
	update_accum_sector(acc);
	update_accum_pos(acc);
}
