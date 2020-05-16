#include "misc_math.hpp"


void calc_sbus_to_skid_mode(
	uint16_t steering,
	uint16_t throttle,
	float* left_wheel,
	float* right_wheel) {

	float steer_percentValue = ((float) steering - 1024.0) / 672.0;  // -1 left, +1 right
	float throt_percentValue = ((float) throttle - 1024.0) / 672.0;  // -1 backwards, +1 forward

	*left_wheel = throt_percentValue + steer_percentValue;
	*right_wheel = throt_percentValue - steer_percentValue;

	if (*left_wheel > 1.0) {
		*left_wheel = 1.0;
	} else if (*left_wheel < -1.0) {
		*left_wheel = -1.0;
	}

	if (*right_wheel > 1.0) {
		*right_wheel = 1.0;
	} else if (*right_wheel < -1.0) {
		*right_wheel = -1.0;
	}
}

