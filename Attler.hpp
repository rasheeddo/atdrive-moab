#ifndef __ATTLER_H
#define __ATTLER_H

#include "mbed.h"
#include "ROBOT_CONFIG.hpp"

class Attler {

public:
	Attler(PinName, PinName);
	long map(long x, long in_min, long in_max, long out_min, long out_max);
	void set_steering_and_throttle(uint16_t, uint16_t);
	//void set_steering(uint16_t);
	//void set_throttle(uint16_t);
	void driveVehicle(float leftWheel, float rightWheel);
	uint16_t get_value_a(void);
	uint16_t get_value_b(void);
	float get_pw_a(void);
	float get_pw_b(void);

private:
	PwmOut *_motor_A;
	PwmOut *_motor_B;

	uint16_t _prev_value_a;
	uint16_t _prev_value_b;

	float _pw_a;
	float _pw_b;

	int SBUS_CENTER = 1024;

	int SBUS_MAX = 1673;

	int SBUS_MIN = 360;      //352

	int SBUS_MIN_DB = 978;   // -46 from 1024
	int SBUS_MAX_DB = 1070;  // +46 from 1024

	float _LEFT_PW_CENTER = 0.001520; //1515
	float _LEFT_PW_RANGE = 0.000400;  //400
	const float _LEFT_PW_MAX = _LEFT_PW_CENTER + _LEFT_PW_RANGE;
	const float _LEFT_PW_MIN = _LEFT_PW_CENTER - _LEFT_PW_RANGE;

	float _RIGHT_PW_CENTER = 0.001520; //1515
	float _RIGHT_PW_RANGE = 0.000400; //400
	const float _RIGHT_PW_MAX = _RIGHT_PW_CENTER + _RIGHT_PW_RANGE;
	const float _RIGHT_PW_MIN = _RIGHT_PW_CENTER - _RIGHT_PW_RANGE;

	// If you push forward full throttle and there is one wheel turn wrong, changes the sign of that wheels
	// the default is, PWM1 is right wheel, PWM2 is left wheel
	int R_DIR = 1.0;    // choose 1.0 or -1.0
	int L_DIR = -1.0;   // choose 1.0 or -1.0
	int CURVE_SIGN = -1.0;

};


#endif // __ATTLER_H
