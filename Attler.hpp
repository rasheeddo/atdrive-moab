#ifndef __ATTLER_H
#define __ATTLER_H

#include "mbed.h"
#include "ROBOT_CONFIG.hpp"

class Attler {

public:
	Attler(PinName, PinName);
	float map(float x, float in_min, float in_max, float out_min, float out_max);
	void set_steering_and_throttle(uint16_t, uint16_t);
	void set_steering(uint16_t sbus_steer);
	void set_throttle(uint16_t sbus_throt);
	void driveVehicle(float leftWheel, float rightWheel);
	uint16_t get_value_a(void);
	uint16_t get_value_b(void);
	float get_pw_a(void);
	float get_pw_b(void);



private:
	PwmOut *_PWM1;
	PwmOut *_PWM2;

	RawSerial *_usb_debug;

	uint16_t _prev_value_a;
	uint16_t _prev_value_b;

	float _pw_a;
	float _pw_b;

	int SBUS_CENTER = 1024;

	int SBUS_MAX = 1673;

	int SBUS_MIN = 360;      //352

	int SBUS_MIN_DB = 978;   // -46 from 1024
	int SBUS_MAX_DB = 1070;  // +46 from 1024

	float PWM_MIN = 0.001000;
	float PWM_MAX = 0.002000;

	/*
	float _LEFT_PW_CENTER = 0.001520; //1515
	float _LEFT_PW_RANGE = 0.000400;  //400
	const float _LEFT_PW_MAX = _LEFT_PW_CENTER + _LEFT_PW_RANGE;
	const float _LEFT_PW_MIN = _LEFT_PW_CENTER - _LEFT_PW_RANGE;

	float _RIGHT_PW_CENTER = 0.001520; //1515
	float _RIGHT_PW_RANGE = 0.000400; //400
	const float _RIGHT_PW_MAX = _RIGHT_PW_CENTER + _RIGHT_PW_RANGE;
	const float _RIGHT_PW_MIN = _RIGHT_PW_CENTER - _RIGHT_PW_RANGE;
*/
};


#endif // __ATTLER_H
