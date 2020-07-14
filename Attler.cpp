
#include "Attler.hpp"
//#include "ROBOT_CONFIG.hpp"

extern void u_printf(const char *fmt, ...);  // Defined in main()

Attler::Attler(PinName a, PinName b) {

	_PWM1 = new PwmOut(a);
	_PWM2 = new PwmOut(b);

	_PWM1->period_ms(15);  // same period as S.Bus
	_PWM2->period_ms(15);  // same period as S.Bus

	_prev_value_a = 0;
	_prev_value_b = 0;

	_pw_a = 0.0;
	_pw_b = 0.0;

	//_usb_debug = new RawSerial(USBTX,USBRX,115200);
}

float Attler::map(float x, float in_min, float in_max, float out_min, float out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Attler::set_steering(uint16_t sbus_steer){

	if (sbus_steer > SBUS_MAX) {
		sbus_steer = SBUS_MAX;
	} else if (sbus_steer < SBUS_MIN) {
		sbus_steer = SBUS_MIN;
	}

	_prev_value_a = sbus_steer;

	float pwm_steer = (float)map(sbus_steer, SBUS_MIN, SBUS_MAX, PWM_MIN, PWM_MAX);

	//_usb_debug->printf("pwm_steer: %f\n", pwm_steer);

	_PWM1->pulsewidth(pwm_steer);

}

void Attler::set_throttle(uint16_t sbus_throt){

	if (sbus_throt > SBUS_MAX) {
		sbus_throt = SBUS_MAX;
	} else if (sbus_throt < SBUS_MIN) {
		sbus_throt = SBUS_MIN;
	}

	_prev_value_b = sbus_throt;

	float pwm_throt = (float)map(sbus_throt, SBUS_MIN, SBUS_MAX, PWM_MIN, PWM_MAX);

	//_usb_debug->printf("pwm_throt: %f\n", pwm_throt);

	_PWM2->pulsewidth(pwm_throt);

}


uint16_t Attler::get_value_a(void) {
	return _prev_value_a;
}

uint16_t Attler::get_value_b(void) {
	return _prev_value_b;
}

float Attler::get_pw_a(void) {
	return _pw_a;
}

float Attler::get_pw_b(void) {
	return _pw_b;
}
