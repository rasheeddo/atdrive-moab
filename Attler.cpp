
#include "Attler.hpp"
//#include "ROBOT_CONFIG.hpp"

extern void u_printf(const char *fmt, ...);  // Defined in main()

Attler::Attler(PinName a, PinName b) {

	_motor_A = new PwmOut(a);
	_motor_B = new PwmOut(b);

	_motor_A->period_ms(15);  // same period as S.Bus
	_motor_B->period_ms(15);  // same period as S.Bus

	_prev_value_a = 0;
	_prev_value_b = 0;

	_pw_a = 0.0;
	_pw_b = 0.0;
}

long Attler::map(long x, long in_min, long in_max, long out_min, long out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Attler::set_steering_and_throttle(uint16_t steer_value, uint16_t throt_value) {

	
	if (steer_value > SBUS_MAX) {
		steer_value = SBUS_MAX;
	} else if (steer_value < SBUS_MIN) {
		steer_value = SBUS_MIN;
	}

	if (throt_value > SBUS_MAX) {
		throt_value = SBUS_MAX;
	} else if (throt_value < SBUS_MIN) {
		throt_value = SBUS_MIN;
	}

	if ((_prev_value_a == steer_value) && (_prev_value_b == throt_value)) {
		return;
	}

	_prev_value_a = steer_value;
	_prev_value_b = throt_value;

	
	// This is a value between -1.0 and +1.0:
	//float steer_percentValue = ((float) steer_value - 1024.0) / 672.0;  // -1 left, +1 right
	//float throt_percentValue = ((float) throt_value - 1024.0) / 672.0;  // -1 backwards, +1 forward
	// each channel of stick has different MAX MIN of SBUS value, shouldn't use constant value as 672.0 or anything here
	// just change SBUS_MIN and SBUS_MAX on top, then this would be nicer I think :)
	float steer_percentValue = map(steer_value, SBUS_MIN, SBUS_MAX, -1000, 1000);
	float throt_percentValue = map(throt_value, SBUS_MIN, SBUS_MAX, -1000, 1000);
	steer_percentValue = steer_percentValue/1000.0;
	throt_percentValue = throt_percentValue/1000.0;

	/*
	// This cannot make a good curvy motion
	float leftWheel = throt_percentValue + steer_percentValue;
	float rightWheel = throt_percentValue - steer_percentValue;
	
	if (leftWheel > 1.0) {
		leftWheel = 1.0;
	} else if (leftWheel < -1.0) {
		leftWheel = -1.0;
	}

	if (rightWheel > 1.0) {
		rightWheel = 1.0;
	} else if (rightWheel < -1.0) {
		rightWheel = -1.0;
	}

	*/
	float MIN_SCALER = 1000.0;
    float MAX_SCALER = 2000.0;   //ATCart is 2000, change this will change the varying speed of a slower wheel when drive in curvy
    							 // attler2 is 5500
    float leftWheel;
    float rightWheel;
    float curve_throt_percent = 0.7;
	/////////////////////////////////////////////////////// STRAIGHT DRIVE ////////////////////////////////////////////////////////////////
    // In case the stick near mid for both ch2 and ch4
    if (steer_value <= SBUS_MAX_DB && steer_value >= SBUS_MIN_DB && throt_value <= SBUS_MAX_DB && throt_value >= SBUS_MIN_DB)
    {
        leftWheel = 0.0;
        rightWheel = 0.0;
    }

    // user push ch2 up or down, UGV drive forward or backward, two wheels same speed and direction
    else if(steer_value <= SBUS_MAX_DB && steer_value >= SBUS_MIN_DB && (throt_value > SBUS_MAX_DB || throt_value < SBUS_MIN_DB))
    {
        leftWheel = L_DIR*throt_percentValue;
        rightWheel = R_DIR*throt_percentValue;

    }
    /////////////////////////////////////////////////////////// TURNS /////////////////////////////////////////////////////////////////////
    // user push ch4 left or right, UGV turns left or right, two wheels same speed but reverse direction
    else if(throt_value <= SBUS_MAX_DB && throt_value >= SBUS_MIN_DB && (steer_value >= SBUS_MAX_DB || steer_value <= SBUS_MIN_DB))
    {
        leftWheel = L_DIR*steer_percentValue;
        rightWheel = -R_DIR*steer_percentValue;
    }
    /////////////////////////////////////////////////////////// CURVES /////////////////////////////////////////////////////////////////////
    // user push both ch2 and ch4 diagonally (first quadrant), UGV curves to the right forward, one wheels is half speed of the another one
    else if(throt_value > SBUS_MAX_DB && steer_value > SBUS_MAX_DB)
    {
        leftWheel = L_DIR*throt_percentValue * curve_throt_percent;  // leftWheel is faster
        float SCALE = (float)map(steer_value, SBUS_MAX_DB+1, SBUS_MAX, MIN_SCALER, MAX_SCALER);
        rightWheel = CURVE_SIGN*R_DIR*leftWheel*MIN_SCALER/SCALE;   //rightWheel is slower
    } 

     // user push both ch2 and ch4 diagonally (second quadrant), UGV curves to the left forward, one wheels is half speed of the another one
    else if(throt_value > SBUS_MAX_DB && steer_value < SBUS_MIN_DB)
    {
        rightWheel = R_DIR*throt_percentValue * curve_throt_percent;
        float SCALE = (float)map(steer_value, SBUS_MIN_DB-1, SBUS_MIN, MIN_SCALER, MAX_SCALER);
        leftWheel = -CURVE_SIGN*L_DIR*rightWheel*MIN_SCALER/SCALE;
    }   

    // user push both ch2 and ch4 diagonally (third quadrant), UGV curves to the left backward, one wheels is half speed of the another one
    else if(throt_value < SBUS_MIN_DB && steer_value < SBUS_MIN_DB)
    {
        rightWheel = R_DIR*throt_percentValue * curve_throt_percent;
        float SCALE = (float)map(steer_value, SBUS_MIN_DB-1, SBUS_MIN, MIN_SCALER, MAX_SCALER);
        leftWheel = -CURVE_SIGN*L_DIR*rightWheel*MIN_SCALER/SCALE;
    }

     // user push both ch2 and ch4 diagonally (fourth quadrant), UGV curves to the right backward, one wheels is half speed of the another one
    else if(throt_value < SBUS_MIN_DB && steer_value > SBUS_MAX_DB)
    {
        leftWheel = L_DIR*throt_percentValue * curve_throt_percent;
        float SCALE = (float)map(steer_value, SBUS_MAX_DB+1, SBUS_MAX, MIN_SCALER, MAX_SCALER);
        rightWheel = CURVE_SIGN*R_DIR*leftWheel*MIN_SCALER/SCALE;
    }

	//_pw_a = leftWheel * _LEFT_PW_RANGE + _LEFT_PW_CENTER;
	//_pw_b = rightWheel * _RIGHT_PW_RANGE + _RIGHT_PW_CENTER;

	//printf("steer_percentValue %f   throt_percentValue %f ", steer_percentValue, throt_percentValue);

	//printf("leftWheel %f    rightWheel %f  _pw_a %f   _pw_b %f\n", leftWheel, rightWheel, _pw_a, _pw_b);
	//printf("\n");
	//printf("steer_value %d   throt_value %d    _pw_a %f   _pw_b %f\n", steer_value, throt_value, _pw_a, _pw_b);
	//printf("\n");


	driveVehicle(leftWheel, rightWheel);
	
	
}

void Attler::driveVehicle(float leftWheel, float rightWheel){

	float _pw_b = leftWheel * _LEFT_PW_RANGE + _LEFT_PW_CENTER;
	float _pw_a = rightWheel * _RIGHT_PW_RANGE + _RIGHT_PW_CENTER;

	_motor_A->pulsewidth(_pw_a);
	_motor_B->pulsewidth(_pw_b);
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

