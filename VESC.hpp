#ifndef __VESC_HPP
#define __VESC_HPP

#include "mbed.h"
#include "rtos.h"
#include "ROBOT_CONFIG.hpp"
#include "MOAB_DEFINITIONS.h"
#include "VescUart/VescUart.h"

// Reported values to auto-pilot
struct vescs_data{
	float _inpVoltage;
	float _avgInputCurrent;
	float _rpmR;
	float _rpmL;
};

class VESC{

	struct dataPackage {
		float avgMotorCurrent;
		float avgInputCurrent;
		float dutyCycleNow;
		long rpm;
		float inpVoltage;
		float ampHours;
		float ampHoursCharged;
		long tachometer;
		long tachometerAbs;
	};

	public:

		VESC(UDPSocket*);

		void Start();

		void setRPMs(float , float);

		void vehicleControl(int UD_ch, int LR_ch, float MotorRPM[2]);

		dataPackage vesc0_data;
		dataPackage vesc1_data;

#ifdef _FUTABA

        int MIN_STICK = 360;       
        int MAX_STICK = 1673;      

        int MIN_DEADBAND = 1014;
        int MAX_DEADBAND = 1034;

        int MID_STICK = 1024;
        int DIVIDER = 2;           		// a divider of another wheel's speed, 
        								// e.g. 2 is half speed of the another wheel's speed

        float MAX_RPM = 100.0;          // Max RPM of the wheels, this is limited by wheels itself.
        float ZERO_RPM = 0.0;           // this may need to adjust according to each robot,

#endif


	private:

		RawSerial *_usb_debug;
		UDPSocket *_sock;

		Thread main_thread;

		struct vescs_data vescs_reported_data;

		void main_worker();

		float RPM_TO_ERPM(float rpm);

		float ERPM_TO_RPM(float erpm);

		float _rpmR;
		float _rpmL;

		// each wheel has different EPRM values
		float _ERPM_ratio = 140.0;   // for big wheels (XWheels's hub)  1 RPM -> 140 ERPM

		float _MIN_PWM = 0.001000;  // 1000ms

		float _MAX_PWM = 0.002000;  // 2000ms


};


#endif