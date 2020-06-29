/*#ifndef __VESC_HPP
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

		//VESC(UDPSocket*, PinName, PinName);
		VESC(UDPSocket*);

		void Start();

		// set speed of two wheels
		void setRPMs(float , float);

		// set speed of two wheels in case manual is true flag, auto is false flag
		void setRPMs(float , float, bool);

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

        float MAX_RPM = 136.0;          // VESC is 100.0 in case of internal PID speed controller
        							 	// Dual FSESC PID speed controller is not good,
        							 	// so we use Duty Cycle control instead     
        float ZERO_RPM = 0.0;           

#endif

#ifdef _LTE_PROPO

        int MIN_STICK = 283;     
        int MAX_STICK = 1758;    

        int MIN_DEADBAND = 924;
        int MAX_DEADBAND = 1124;

        int MID_STICK = 1024;
        int DIVIDER = 2;           		// a divider of another wheel's speed, 
        								// e.g. 2 is half speed of the another wheel's speed

        float MAX_RPM = 136.0;          // VESC is 100.0 in case of internal PID speed controller
        							 	// Dual FSESC PID speed controller is not good,
        							 	// so we use Duty Cycle control instead     
        float ZERO_RPM = 0.0;           

#endif


	private:

		RawSerial *_usb_debug;
		UDPSocket *_sock;

		//PwmOut *_motor_A;
		//PwmOut *_motor_B;

		Timer _timer;

		Thread main_thread;

		struct vescs_data vescs_reported_data;

		void main_worker();

		float RPM_TO_ERPM(float rpm);

		float ERPM_TO_RPM(float erpm);

		float RPM_TO_DUTY(float rpm);

		float _rpmR;
		float _rpmL;
		bool _man_flag = true;

		float _percentR;
		float _percentL;

		float MAX_DUTY = 1.0;

		// each wheel has different EPRM values
		float _ERPM_ratio = 140.0;   // for big wheels (XWheels's hub)  1 RPM -> 140 ERPM

		float read_rpm0;
		float read_rpm1;
		float in_voltage0;
		float in_voltage1;
		float in_current0;
		float in_current1;

		float _period;


};


#endif*/