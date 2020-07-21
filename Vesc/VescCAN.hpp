#ifndef __VESCCAN_HPP
#define __VESCCAN_HPP

#include "mbed.h"
#include "rtos.h"
#include "ROBOT_CONFIG.hpp"
#include "MOAB_DEFINITIONS.h"

class VescCan
{
	struct vesc_data{
		float _reported_rpmR;
		float _reported_rpmL;
	};

	public:

		VescCan(PinName, PinName, UDPSocket*);

		vesc_data report_data; 

		void Start();

		void setRPMs(float , float);

		void vehicleControlProportionalMixing(int UD_ch, int LR_ch, float MotorRPM[2]);

#ifdef _FUTABA

        int MIN_STICK = 360;       
        int MAX_STICK = 1673;      

        int MIN_DEADBAND = 1014;
        int MAX_DEADBAND = 1034;

        int MID_STICK = 1024;
        int DIVIDER = 2;           		// a divider of another wheel's speed, 
        								// e.g. 2 is half speed of the another wheel's speed
	#ifdef _XWHEELS
        float MAX_RPM = 136.0;          // XWheels -> 136.0
        								// Flipsky's wheels -> I guess even >500 but keep it safe as 300
	#endif
	#ifdef _OFFROAD
		float MAX_RPM = 300.0; 
	#endif						 	
        							 	  
        float ZERO_RPM = 0.0;           

#endif

#ifdef _LTE_PROPO

        int MIN_STICK = 283;     
        int MAX_STICK = 1758;    

        int MIN_DEADBAND = 964;			//924;
        int MAX_DEADBAND = 1084;		//1124;

        int MID_STICK = 1024;
        int DIVIDER = 2;           		// a divider of another wheel's speed, 
        								// e.g. 2 is half speed of the another wheel's speed

	#ifdef _XWHEELS
        float MAX_RPM = 136.0;          // XWheels -> 136.0
        								// Flipsky's wheels -> I guess even >500 but keep it safe as 300
	#endif
	#ifdef _OFFROAD
		float MAX_RPM = 300.0; 
	#endif	

        float ZERO_RPM = 0.0;           

#endif

	private:

		PinName _td_pin;
        PinName _rd_pin;

        CAN *_can;
        RawSerial *_usb_debug;

        //CANMessage _rx_msg_buffer;

        Thread main_thread;

        UDPSocket *_sock;

        void main_worker();

        void Int16ToByteData(unsigned int Data, unsigned char StoreByte[2]);

		void Int32ToByteData(int32_t Data, unsigned char StoreByte[4]);

		uint16_t ByteDataToInt16(unsigned char ByteData[2]);

		int32_t ByteDataToInt32(unsigned char ByteData[4]);

		void CANErrorCheck();

        void canWrite(unsigned int _ID, char data[8]);

		void canWriteEx(unsigned int _ID, char data[8]);

		void setRPM(uint8_t vesc_id, float rpm);

		void setDuty(uint8_t vesc_id, float duty);

		//bool readReply(unsigned int _ID, unsigned char output[8]);
		void readReply();

        float RPM_TO_ERPM(float rpm);

		float ERPM_TO_RPM(float erpm);

		float RPM_TO_DUTY(float rpm);

        //void Rx_Interrupt();

        float _vesc0_rpm;
        float _vesc1_rpm;

        float _rpmR;
		float _rpmL;

		float _read_rpmR;
		float _read_rpmL;

		float _prev_read_rpmR;
		float _prev_read_rpmL;

#ifdef _XWHEELS
		float _ERPM_ratio = 140.0;   // for big wheels (XWheels's hub)  1 RPM -> 140 ERPM
									 // for small off-road tire (Flipsky's wheels set)  1 RPM -> 33.333 ERPM
#endif
#ifdef _OFFROAD
		float _ERPM_ratio = 33.333;   // for big wheels (XWheels's hub)  1 RPM -> 140 ERPM
									 // for small off-road tire (Flipsky's wheels set)  1 RPM -> 33.333 ERPM
#endif
		float MAX_DUTY = 1.0;

		float _prev_report_rpmR = 0.0;
		float _prev_report_rpmL = 0.0;

		float _pre_Y;

};

#endif