#ifndef _VESCUART_h
#define _VESCUART_h

#include "mbed.h"
#include "rtos.h"
#include "datatypes.h"
#include "buffer.h"
#include "crc.h"
#include "ROBOT_CONFIG.hpp"
#include "MOAB_DEFINITIONS.h"
#include "platform/CircularBuffer.h"

class VescUart
{
	/** Struct to store the telemetry data returned by the VESC */
	struct dataPackage {
		float avgMotorCurrent;
		float avgInputCurrent;
		float dutyCycleNow;
		float rpm;
		float inpVoltage;
		float ampHours;
		float ampHoursCharged;
		long tachometer;
		long tachometerAbs;
	};

	struct vesc_data{
		float _reported_rpmR;
		float _reported_rpmL;
	};

	public:

		VescUart(PinName, PinName, UDPSocket*);

		dataPackage data;
		dataPackage data1;
		vesc_data report_data;  

		
		void Start();

		// set speed of two wheels
		void setRPMs(float , float);

		// set speed of two wheels in case manual is true flag, auto is false flag
		void setRPMs(float , float, bool);

		// convert SBUS stick value to motor RPM of each wheel
		void vehicleControl(int UD_ch, int LR_ch, float MotorRPM[2]);

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

        int MIN_DEADBAND = 924;
        int MAX_DEADBAND = 1124;

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

		PinName _tx_pin;
        PinName _rx_pin;

        RawSerial *_uart;
        RawSerial *_usb_debug;

        Timer _timer;

        // CircularBuffer<char, MUX_BUF_SIZE> *_rxbuf;

        Thread main_thread;

        UDPSocket *_sock;
		
		////////////////////////////////////////////////////////
		//-------------- My modified functions -------------- //
		////////////////////////////////////////////////////////

        void main_worker();

        void _Serial_Rx_Interrupt();

		float RPM_TO_ERPM(float rpm);

		float ERPM_TO_RPM(float erpm);

		float RPM_TO_DUTY(float rpm);

		/////////////////////////////////////////////////////////////////////////
		//--------------- Original functions from SolidGeek -------------------//
		//        check this out https://github.com/SolidGeek/VescUart         //
		/////////////////////////////////////////////////////////////////////////

		int packSendPayload(uint8_t * payload, int lenPay);

		int receiveUartMessage(uint8_t * payloadReceived);

		bool unpackPayload(uint8_t * message, int lenMes, uint8_t * payload);

		bool processReadPacket(uint8_t * message);

		bool processReadPacket(uint8_t * message, int vescID);

		void serialPrint(uint8_t * data, int len);

		bool getVescValues(void);

		bool getVescValues(int vescID);

		void setNunchuckValues(void);

		void setCurrent(float current);

		void setBrakeCurrent(float brakeCurrent);

		void setRPM(float rpm);

		void setDuty(float duty);

		void printVescValues(void);

		////////////////////////////////////////////////////////
		//-------------- My modified functions -------------- //
		////////////////////////////////////////////////////////

		void setRPM(float rpm, int vescID);

		void setDuty(float duty, int vescID);

		void askForValues();

		void askForValues(int vescID);

		void recvUartWorker();

		#define _RING_BUFFER_SIZE 156   // 78bytes for each ESC's reply
		
		uint8_t _ringBuf[_RING_BUFFER_SIZE];

		int _count = 0;
		bool _readyToAsk = true;
		float _rpmR;
		float _rpmL;
		bool _man_flag = true;

		uint8_t _vesc0Buf[78];
		uint8_t _vesc1Buf[78];

#ifdef _XWHEELS
		float _ERPM_ratio = 140.0;   // for big wheels (XWheels's hub)  1 RPM -> 140 ERPM
									 // for small off-road tire (Flipsky's wheels set)  1 RPM -> 33.333 ERPM
#endif
#ifdef _OFFROAD
		float _ERPM_ratio = 33.333;   // for big wheels (XWheels's hub)  1 RPM -> 140 ERPM
									 // for small off-road tire (Flipsky's wheels set)  1 RPM -> 33.333 ERPM
#endif
		float MAX_DUTY = 1.0;

		float _period;
		
};

#endif
