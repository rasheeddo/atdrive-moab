#ifndef _VESCUART_h
#define _VESCUART_h

#include "mbed.h"
#include "rtos.h"
#include "datatypes.h"
#include "buffer.h"
#include "crc.h"

class VescUart
{
	/** Struct to store the telemetry data returned by the VESC */
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
		/**
		 * @brief      Class constructor
		 */
		VescUart(PinName, PinName);

		/** Variabel to hold measurements returned from VESC */
		dataPackage data;
		dataPackage data1;  

		/**
		 * @brief      Sends a command to VESC and stores the returned data
		 *
		 * @return     True if successfull otherwise false
		 */
		bool getVescValues(void);

		bool getVescValues(int vescID);

		/**
		 * @brief      Sends values for joystick and buttons to the nunchuck app
		 */
		void setNunchuckValues(void);

		/**
		 * @brief      Set the current to drive the motor
		 * @param      current  - The current to apply
		 */
		void setCurrent(float current);

		/**
		 * @brief      Set the current to brake the motor
		 * @param      brakeCurrent  - The current to apply
		 */
		void setBrakeCurrent(float brakeCurrent);

		/**
		 * @brief      Set the rpm of the motor
		 * @param      rpm  - The desired RPM (actually eRPM = RPM * poles)
		 */
		void setRPM(float rpm);

		void setRPM(float rpm, int vescID);

		/**
		 * @brief      Set the duty of the motor
		 * @param      duty  - The desired duty (0.0-1.0)
		 */
		void setDuty(float duty);

		void setDuty(float duty, int vescID);

		/**
		 * @brief      Help Function to print struct dataPackage over Serial for Debug
		 */
		void printVescValues(void);

	private: 

		PinName _tx_pin;
        PinName _rx_pin;

        RawSerial *_uart;
        RawSerial *_usb_debug;

		/**
		 * @brief      Packs the payload and sends it over Serial
		 *
		 * @param      payload  - The payload as a unit8_t Array with length of int lenPayload
		 * @param      lenPay   - Length of payload
		 * @return     The number of bytes send
		 */
		int packSendPayload(uint8_t * payload, int lenPay);

		/**
		 * @brief      Receives the message over Serial
		 *
		 * @param      payloadReceived  - The received payload as a unit8_t Array
		 * @return     The number of bytes receeived within the payload
		 */
		int receiveUartMessage(uint8_t * payloadReceived);

		/**
		 * @brief      Verifies the message (CRC-16) and extracts the payload
		 *
		 * @param      message  - The received UART message
		 * @param      lenMes   - The lenght of the message
		 * @param      payload  - The final payload ready to extract data from
		 * @return     True if the process was a success
		 */
		bool unpackPayload(uint8_t * message, int lenMes, uint8_t * payload);

		/**
		 * @brief      Extracts the data from the received payload
		 *
		 * @param      message  - The payload to extract data from
		 * @return     True if the process was a success
		 */
		bool processReadPacket(uint8_t * message);

		bool processReadPacket(uint8_t * message, int vescID);

		/**
		 * @brief      Help Function to print uint8_t array over Serial for Debug
		 *
		 * @param      data  - Data array to print
		 * @param      len   - Lenght of the array to print
		 */
		void serialPrint(uint8_t * data, int len);

		
		char _recvBuf[_BUFFER_SIZE];
		
};

#endif
