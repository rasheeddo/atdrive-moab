#ifndef __MUX_MODULE_HPP
#define __MUX_MODULE_HPP

#include "mbed.h"
#include "EthernetInterface.h"
#include "rtos.h"
#include <string>
#include <sstream>
#include <stdint.h>
#include <cstdlib>
#include "ROBOT_CONFIG.hpp"
#include "MOAB_DEFINITIONS.h"


struct I2C_MUX_BOARD_COMMON_t{
    uint32_t current1;
    uint32_t current2;
    int32_t  tmp1;
    int32_t  tmp2;
};

struct I2C_MUX_BOARD_BATTINFO_t{
	uint16_t temp;
	uint16_t volt;
	int32_t current;
	uint8_t SOC;
	uint8_t SOH;
	uint16_t CellVoltage6;
	uint16_t CellVoltage5;
	uint16_t CellVoltage4;
	uint16_t CellVoltage3;
	uint16_t CellVoltage2;
	uint16_t CellVoltage1;
	uint8_t PFAlert;
	uint8_t PFStatus;
	uint16_t CycleCount;
	uint16_t DesignVoltage;
	uint16_t SerialNumber;
	uint16_t ManufacturerDate;
	uint8_t MaxTempCell;
	uint16_t LastFCCUpdate;
	uint16_t TotalFwRuntime;
	uint16_t TimeSpentinOT;
};



class MUXBoard_daemon {

	public:

		MUXBoard_daemon(PinName , PinName, UDPSocket*);

		void Start();

	private:

		RawSerial *_bat_mon;
		RawSerial *_usb_debug;

		struct I2C_MUX_BOARD_COMMON_t I2C_MUX_BOARD_common;
		struct I2C_MUX_BOARD_BATTINFO_t I2C_MUX_BOARD_BattInfo[6];

		UDPSocket *_sock;

		Thread main_thread;

		std::string readString_battMonitor();

		void main_worker();

		void recvBatteryInfo(char *cstr, uint8_t ch);

		void recvCurrentInfo(char *cstr, uint8_t ch);

		void recvTmpInfo(char *cstr, uint8_t ch);

		int8_t recvInfo(void);
};



#endif
