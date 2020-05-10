#ifndef __IMU_MODULE_HPP
#define __IMU_MODULE_HPP

#include "mbed.h"
#include "EthernetInterface.h"

#include "ROBOT_CONFIG.hpp"
//#include "EVENT_FLAGS.hpp"
#include "MOAB_DEFINITIONS.h"

#include "ShaftEncoder.hpp"

#include "drivers/ST_LIS3MDL.hpp"
#include "drivers/BMP280.hpp"
#include "drivers/BNO055.hpp"


struct multi_data {

	// 64 bits:
	uint16_t version;
	int16_t compass_XYZ[3];  // external compass

	// 3 * 64 bits:
	char bnoData[22];  // internal IMU
	int16_t _padding2;  // the compiler seems to like 64-bit boundaries

	// 64 bits:
	float temperature; // degrees celsius, no need for high accuracy

	// Pressure:  typical sensor value is ~100000, with accuracy of +/- 12.0,
	// (don't forget to convert between Pa and hPa), so this is well
	// within the accuracy of float32
	float pressure;

	// 64 bits:
	uint16_t sbus_a;
	uint16_t sbus_b;
	uint16_t _padding3;  // 64-bit boundary
	uint16_t _padding4;  // 64-bit boundary

	// Everything ABOVE here is the official, "version 1" of this protocol
	// Everything BELOW here is extra, and might change in the future

	// 64 bits:
	// TODO:  do we really need float64 for these numbers?
#ifdef _TWO_SHAFT_ENCODERS
	double shaft_a_pps;
	double shaft_b_pps;
#else // _TWO_SHAFT_ENCODERS
	double shaft_pps;
#endif // _TWO_SHAFT_ENCODERS

};





class IMU_daemon {
public:
	IMU_daemon(UDPSocket*, uint16_t*, uint16_t*);

	void Start();

private:

	uint16_t *_sbus_a;
	uint16_t *_sbus_b;

	struct multi_data _mData;

	UDPSocket *_sock;

	Thread main_thread;

	void main_worker();
};



#endif // __IMU_MODULE_HPP
