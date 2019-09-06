
#include "BNO055.hpp"


BNO055::BNO055(I2C *i2c) {
	_i2c = i2c;

	_addr8bit = 0x28 << 1;
}



int BNO055::read_reg_u8(uint8_t reg, uint8_t *value) {
	char txBuf[2];
	char rxBuf[2] = {0, 0};

	txBuf[0] = reg;
	_i2c->write(_addr8bit, txBuf, 1);
	_i2c->read(_addr8bit, rxBuf, 1);

	*value = rxBuf[0];

	return 0;
}


int BNO055::write_reg_u8(uint8_t reg, uint8_t value) {
	char txBuf[2] = {reg, value};

	//txBuf[0] = reg;
	//txBuf[1] = value;
	_i2c->write(_addr8bit, txBuf, 2);

	return 0;
}


int BNO055::read_reg_u16(uint8_t reg, uint16_t *value) {
	char txBuf[2];
	char rxBuf[2] = {0, 0};

	txBuf[0] = reg;
	_i2c->write(_addr8bit, txBuf, 1);
	_i2c->read(_addr8bit, rxBuf, 2);

	*value = rxBuf[0] | rxBuf[1] << 8;

	return 0;
}


int BNO055::read_reg_s16(uint8_t reg, int16_t *value) {
	char txBuf[2];
	char rxBuf[2] = {0, 0};

	txBuf[0] = reg;
	_i2c->write(_addr8bit, txBuf, 1);
	_i2c->read(_addr8bit, rxBuf, 2);

	*value = (int16_t) (rxBuf[0] | rxBuf[1] << 8);

	return 0;
}



ssize_t BNO055::init() {
	uint8_t value;

	// BNO055 Chip ID
	read_reg_u8(0x00, &value);
	if (value != 0xa0) {
		return -1;
	}

	// ACC chip ID
	read_reg_u8(0x01, &value);
	if (value != 0xfb) {
		return -2;
	}

	// MAG chip ID
	read_reg_u8(0x02, &value);
	if (value != 0x32) {
		return -3;
	}

	// GYRO chip ID
	read_reg_u8(0x03, &value);
	if (value != 0x0f) {
		return -4;
	}


	// Set to config mode
	write_reg_u8(0x3d, 0x00);

	// There are 3 fusion modes that we care about:
	// 0x08 -- Fusion-IMU
	//   6dof IMU
	//    heading is only relative because no compass
	//    pitch and roll are absolute because gravity
	//
	// 0x0b -- Fusion-NDOF_FMC_OFF
	//   9dof sensor fusion, "Fast Magnet Calibration" is off
	//    gives us an absolute orientation because gravity and compass
	//
	// 0x0c -- Fusion-NDOF
	//   9dof sensor fusion with fast magnet calibration on
	//    gives us an absolute orientation because gravity and compass

	// All 3 of these modes have a maximum data rate of 100 Hz.

	// Set the operating mode:
	//write_reg_u8(0x3d, 0x08);  // 6dof IMU, no compass
	write_reg_u8(0x3d, 0x0b);  // 9dof IMU, slow magnet calibration
	//write_reg_u8(0x3d, 0x0c);  // 9dof IMU, fast magnet calibration

	_ready = true;

	return 0;
}


int BNO055::get_data(char* buf) {
	if (!_ready) {
		return -1;
	}

	char txBuf[2];

	txBuf[0] = 0x20;
	_i2c->write(_addr8bit, txBuf, 1);
	_i2c->read(_addr8bit, buf, 20);

	return 20;
}



