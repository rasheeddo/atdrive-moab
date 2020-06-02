//#include "mbed.h"

#include "SbusParser.hpp"


SbusParser::SbusParser(struct sbus_udp_payload *sup) {
	_rxIdx = 0;
	_rxInBand = false;
	_sup = sup;
}


//int missedChars = 0;
int SbusParser::_processSbusMessage() {
	/*
	switch (_rxBuf[23]) {
	// my transmitter always has 1 of these 4 bytes:
	case 0x04:
	case 0x14:
	case 0x24:
	case 0x34:
		break;
	default:
		//return _rxBuf[23];
		return -1;
	}*/

	_sup->ch1 =   _rxBuf[0]			   | ((_rxBuf[1] & 0x07) << 8);
	_sup->ch2 = ((_rxBuf[1] & 0xf8) >> 3) | ((_rxBuf[2] & 0x3f) << 5);
	_sup->ch3 = ((_rxBuf[2] & 0xc0) >> 6) | ((_rxBuf[3] ) << 2) | ((_rxBuf[4] & 0x01) << 10);
	_sup->ch4 = ((_rxBuf[4] & 0xfe) >> 1) | ((_rxBuf[5] & 0x0f) << 7);
	_sup->ch5 = ((_rxBuf[5] & 0xf0) >> 4) | ((_rxBuf[6] & 0x7f) << 4);
	_sup->ch6 = ((_rxBuf[6] & 0x80) >> 7) | ((_rxBuf[7] ) << 1) | ((_rxBuf[8] & 0x03) << 9);
	_sup->ch7 = ((_rxBuf[8] & 0xfc) >> 2) | ((_rxBuf[9] & 0x1f) << 6);
	_sup->ch8 = ((_rxBuf[9] & 0xe0) >> 5) | ((_rxBuf[10] ) << 3);

	_sup->ch9 =   _rxBuf[11]			   | ((_rxBuf[12] & 0x07) << 8);
	_sup->ch10 = ((_rxBuf[12] & 0xf8) >> 3) | ((_rxBuf[13] & 0x3f) << 5);
	_sup->ch11 = ((_rxBuf[13] & 0xc0) >> 6) | ((_rxBuf[14] ) << 2) | ((_rxBuf[15] & 0x01) << 10);
	_sup->ch12 = ((_rxBuf[15] & 0xfe) >> 1) | ((_rxBuf[16] & 0x0f) << 7);
	_sup->ch13 = ((_rxBuf[16] & 0xf0) >> 4) | ((_rxBuf[17] & 0x7f) << 4);
	_sup->ch14 = ((_rxBuf[17] & 0x80) >> 7) | ((_rxBuf[18] ) << 1) | ((_rxBuf[19] & 0x03) << 9);
	_sup->ch15 = ((_rxBuf[19] & 0xfc) >> 2) | ((_rxBuf[20] & 0x1f) << 6);
	_sup->ch16 = ((_rxBuf[20] & 0xe0) >> 5) | ((_rxBuf[21] ) << 3);

	_sup->failsafe = (_rxBuf[22] & 0x08);
	_sup->frame_lost = (_rxBuf[22] & 0x04);

	return 1;
}


int SbusParser::rx_char(int c) {

	if (!_rxInBand) {
		if (c == 0x0f) {
			_rxInBand = 1;
			_rxIdx = 0;
		} else {
			//missedChars++;
		}
	} else if (_rxInBand) {
		_rxBuf[_rxIdx] = c;
		if (_rxIdx > 22) {
			_rxInBand = 0;
			return _processSbusMessage();
			//missedChars = 0;
			//return retval;
		} else {
			_rxIdx++;
		}
	}
	return 0;  // no new data yet...
}


