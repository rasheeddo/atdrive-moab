/*
 * firmware driver for a particular motor drive
 *
 * Reverse-engineering and original code was done by Rasheed Kittinanthapanya
 *
 *   see here:  https://github.com/rasheeddo/BrushlessDriveWheels
 * for info on the wheels protocol
 */

#include "XWheels.hpp"


extern void u_printf(const char *fmt, ...);  // Defined in main()


XWheels::XWheels(PinName a, PinName b) {
	_tx_pin = a;
	_rx_pin = b;
}


void XWheels::Start() {
	main_thread.start(callback(this, &XWheels::main_worker));
}


void XWheels::main_worker() {
	// We want the UART to offline before doing the init sequence,
	// this seems to be how to reset the motor drive

	ThisThread::sleep_for(1000);

	//u_printf("Trying to start XWheels...\n");

	_uart = new RawSerial(_tx_pin, _rx_pin, 9600);
	waitUntilFourZero();
	ThisThread::sleep_for(219);
	ESCHandShake();

	_rpm_motor_1 = 0;
	_rpm_motor_2 = 0;

	send_motor_command();

	//u_printf("... did it start?\n");

	while (true) {
		send_motor_command();
	}
}


void XWheels::setRPMs(float rpm_motor_1, float rpm_motor_2) {
	_rpm_motor_1 = rpm_motor_1;
	_rpm_motor_2 = rpm_motor_2;
}


void XWheels::waitUntilFourZero() {
	int count_zeros = 0;
	if (_uart->readable() == true) {

		while (true) {
			char c = _uart->getc();
			if (c == 0) {
				count_zeros++;
			} else {
				count_zeros = 0;
			}

			if (count_zeros >= 4) {
				break;
			}
		}
	}
}



void XWheels::ESCHandShake() {

	unsigned char data_packet[17] = {
		0x01, // header1
		0x11, // header2 -- looks like it might be packet size
		0x32, // Forward acceleration
		0x00, // forward delay
		0x0a, // brake distance
		0x32, // turning acceleration,
		0x01, // turn delay
		0x00, // acctimeofstart
		0x83, // senrocker
		0x14, // undervolt1
		0x05, // undervolt2
		0x0a, // start speed
		0x01, // drive mode
		0x03, // phaseLMotor
		0x04, // phaseRMotor
		0x07, // motor config
		0x00  // init checksum
	};

	unsigned char cksum = 0;
	for (int i=0; i<16; ++i) {
		cksum += data_packet[i];
	}
	data_packet[16] = cksum;


	for (int k=1;k<=20;k++) {  // I guess we retry 20 times?
		for (int i=0; i<17; ++i) {
			_uart->putc(data_packet[i]);
		}

		if (k==1) {
			ThisThread::sleep_for(1);

		} else {

			ThisThread::sleep_for(14);
		}
	}
}



inline float _linear_map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


unsigned int RPM_to_data(float rpm) {
	// Convert a real RPM value to the data format expected by the device

	// There seems to be a dead zone between -70 and 70
	// At max RPM (144 RPM), the data should be about 3200

	if (rpm > 0.0 && rpm < 40.0) {

		return (unsigned int)_linear_map(rpm, 0.0, 39.0, 70, 866);

	} else if (rpm < 0.0 && rpm > -40.0) {

		return (unsigned int)_linear_map(rpm, 0.0, -39.0, 32845, 33634); 

	} else if (rpm >= 40.0) {

		return (unsigned int)_linear_map(rpm, 40.0, 144.0, 888, 3200); 

	} else if (rpm <= -40.0) {

		return (unsigned int)_linear_map(rpm, -40.0, -144.0, 33656, 35968); 
	}

	return 0;
}


void XWheels::send_motor_command() {
	unsigned char data_packet[9] = {
		0x02,  // header1
		0x09,  // header2 -- looks like it might be packet size

		0x00,  // motor1 speed MSB
		0x00,  // motor1 speed LSB
		0x00,  // motor2 speed MSB
		0x00,  // motor2 speed LSB

		//0xb4, // mode hibyte
		0x00, // mode hibyte
		0x00, // mode lobyte

		0x00 // checksum
	};

	uint16_t motor1_int = RPM_to_data(_rpm_motor_1);
	uint16_t motor2_int = RPM_to_data(_rpm_motor_2);

	uint8_t *Motor1SpeedByte;
	uint8_t *Motor2SpeedByte;

	// Type-casting trick to turn a 16 bit into a pair of 8 bits's
	Motor1SpeedByte = (uint8_t *) &motor1_int;
	Motor2SpeedByte = (uint8_t *) &motor2_int;

	data_packet[2] = Motor1SpeedByte[1];  // little-endian
	data_packet[3] = Motor1SpeedByte[0];
	data_packet[4] = Motor2SpeedByte[1];  // little-endian
	data_packet[5] = Motor2SpeedByte[0];

	unsigned char cksum = 0;
	for (int i=0; i<8; ++i) {
		cksum += data_packet[i];
	}
	data_packet[8] = cksum;

	for (int i=0; i<9; ++i) {
		_uart->putc(data_packet[i]);
	}

	// This delay was discovered from reverse-engineering:
	ThisThread::sleep_for(23);
}



