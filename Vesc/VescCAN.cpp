#include "VescCAN.hpp"

extern void u_printf(const char *fmt, ...);  // Defined in main()

VescCan::VescCan(PinName a, PinName b, UDPSocket *tx_sock){

	_rd_pin = a;
	_td_pin = b;

	_sock = tx_sock;

	_can = new CAN(_rd_pin, _td_pin);  // RD then TD
	//_can->attach(callback(this, &VescCan::Rx_Interrupt));

	_usb_debug = new RawSerial(USBTX,USBRX,115200); 

}

void VescCan::Start() {
	main_thread.start(callback(this, &VescCan::main_worker));
}

// void VescCan::Rx_Interrupt(){

// 	if(_can->read(_rx_msg_buffer)){

// 	}


// }

void VescCan::main_worker(){

	ThisThread::sleep_for(1000);

	_can->frequency(500000);

	while (true){

#ifdef _XWHEELS
		float dutyR = RPM_TO_DUTY(_rpmR);
		float dutyL = RPM_TO_DUTY(_rpmL);
		setDuty(0, dutyR);
		setDuty(1, dutyL);

		_usb_debug->printf("dutyR %f   dutyL %f\n",dutyR, dutyL);
#endif
#ifdef _OFFROAD
		setRPM(0, RPM_TO_ERPM(_rpmR));
		setRPM(1, RPM_TO_ERPM(_rpmL));
#endif
		//_usb_debug->printf("_rpmR %f   _rpmL %f\n",_rpmR, _rpmL);
		readReply();

		CANErrorCheck();

		ThisThread::sleep_for(1);
	}

}


void VescCan::Int16ToByteData(unsigned int Data, unsigned char StoreByte[2]){

	StoreByte[0] = (Data & 0xFF00) >> 8;
	StoreByte[1] = (Data & 0x00FF);

}

void VescCan::Int32ToByteData(int32_t Data, unsigned char StoreByte[4]){

	StoreByte[0] = (Data & 0xFF000000) >> 24;
	StoreByte[1] = (Data & 0x00FF0000) >> 16;
	StoreByte[2] = (Data & 0x0000FF00) >> 8;
	StoreByte[3] = (Data & 0x000000FF);

}

uint16_t VescCan::ByteDataToInt16(unsigned char ByteData[2]){

	uint16_t IntData;

	IntData = (ByteData[0] & 0xFF) | ((ByteData[1] & 0xFF) << 8);

	return IntData;
} 

int32_t VescCan::ByteDataToInt32(unsigned char ByteData[4]){

	int32_t IntData;

	IntData = ByteData[0] << 24 | ByteData[1] << 16 | ByteData[2] << 8 | ByteData[3];

	return IntData;
} 

void VescCan::CANErrorCheck(){
	// If there is an error of tx or rx caused by overflow ( still don't know where there is overflow...)
	// the servo will move for a while then stop forever even we keep sending command
	if (_can->rderror() || _can->tderror()){
		_usb_debug->printf("rderror: %d\n", _can->rderror());
		_usb_debug->printf("tderror: %d\n", _can->tderror());
		//_can->reset();
	}
}

void VescCan::canWrite(unsigned int _ID, char data[8]){
	
	int i;
	// create a CANMessage object
	CANMessage sendMsg = CANMessage(_ID, data, 8, CANData, CANStandard);
	
	// keep sending it until there is nothing to send
	do{
		i = _can->write(sendMsg);
	}
	while(!i);
	//wait_us(100);
}

void VescCan::canWriteEx(unsigned int _ID, char data[8]){
	
	int i;
	// create a CANMessage object
	CANMessage sendMsg = CANMessage(_ID, data, 8, CANData, CANExtended);
	
	// keep sending it until there is nothing to send
	do{
		i = _can->write(sendMsg);
	}
	while(!i);
	//wait_us(100);
}

void VescCan::setRPM(uint8_t vesc_id, float rpm){

	unsigned char SpeedByte[4];
	char CommandSpeedByte[8];
	unsigned int CAN_id;

	Int32ToByteData((int32_t)rpm, SpeedByte);

	CommandSpeedByte[0] = SpeedByte[0];
	CommandSpeedByte[1] = SpeedByte[1];
	CommandSpeedByte[2] = SpeedByte[2];
	CommandSpeedByte[3] = SpeedByte[3];
	CommandSpeedByte[4] = 0;
	CommandSpeedByte[5] = 0;
	CommandSpeedByte[6] = 0;
	CommandSpeedByte[7] = 0;

	CAN_id = 0x300 | vesc_id;

	canWriteEx(CAN_id, CommandSpeedByte);
}

void VescCan::setDuty(uint8_t vesc_id, float duty){

	unsigned char DutyByte[4];
	char CommandDutyByte[8];
	unsigned int CAN_id;

	if(duty > 1.0){
		duty = 1.0;
	} else if(duty < -1.0){
		duty = -1.0;
	}

	Int32ToByteData((int32_t)(duty*100000), DutyByte);

	CommandDutyByte[0] = DutyByte[0];
	CommandDutyByte[1] = DutyByte[1];
	CommandDutyByte[2] = DutyByte[2];
	CommandDutyByte[3] = DutyByte[3];
	CommandDutyByte[4] = 0;
	CommandDutyByte[5] = 0;
	CommandDutyByte[6] = 0;
	CommandDutyByte[7] = 0;

	CAN_id = 0x000 | vesc_id;

	canWriteEx(CAN_id, CommandDutyByte);
}

void VescCan::readReply(){

	//_usb_debug->printf("readReply\n");
	CANMessage replyMsg;
	bool readOK = false;
	bool finishRead = false;
	unsigned char _rpm_byte[4];

	while (_can->read(replyMsg)){
		

		// _usb_debug->printf("ID: %X    %X %X %X %X     %X %X %X %X     ", 
		// 	(replyMsg.id),
		// 	replyMsg.data[0],replyMsg.data[1],replyMsg.data[2],replyMsg.data[3],
		// 	replyMsg.data[4],replyMsg.data[5],replyMsg.data[6],replyMsg.data[7]);

		_rpm_byte[0] = replyMsg.data[0];
		_rpm_byte[1] = replyMsg.data[1];
		_rpm_byte[2] = replyMsg.data[2];
		_rpm_byte[3] = replyMsg.data[3];


		if(replyMsg.id == 0x900){
			_read_rpmR = ERPM_TO_RPM((float)ByteDataToInt32(_rpm_byte));
			report_data._reported_rpmR = _read_rpmR;
		}

		if(replyMsg.id == 0x901){
			_read_rpmL = ERPM_TO_RPM((float)ByteDataToInt32(_rpm_byte));
			report_data._reported_rpmL = _read_rpmL;
		}

		//_usb_debug->printf("rpmR: %f     rpmL: %f\n", _read_rpmR, _read_rpmL);

		readOK = true;
	}

	if (!readOK){
		_read_rpmR = _prev_read_rpmR;
		_read_rpmL = _prev_read_rpmL;

		report_data._reported_rpmR = _prev_report_rpmR;
		report_data._reported_rpmL = _prev_report_rpmL;
	}


	int retval = _sock->sendto(_AUTOPILOT_IP_ADDRESS, UDP_PORT_VESC, 
		(char *) &report_data, sizeof(report_data));

	_prev_read_rpmR = _read_rpmR;
	_prev_read_rpmL = _read_rpmL;

	_prev_report_rpmR = report_data._reported_rpmR;
	_prev_report_rpmL = report_data._reported_rpmL;

}

inline float _linear_map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float VescCan::RPM_TO_ERPM(float rpm){
	return rpm*_ERPM_ratio;
}

float VescCan::ERPM_TO_RPM(float erpm){
	return erpm/_ERPM_ratio;
}

float VescCan::RPM_TO_DUTY(float rpm){
	return _linear_map(rpm, -MAX_RPM, MAX_RPM, -MAX_DUTY, MAX_DUTY);
}

void VescCan::setRPMs(float rpmR, float rpmL){

	_rpmR = rpmR;
	_rpmL = rpmL;

}

void VescCan::vehicleControlProportionalMixing(int UD_ch, int LR_ch, float MotorRPM[2]){   
    // UD_ch is up-down stick channel, in this case is ch2
    // LR_ch is left-right stick channel, in this case is ch4
    // MotorRPM[0] is a right wheel
    // MotorRPM[1] is a left wheel

	// I followed the method from this video 
	// https://www.youtube.com/watch?v=t1NSeMTVRH8

	// the speed changing is smooth, but just curvy backward is opposite

    float y = (float)_linear_map(UD_ch, MIN_STICK, MAX_STICK, -100.0, 100.0);
    float x = (float)_linear_map(LR_ch, MIN_STICK, MAX_STICK, -100.0, 100.0);

    float left = y+x;
    float right = y-x;

    float diff = abs(x) - abs(y);

    float swap;


    if (left < 0.0){
    	left = left - abs(diff);
    } else{
    	left = left + abs(diff);
    }


    if (right < 0.0){
    	right = right - abs(diff);
    } else{
    	right = right + abs(diff);
    }

    // This is in case correct curvy backward, but it doesn't smooth, when suddenly changes
    // if (_pre_Y < 0.0){
    // 	swap = left;
    // 	left = right;
    // 	right = swap;
    // }
    // _pre_Y = y;

    // some deadband
   	if (abs(left) < 2.0){
   		left = 0.0;
   	}
   	if (abs(right) < 2.0){
   		right = 0.0;
   	}

    //left = _linear_map(left, -200.0, 200.0, -100.0, 100.0);
    //right = _linear_map(right, -200.0, 200.0, -100.0, 100.0);

    MotorRPM[0] = _linear_map(right, -200.0, 200.0, -MAX_RPM, MAX_RPM);
	MotorRPM[1] = _linear_map(left, -200.0, 200.0, -MAX_RPM, MAX_RPM);
	
    //_usb_debug->printf("left: %f   right: %f   \n", left, right);
}