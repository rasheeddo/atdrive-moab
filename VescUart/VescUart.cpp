#include "VescUart.h"

extern void u_printf(const char *fmt, ...);  // Defined in main()

VescUart::VescUart(PinName a, PinName b, UDPSocket *tx_sock){

	_tx_pin = a;
	_rx_pin = b;

	_sock = tx_sock;

	_usb_debug = new RawSerial(USBTX,USBRX,115200);

	_uart = new RawSerial(_tx_pin, _rx_pin, 115200);
	_uart->attach(callback(this, &VescUart::_Serial_Rx_Interrupt));

	_ringBuf = new CircularBuffer<uint8_t, _RING_BUFFER_SIZE>;

}

void VescUart::Start() {
	main_thread.start(callback(this, &VescUart::main_worker));
}

void VescUart::_Serial_Rx_Interrupt(){

	uint8_t c;

	while(_uart->readable()) {

		c = _uart->getc();

		// similar with XWheel Rx_interrupt
		// I found that sometime my header byte [0] 0x02 or length byte [1] 0x49 were misordered
		// and that makes parsing data collapse
		// also the second half of buffer from 78 to 155
		/*if ((_count == 0 && c != 0x02) || (_count == 1 && c != 0x49) || (_count == 77 && c != 0x03) ||
			(_count == 78 && c != 0x02) || (_count == 79 && c != 0x49) || (_count == 155 && c != 0x03)){
			// reset counter
			_count = 0;
			// clear all data in array buffer 
			// check this https://stackoverflow.com/questions/13308216/how-do-i-clear-a-c-array
			std::fill_n(_ringBuf, _RING_BUFFER_SIZE, 0);
			// stop read anything and get out now!
			break;

		} else {

			// here is normal situation
			_ringBuf[_count] = c;
			_readyToParse = false; 	// while there is a data readable, stop parse the packets
			_count++;
		}*/


		// _ringBuf[_count] = c;
		// _count++;
		// _readyToParse = false;

		// I tried a lot of stuff, but this CircularBuffer worked the best for me
		// the other I commented it out as you can see, and leave it as the history of my mistake
		if (_ringBuf->full() == false){
			_ringBuf->push(c);
		}
		
		//// NOTE ////
		// there is one FSESC6.6 that Rx-UART has some noises which shown 0xFF on logic analyzer
		// I think that also make the reading mess up sometime
		// if it happend, we just break out from interrupt loop like above if-state

		// if (_count == 156){
		// 	_readyToParse = true;	// start parse the packet when interrupt finished reading
		// }
		

		// if (_count >= _RING_BUFFER_SIZE) {
		// 	_readyToParse = true;
		// 	_count = 0; //_ringBuf->push(c);
			
		// }
	}

}

void VescUart::main_worker(){

	ThisThread::sleep_for(1000);

	while(true){

		
		_timer.start();
		//////////////////////////////////// WRITE SPEED COMMAND //////////////////////////////////
		if (_man_flag){

#ifdef _XWHEELS
			setDuty(RPM_TO_DUTY(_rpmR));
			//ThisThread::sleep_for(1);
			setDuty(RPM_TO_DUTY(_rpmL),1);
			//ThisThread::sleep_for(1);
#endif
#ifdef _OFFROAD
			setRPM(RPM_TO_ERPM(_rpmR));
			setRPM(RPM_TO_ERPM(_rpmL),1);
#endif

		} else{

#ifdef _XWHEELS
			setDuty(RPM_TO_DUTY(_rpmR));
			setDuty(RPM_TO_DUTY(_rpmL),1);
#endif			
#ifdef _OFFROAD			
			setRPM(RPM_TO_ERPM(_rpmR));
			setRPM(RPM_TO_ERPM(_rpmL),1);
#endif
		}

		//////////////////////////////////// READ SPEED FEEDBACK /////////////////////////////////
		//set command to ask for a speed (getVescValues)
		recvUartByInterrupt();
		_timer.stop();

		//////////////////////////////////// REPORT SPEED FEEDBACK //////////////////////////////
		 
		_period = _timer.read();
		//_usb_debug->printf("time read %f seconds\n", _period);
		_timer.reset();

	}

}

void VescUart::recvUartByInterrupt(){

	uint8_t _vesc0Buf[78];
	uint8_t _vesc1Buf[78];
	uint8_t vesc0_payloadReceived[78];
	uint8_t vesc1_payloadReceived[78];

	bool vesc0_unpacked = false;
	bool vesc1_unpacked = false;
	uint8_t data;
	float _read_rpmR = 0.0;
	float _read_rpmL = 0.0;

	int idx = 0;

	bool _vesc0GoodPacket = false;
	bool _vesc1GoodPacket = false;

	bool set_print0 = false;
	bool set_print1 = false;


	/////////////////////////// VESC 0 RECEIVE //////////////////////////////
	askForValues();

	// this while loop will pop the data from FIFO buffer out
	while(_ringBuf->empty() == false){
		
		// pop and store to local buffer
		_ringBuf->pop(data);
		_vesc0Buf[idx] = data;

		idx++;

		// vesc feedback values are only 78bytes
		if (idx >= _VESC_RECV_BYTES){
			set_print0 = true;
			idx = 0;
			break;
		}

		set_print0 = false;

	}
	
	// This is to check that our local buffer _vesc0Buf got a correct packet
	if (_vesc0Buf[0] != 0x02 || _vesc0Buf[1] != 0x49 || _vesc0Buf[77] != 0x03){
		// sometime there is some noise mixed up with our Rx data,
		// and it messed up all the packet sequence, so just reset the ring buffer
		_ringBuf->reset();
		_usb_debug->printf("________________Reset Buffer 0______________\n");
		_vesc0GoodPacket = false;
	} else{
		_vesc0GoodPacket = true;
	}

	// some of delay make the data not collapse and misordered
	ThisThread::sleep_for(8);

	////////////////////////////////// VESC 1 RECEIVE ///////////////////////////////////////
	askForValues(1);

	while(_ringBuf->empty() == false){

		// pop and store to local buffer
		_ringBuf->pop(data);
		_vesc1Buf[idx] = data;

		idx++;

		// vesc feedback values are only 78bytes
		if (idx >= _VESC_RECV_BYTES){
			set_print1 = true;
			idx = 0;
			break;
		}

		set_print1 = false;

	}

	// This is to check that our local buffer _vesc0Buf got a correct packet
	if (_vesc1Buf[0] != 0x02 || _vesc1Buf[1] != 0x49 || _vesc1Buf[77] != 0x03){
		// sometime there is some noise mixed up with our Rx data,
		// and it messed up all the packet sequence, so just reset the ring buffer
		_ringBuf->reset();
		_usb_debug->printf("________________Reset Buffer 1_____________\n");
		_vesc1GoodPacket = false;
	} else{
		_vesc1GoodPacket = true;
	}

	//////////////////////////   PARSE DATA and REPORT   ////////////////////////////////

	if (_vesc0GoodPacket){
		vesc0_unpacked = unpackPayload(_vesc0Buf, 78, vesc0_payloadReceived);
		_read_rpmR = getRPMFormPacket(vesc0_payloadReceived);
		_read_rpmR = ERPM_TO_RPM(_read_rpmR);
		report_data._reported_rpmR = _read_rpmR;
	} else {
		report_data._reported_rpmR = _prev_report_rpmR;
	}

	if (_vesc1GoodPacket){
		vesc1_unpacked = unpackPayload(_vesc1Buf, 78, vesc1_payloadReceived);
		_read_rpmL = getRPMFormPacket(vesc1_payloadReceived);
		_read_rpmL = ERPM_TO_RPM(_read_rpmL);
		report_data._reported_rpmL = _read_rpmL;
	} else{
		report_data._reported_rpmL = _prev_report_rpmL;
	}

	int retval = _sock->sendto(_AUTOPILOT_IP_ADDRESS, UDP_PORT_VESC, 
		(char *) &report_data, sizeof(report_data));
				

	_prev_report_rpmR = report_data._reported_rpmR;
	_prev_report_rpmL = report_data._reported_rpmL;

	//// For debugging
	//// if we have this print state we can lower the delay below to 5ms
	// if (set_print0 && set_print1){
	// 	_usb_debug->printf("0[0]: %X 0[1]: %X 0[77]: %X    1[0]: %X 1[1]: %X 1[77]: %X   _read_rpmR: %f _read_rpmL: %f\n",
	// _vesc0Buf[0], _vesc0Buf[1] , _vesc0Buf[77], _vesc1Buf[0], _vesc1Buf[1], _vesc1Buf[77], _read_rpmR, _read_rpmL);
	// }
	
	// some of delay make the data not collapse and misordered, cannot lower than this!
	ThisThread::sleep_for(8);

}


int VescUart::receiveUartMessage(uint8_t * payloadReceived) {

	// Messages <= 255 starts with "2", 2nd byte is length
	// Messages > 255 starts with "3" 2nd and 3rd byte is length combined with 1st >>8 and then &0xFF

	uint16_t counter = 0;
	uint16_t endMessage = 256;
	bool messageRead = false;
	uint8_t messageReceived[256];
	uint16_t lenPayload = 0;
	
	uint64_t timeout = rtos::Kernel::get_ms_count() + 100; // Defining the timestamp for timeout (100ms before timeout)

	while ( rtos::Kernel::get_ms_count() < timeout && messageRead == false) {

		while (_uart->readable()) {

			messageReceived[counter++] = _uart->getc();

			if (counter == 2) {

				switch (messageReceived[0])
				{
					case 2:
						endMessage = messageReceived[1] + 5; //Payload size + 2 for sice + 3 for SRC and End.
						lenPayload = messageReceived[1];
					break;

					case 3:
						// ToDo: Add Message Handling > 255 (starting with 3)
						if( _usb_debug != NULL ){
							_usb_debug->printf("Message is larger than 256 bytes - not supported\n");
						}
					break;

					default:
						if( _usb_debug != NULL ){
							_usb_debug->printf("Unvalid start bit\n");
						}
					break;
				}
			}

			if (counter >= sizeof(messageReceived)) {
				break;
			}


			// if this is true we will get out from this outer whiel loop
			if (counter == endMessage && messageReceived[endMessage - 1] == 3) {
				messageReceived[endMessage] = 0;
				if (_usb_debug != NULL) {
					_usb_debug->printf("End of message reached!\n");
				}
				messageRead = true;
				break; // Exit if end of message is reached, even if there is still more data in the buffer.
			}
		}
	}

	if(messageRead == false && _usb_debug != NULL ) {
		_usb_debug->printf("Timeout\n");
	}
	
	bool unpacked = false;

	if (messageRead) {
		unpacked = unpackPayload(messageReceived, endMessage, payloadReceived);
	}

	if (unpacked) {
		// Message was read
		return lenPayload; 
	}
	else {
		// No Message Read
		return 0;
	}
}


bool VescUart::unpackPayload(uint8_t * message, int lenMes, uint8_t * payload) {

	uint16_t crcMessage = 0;
	uint16_t crcPayload = 0;

	// Rebuild crc:
	crcMessage = message[lenMes - 3] << 8;
	crcMessage &= 0xFF00;
	crcMessage += message[lenMes - 2];

	if(_usb_debug!=NULL){
		//_usb_debug->printf("SRC received: %d\n", crcMessage);
	}

	// Extract payload:
	memcpy(payload, &message[2], message[1]);

	crcPayload = crc16(payload, message[1]);

	if( _usb_debug != NULL ){
		//_usb_debug->printf("SRC calc: %d\n", crcPayload);
	}
	
	if (crcPayload == crcMessage) {
		if( _usb_debug != NULL ) {
			//_usb_debug->printf("Received: "); 
			//serialPrint(message, lenMes);

			//_usb_debug->printf("Payload : ");
			//serialPrint(payload, message[1] - 1);
		}

		return true;
	}else{
		return false;
	}
}


int VescUart::packSendPayload(uint8_t * payload, int lenPay) {

	uint16_t crcPayload = crc16(payload, lenPay);
	int count = 0;
	uint8_t messageSend[256];

	if (lenPay <= 256)
	{
		messageSend[count++] = 2;
		messageSend[count++] = lenPay;
	}
	else
	{
		messageSend[count++] = 3;
		messageSend[count++] = (uint8_t)(lenPay >> 8);
		messageSend[count++] = (uint8_t)(lenPay & 0xFF);
	}

	memcpy(&messageSend[count], payload, lenPay);

	count += lenPay;
	messageSend[count++] = (uint8_t)(crcPayload >> 8);
	messageSend[count++] = (uint8_t)(crcPayload & 0xFF);
	messageSend[count++] = 3;
	messageSend[count] = '\0';

	if(_usb_debug!=NULL){
		//_usb_debug->printf("UART package send: "); 
		//serialPrint(messageSend, count);
	}

	// Sending package
	//_uart->write(messageSend, count);
	for (int i=0;i<count;i++){
		_uart->putc(messageSend[i]);
	}

	// Returns number of send bytes
	return count;
}


bool VescUart::processReadPacket(uint8_t * message) {

	COMM_PACKET_ID packetId;
	int32_t ind = 0;

	packetId = (COMM_PACKET_ID)message[0];
	message++; // Removes the packetId from the actual message (payload)

	switch (packetId){
		case COMM_GET_VALUES: // Structure defined here: https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164

			ind = 4; // Skip the first 4 bytes 
			data.avgMotorCurrent 	= buffer_get_float32(message, 100.0, &ind);
			data.avgInputCurrent 	= buffer_get_float32(message, 100.0, &ind);
			ind += 8; // Skip the next 8 bytes
			data.dutyCycleNow 		= buffer_get_float16(message, 1000.0, &ind);
			data.rpm 				= buffer_get_int32(message, &ind);
			data.inpVoltage 		= buffer_get_float16(message, 10.0, &ind);
			data.ampHours 			= buffer_get_float32(message, 10000.0, &ind);
			data.ampHoursCharged 	= buffer_get_float32(message, 10000.0, &ind);
			ind += 8; // Skip the next 8 bytes 
			data.tachometer 		= buffer_get_int32(message, &ind);
			data.tachometerAbs 		= buffer_get_int32(message, &ind);
			return true;

		break;

		default:
			return false;
		break;
	}
}

float VescUart::getRPMFormPacket(uint8_t * message) {

	COMM_PACKET_ID packetId;
	int32_t ind = 0;

	packetId = (COMM_PACKET_ID)message[0];
	message++; // Removes the packetId from the actual message (payload)

	float current;
	float inputCurret;
	float dutyCycle;
	float rpm;

	switch (packetId){
		case COMM_GET_VALUES: // Structure defined here: https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164

			ind = 4; // Skip the first 4 bytes 
			current = buffer_get_float32(message, 100.0, &ind);
			inputCurret = buffer_get_float32(message, 100.0, &ind);
			ind += 8; // Skip the next 8 bytes
			dutyCycle = buffer_get_float16(message, 1000.0, &ind);
			rpm = buffer_get_int32(message, &ind);
			// data.inpVoltage 		= buffer_get_float16(message, 10.0, &ind);
			// data.ampHours 			= buffer_get_float32(message, 10000.0, &ind);
			// data.ampHoursCharged 	= buffer_get_float32(message, 10000.0, &ind);
			// ind += 8; // Skip the next 8 bytes 
			// data.tachometer 		= buffer_get_int32(message, &ind);
			// data.tachometerAbs 		= buffer_get_int32(message, &ind);
			return rpm;

		break;

		default:
			return NULL;
		break;
	}
}

bool VescUart::processReadPacket(uint8_t * message, int vescID) {

	COMM_PACKET_ID packetId;
	int32_t ind = 0;

	packetId = (COMM_PACKET_ID)message[0];
	message++; // Removes the packetId from the actual message (payload)

	switch (packetId){
		case COMM_GET_VALUES: // Structure defined here: https://github.com/vedderb/bldc/blob/43c3bbaf91f5052a35b75c2ff17b5fe99fad94d1/commands.c#L164

			ind = 4; // Skip the first 4 bytes 
			data1.avgMotorCurrent 	= buffer_get_float32(message, 100.0, &ind);
			data1.avgInputCurrent 	= buffer_get_float32(message, 100.0, &ind);
			ind += 8; // Skip the next 8 bytes
			data1.dutyCycleNow 		= buffer_get_float16(message, 1000.0, &ind);
			data1.rpm 				= buffer_get_int32(message, &ind);
			data1.inpVoltage 		= buffer_get_float16(message, 10.0, &ind);
			data1.ampHours 			= buffer_get_float32(message, 10000.0, &ind);
			data1.ampHoursCharged 	= buffer_get_float32(message, 10000.0, &ind);
			ind += 8; // Skip the next 8 bytes 
			data1.tachometer 		= buffer_get_int32(message, &ind);
			data1.tachometerAbs 		= buffer_get_int32(message, &ind);
			return true;

		break;

		default:
			return false;
		break;
	}
}


bool VescUart::getVescValues(void) {

	uint8_t command[1] = { COMM_GET_VALUES };
	uint8_t payload[256];

	packSendPayload(command, 1);
	// delay(1); //needed, otherwise data is not read

	int lenPayload = receiveUartMessage(payload);

	if (lenPayload > 55) {
		bool read = processReadPacket(payload); //returns true if sucessful
		return read;
	}
	else
	{
		return false;
	}
}

bool VescUart::getVescValues(int vescID) {

	int32_t index = 0;
	uint8_t command[3];
	command[index++] = { COMM_FORWARD_CAN };
	command[index++] = { vescID };
	command[index++] = { COMM_GET_VALUES };

	uint8_t payload[256];

	packSendPayload(command, 3);
	// delay(1); //needed, otherwise data is not read

	int lenPayload = receiveUartMessage(payload);

	if (lenPayload > 55) {
		bool read = processReadPacket(payload, vescID); //returns true if sucessful
		return read;
	}
	else
	{
		return false;
	}
}

void VescUart::askForValues(){
	uint8_t command[1] = { COMM_GET_VALUES };
	uint8_t payload[256];

	packSendPayload(command, 1);
}

void VescUart::askForValues(int vescID){
	int32_t index = 0;
	uint8_t command[3];
	command[index++] = { COMM_FORWARD_CAN };
	command[index++] = { vescID };
	command[index++] = { COMM_GET_VALUES };

	uint8_t payload[256];

	packSendPayload(command, 3);
}


void VescUart::setCurrent(float current) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_CURRENT;
	buffer_append_int32(payload, (int32_t)(current * 1000), &index);

	packSendPayload(payload, 5);
}

void VescUart::setBrakeCurrent(float brakeCurrent) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_CURRENT_BRAKE;
	buffer_append_int32(payload, (int32_t)(brakeCurrent * 1000), &index);

	packSendPayload(payload, 5);
}

void VescUart::setRPM(float rpm) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_RPM ;
	buffer_append_int32(payload, (int32_t)(rpm), &index);

	packSendPayload(payload, 5);
}

void VescUart::setRPM(float rpm, int vescID) {
	int32_t index = 0;
	uint8_t payload[7];

	payload[index++] = COMM_FORWARD_CAN ;
	payload[index++] = vescID;
	payload[index++] = COMM_SET_RPM;
	buffer_append_int32(payload, (int32_t)(rpm), &index);

	packSendPayload(payload, 7);
}

void VescUart::setDuty(float duty) {
	int32_t index = 0;
	uint8_t payload[5];

	payload[index++] = COMM_SET_DUTY;
	buffer_append_int32(payload, (int32_t)(duty * 100000), &index);

	packSendPayload(payload, 5);
}

void VescUart::setDuty(float duty, int vescID) {
	int32_t index = 0;
	uint8_t payload[7];

	payload[index++] = COMM_FORWARD_CAN ;
	payload[index++] = vescID;
	payload[index++] = COMM_SET_DUTY;
	buffer_append_int32(payload, (int32_t)(duty * 100000), &index);

	packSendPayload(payload, 7);
}



void VescUart::serialPrint(uint8_t * data, int len) {
	if(_usb_debug != NULL){
		for (int i = 0; i <= len; i++)
		{
			_usb_debug->printf("%x ", data[i]);
		}

		_usb_debug->printf("\n");
	}
}



void VescUart::printVescValues() {
	if(_usb_debug != NULL){
		_usb_debug->printf("avgMotorCurrent: %f\n", data.avgMotorCurrent);
		_usb_debug->printf("avgInputCurrent: %f\n", data.avgInputCurrent);
		_usb_debug->printf("dutyCycleNow: %f\n", data.dutyCycleNow);
		_usb_debug->printf("rpm: %f\n", data.rpm);
		_usb_debug->printf("inputVoltage: %f\n", data.inpVoltage);
		_usb_debug->printf("ampHours: %f\n", data.ampHours);
		_usb_debug->printf("ampHoursCharges: %f\n", data.ampHoursCharged);
		_usb_debug->printf("tachometer: %f\n", data.tachometer);
		_usb_debug->printf("tachometerAbs: %f\n", data.tachometerAbs);
	}
}



inline float _linear_map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float VescUart::RPM_TO_ERPM(float rpm){
	return rpm*_ERPM_ratio;
}

float VescUart::ERPM_TO_RPM(float erpm){
	return erpm/_ERPM_ratio;
}

float VescUart::RPM_TO_DUTY(float rpm){
	return _linear_map(rpm, -MAX_RPM, MAX_RPM, -MAX_DUTY, MAX_DUTY);
}

void VescUart::setRPMs(float rpmR, float rpmL){

	_rpmR = rpmR;
	_rpmL = rpmL;

}


void VescUart::setRPMs(float rpmR, float rpmL, bool man_flag){

	_rpmR = rpmR;
	_rpmL = rpmL;
	_man_flag = man_flag;
}


void VescUart::vehicleControl(int UD_ch, int LR_ch, float MotorRPM[2]){   
    // UD_ch is up-down stick channel, in this case is ch2
    // LR_ch is left-right stick channel, in this case is ch4
    // MotorRPM[0] is a right wheel
    // MotorRPM[1] is a left wheel

    float MIN_SCALER = 1000.0;
    float MAX_SCALER = 2000.0;
    

    /////////////////////////////////////////////////////// STRAIGHT DRIVE ////////////////////////////////////////////////////////////////
    // In case the stick near mid for both ch2 and ch4
    if (LR_ch <= MAX_DEADBAND && LR_ch >= MIN_DEADBAND && UD_ch <= MAX_DEADBAND && UD_ch >= MIN_DEADBAND)
    {
        MotorRPM[0] = ZERO_RPM;
        MotorRPM[1] = ZERO_RPM;
    }

    // user push ch2 up or down, UGV drive forward or backward, two wheels same speed and direction
    else if(LR_ch <= MAX_DEADBAND && LR_ch >= MIN_DEADBAND && (UD_ch > MAX_DEADBAND || UD_ch < MIN_DEADBAND))
    {
        MotorRPM[0] = (float)_linear_map(UD_ch, MIN_STICK, MAX_STICK, -MAX_RPM, MAX_RPM);
        MotorRPM[1] = MotorRPM[0];

    }
    /////////////////////////////////////////////////////////// TURNS /////////////////////////////////////////////////////////////////////
    // user push ch4 left or right, UGV turns left or right, two wheels same speed but reverse direction
    else if(UD_ch <= MAX_DEADBAND && UD_ch >= MIN_DEADBAND && (LR_ch >= MAX_DEADBAND || LR_ch <= MIN_DEADBAND))
    {
        MotorRPM[1] = (float)_linear_map(LR_ch, MIN_STICK, MAX_STICK, -MAX_RPM, MAX_RPM);
        MotorRPM[0] = -MotorRPM[1];
    }
    /////////////////////////////////////////////////////////// CURVES /////////////////////////////////////////////////////////////////////
    // user push both ch2 and ch4 diagonally (first quadrant), UGV curves to the right forward, one wheels is half speed of the another one
    else if(UD_ch > MAX_DEADBAND && LR_ch > MAX_DEADBAND)
    {
        MotorRPM[1] = (float)_linear_map(UD_ch, MAX_DEADBAND+1, MAX_STICK, ZERO_RPM, MAX_RPM);
        float SCALE = (float)_linear_map(LR_ch, MAX_DEADBAND+1, MAX_STICK, MIN_SCALER, MAX_SCALER);
        MotorRPM[0] = MotorRPM[1]*MIN_SCALER/SCALE;
        //printf("SCALE %f\n",SCALE);
    } 

     // user push both ch2 and ch4 diagonally (second quadrant), UGV curves to the left forward, one wheels is half speed of the another one
    else if(UD_ch > MAX_DEADBAND && LR_ch < MIN_DEADBAND)
    {
        MotorRPM[0] = (float)_linear_map(UD_ch, MAX_DEADBAND+1, MAX_STICK, ZERO_RPM, MAX_RPM);
        float SCALE = (float)_linear_map(LR_ch, MIN_DEADBAND-1, MIN_STICK, MIN_SCALER, MAX_SCALER);
        MotorRPM[1] = MotorRPM[0]*MIN_SCALER/SCALE;
        //printf("SCALE %f\n",SCALE);
    }   

    // user push both ch2 and ch4 diagonally (third quadrant), UGV curves to the left backward, one wheels is half speed of the another one
    else if(UD_ch < MIN_DEADBAND && LR_ch < MIN_DEADBAND)
    {
        MotorRPM[0] = (float)_linear_map(UD_ch, MIN_DEADBAND-1, MIN_STICK, ZERO_RPM, -MAX_RPM);
        float SCALE = (float)_linear_map(LR_ch, MIN_DEADBAND-1, MIN_STICK, MIN_SCALER, MAX_SCALER);
        MotorRPM[1] = MotorRPM[0]*MIN_SCALER/SCALE;
        //printf("SCALE %f\n",SCALE);
    }

     // user push both ch2 and ch4 diagonally (fourth quadrant), UGV curves to the right backward, one wheels is half speed of the another one
    else if(UD_ch < MIN_DEADBAND && LR_ch > MAX_DEADBAND)
    {
        MotorRPM[1] = (float)_linear_map(UD_ch, MIN_DEADBAND-1, MIN_STICK, ZERO_RPM, -MAX_RPM);
        float SCALE = (float)_linear_map(LR_ch, MAX_DEADBAND+1, MAX_STICK, MIN_SCALER, MAX_SCALER);
        MotorRPM[0] = MotorRPM[1]*MIN_SCALER/SCALE;
        //printf("SCALE %f\n",SCALE);
    }  
   
}