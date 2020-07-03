#include "VescUart.h"

extern void u_printf(const char *fmt, ...);  // Defined in main()

VescUart::VescUart(PinName a, PinName b, UDPSocket *tx_sock){

	_tx_pin = a;
	_rx_pin = b;

	_sock = tx_sock;

	_usb_debug = new RawSerial(USBTX,USBRX,115200);

	_uart = new RawSerial(_tx_pin, _rx_pin, 115200);
	_uart->attach(callback(this, &VescUart::_Serial_Rx_Interrupt));

	// _ringBuf = new CircularBuffer<char, _RING_BUFFER_SIZE>;

}

void VescUart::Start() {
	main_thread.start(callback(this, &VescUart::main_worker));
}

void VescUart::_Serial_Rx_Interrupt(){

	//char c;

	while(_uart->readable()) {

		_ringBuf[_count] = _uart->getc(); //c = _uart->getc();
		_count++;

		if (_count >= _RING_BUFFER_SIZE) {
			_count = 0; //_ringBuf->push(c);
			_readyToAsk = true;
		}
		
	}

}

void VescUart::main_worker(){

	ThisThread::sleep_for(1000);

	while(true){

		//_timer.start();

		//////////////////////////////////// WRITE SPEED COMMAND //////////////////////////////////
		if (_man_flag){

#ifdef _XWHEELS
			setDuty(RPM_TO_DUTY(_rpmR));
			setDuty(RPM_TO_DUTY(_rpmL),1);
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

		// set command to ask for a speed (getVescValues)
		recvUartWorker();
		_usb_debug->printf("rpm0: %f   rpm1:%f \n", data.rpm, data1.rpm);

		//////////////////////////////////// REPORT SPEED FEEDBACK //////////////////////////////
		int retval = _sock->sendto(_AUTOPILOT_IP_ADDRESS, UDP_PORT_VESC, 
			(char *) &report_data, sizeof(report_data));

		// _timer.stop();
		// _period = _timer.read();
		// u_printf("_period in VESC %f seconds", _period);
		// _timer.reset();
	}

}

void VescUart::recvUartWorker(){

	uint8_t vesc0_payloadReceived[78];
	uint8_t vesc1_payloadReceived[78];

	if (_readyToAsk){
		askForValues();
		//ThisThread::sleep_for(0.5);
		askForValues(1);
		//ThisThread::sleep_for(0.5);
		_readyToAsk = false;
	} else{

		// what I want to do here is just copy first half of _ringBuf as ves0Buf
		// and second half as vesc1Buf, there might be a std function to do this I think
		for (int i=0; i<78; i++){
			_vesc0Buf[i] = _ringBuf[i];
		}
		for (int i=0; i<78; i++){
			_vesc1Buf[i] = _ringBuf[i+78];
		}


		// unpack received data
		bool vesc0_unpacked = unpackPayload(_vesc0Buf, 78, vesc0_payloadReceived);
		bool vesc1_unpacked = unpackPayload(_vesc1Buf, 78, vesc1_payloadReceived);

		// parse byte data to struct as user can use
		bool proFlag0 = processReadPacket(vesc0_payloadReceived);
		bool proFlag1 = processReadPacket(vesc1_payloadReceived, 1);

		report_data._reported_rpmR = ERPM_TO_RPM(data.rpm);
		report_data._reported_rpmL = ERPM_TO_RPM(data1.rpm);

	}
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
        MotorRPM[1] = (float)_linear_map(LR_ch, MIN_STICK, MAX_STICK, -MAX_RPM/2, MAX_RPM/2);
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