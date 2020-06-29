#include "VescUart.h"

VescUart::VescUart(PinName a, PinName b){

	_tx_pin = a;
	_rx_pin = b;

	//_usb_debug = new RawSerial(USBTX,USBRX,115200);

	_uart = new RawSerial(_tx_pin, _rx_pin, 115200);
	_uart->attach(callback(this, &VescUart::_Serial_Rx_Interrupt));

}

void VescUart::_Serial_Rx_Interrupt(){

	while(_uart->readable()) {
		
		

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
		_usb_debug->printf("SRC received: %d\n", crcMessage);
	}

	// Extract payload:
	memcpy(payload, &message[2], message[1]);

	crcPayload = crc16(payload, message[1]);

	if( _usb_debug != NULL ){
		_usb_debug->printf("SRC calc: %d\n", crcPayload);
	}
	
	if (crcPayload == crcMessage) {
		if( _usb_debug != NULL ) {
			_usb_debug->printf("Received: "); 
			serialPrint(message, lenMes);

			_usb_debug->printf("Payload : ");
			serialPrint(payload, message[1] - 1);
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
		_usb_debug->printf("UART package send: "); 
		serialPrint(messageSend, count);
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
