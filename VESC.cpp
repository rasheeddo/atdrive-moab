#include "VESC.hpp"

extern void u_printf(const char *fmt, ...);  // Defined in main()

VescUart vesc0(PD_1, PD_0);
VescUart vesc1(PD_5, PD_6);

VESC::VESC(UDPSocket *tx_sock){

	//_usb_debug = new RawSerial(USBTX,USBRX,115200);

	_sock = tx_sock;

}

void VESC::Start() {
	main_thread.start(callback(this, &VESC::main_worker));
}


void VESC::main_worker(){

	ThisThread::sleep_for(1000);

	float read_rpm0 = 0.0;
	float read_rpm1 = 0.0;
	float in_voltage0 = 0.0;
	float in_voltage1 = 0.0;
	float in_current0 = 0.0;
	float in_current1 = 0.0;
 

	while(true){

		vesc0.setRPM(RPM_TO_ERPM(_rpmR));
		vesc1.setRPM(RPM_TO_ERPM(_rpmL));
		
		if (vesc0.getVescValues()){
			read_rpm0 = ERPM_TO_RPM(vesc0.data.rpm);
			in_voltage0 = vesc0.data.inpVoltage;
			in_current0 = vesc0.data.avgInputCurrent;
		}
		
		if (vesc1.getVescValues()){
			read_rpm1 = ERPM_TO_RPM(vesc1.data.rpm);
			in_voltage1 = vesc1.data.inpVoltage;
			in_current1 = vesc1.data.avgInputCurrent;

		}

		vescs_reported_data._inpVoltage = (in_voltage0 + in_voltage1)/2.0;
		vescs_reported_data._avgInputCurrent = (in_current0 + in_current1)/2.0;
		vescs_reported_data._rpmR = read_rpm0;
		vescs_reported_data._rpmL = read_rpm1;

		int retval = _sock->sendto(_AUTOPILOT_IP_ADDRESS, UDP_PORT_VESC, 
			(char *) &vescs_reported_data, sizeof(vescs_reported_data));
	

		//_usb_debug->printf("read_rpm0: %f   read_rpm1: %f\n", read_rpm0, read_rpm1);

		//ThisThread::sleep_for(10);
		

	}


}

inline float _linear_map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float VESC::RPM_TO_ERPM(float rpm){
	return rpm*_ERPM_ratio;
}

float VESC::ERPM_TO_RPM(float erpm){
	return erpm/_ERPM_ratio;
}


void VESC::setRPMs(float rpmR, float rpmL){

	_rpmR = rpmR;
	_rpmL = rpmL;

}

void VESC::vehicleControl(int UD_ch, int LR_ch, float MotorRPM[2]){   
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

