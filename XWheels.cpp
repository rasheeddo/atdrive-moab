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

char response[6];
char show_response[6];
int _len = 0;
bool reset_flag = false;

XWheels::XWheels(PinName a, PinName b) {
	_tx_pin = a;
	_rx_pin = b;

     // Sending Command
    Header1 = 0x02;
    Header2 = 0x09;

    // Initialized of 17 bytes
    InitHeader1 = 0x01;     // always constant
    InitHeader2 = 0x11;     // always constant
    ForwardAcc = 0x32;      // 0x00 to 0x64   [0-100]   An acceleration when changing speed value
    ForwardDelay = 0x00;    // 0x00 t0 0x05   [0-5]     A delay before start to go
    BrakeDis = 0x32;        // 0x01 to 0x64   [0-100]   Brake distance, should be as short as possible (rigid brake), DONT'T PUT 0x00
    TurnAcc = 0x32;         // 0x00 to 0x64   [0-100]   Turning acceleration, when two wheels has reverse direction
    TurnDelay = 0x01;       // 0x00 t0 0x05   [0-5]     A delay before start turning
    AccTimeOfStart = 0x00;  // 0x00 to 0x32   [0-50]    increase this will make wheels slower
    SenRocker = 0x83;       // Don't need to change     this is for curving motion, we have our own calculation.
    UnderVolt1 = 0x14;      // 0x12 -> 18.0V, 0x13 -> 19.0V, 0x14 -> 20.0V, 0x15 -> 21.0V, 0x16 -> 22.0V
    UnderVolt2 = 0x05;      // 0x01 to 0x09 -> 0.1V tp 0.9V
    StartSpeed = 0x0A;      // 0x00 to 0x64   [0-100]   starting speed if too high you will hear some cogging sound out from gear, set not too high
    DriveMode = 0x01;       // 0x01 is Sine wave, 0x00 is square wave. Don't need to change, square wave seems not working well...
    PhaseLMotor = 0x03;     // Don't need to change     not sure what is this, so leave it alone 
    PhaseRMotor = 0x04;     // Don't need to change     not sure what is this, so leave it alone
    MotorConfig = 0x07;     // Don't need to change     This is about choosing which wheel will be reverse
    InitCheckSum = InitHeader1 + InitHeader2 + ForwardAcc + ForwardDelay + BrakeDis + TurnAcc + TurnDelay + AccTimeOfStart + SenRocker 
                    + UnderVolt1 + UnderVolt2 + StartSpeed + DriveMode + PhaseLMotor + PhaseRMotor + MotorConfig;
    
}


void XWheels::Start() {
	main_thread.start(callback(this, &XWheels::main_worker));
}

// This Rx_interrupt is made to capture reply values from black ESC
// because sometimes the XWheels stop operate when we left it for such a while
// we need somehow to reset it, I firstly made a software reset

// I observed something that when we leave the robot for a while without moving
// and we come back and push the stick again, it's like the error happend at that time
// and everytime it starts to move, the relay on ESC will click
// so I tried find the ZERO_RPM that the ESC still hold-click but the wheel doesn't turn 
// check on ZERO_RPM, with this value it will prevent the robot to become error

void XWheels::Rx_Interrupt(){
    char c;
    // basically, the reply value is 0x82, 0x06, 0x00, 0x00, 0x0*, 0x**
    // first, second and third bytes always seem not to change
    // 5th byte is battery status, 6th (last) byte is check sum
    // 4th byte is error flag, if it was 4 or 6 the motor won't run anymore

    while (_uart->readable()) {

        c = _uart->getc();
        // 130 is 0x82
        // sometime the last byte comes first which I don't understand
        // to prevent that just make sure c is 0x82 
        if (c == 130){
            _len = 0;
        }
        response[_len] = c;
        _len++;

        if (_len == 6){
            memcpy(show_response, response, 6);
            _len = 0;
        }

        if (response[3] == 4 || response[3] == 6){
            NVIC_SystemReset();
            reset_flag = true;
        }
    }    
}

void XWheels::main_worker() {
	// We want the UART to offline before doing the init sequence,
	// this seems to be how to reset the motor drive

	// previously it was 1000ms but I changed it to a bit longer
    // seem like when the wheels are in ESC error situation, and we resest it again (by NVIC_SystemReset)
    // when it comes back, it will become able to work faster than than small delay
    ThisThread::sleep_for(3000);  //3000

	//u_printf("Trying to start XWheels...\n");

	_uart = new RawSerial(_tx_pin, _rx_pin, 9600);
	waitUntilFourZero();
	ThisThread::sleep_for(219);
	ESCHandShake();

	_rpm_motor_1 = 0.0;
	_rpm_motor_2 = 0.0;

	send_motor_command();

	//u_printf("... did it start?\n");
    _uart->attach(callback(this, &XWheels::Rx_Interrupt));
	while (true) {
		send_motor_command();
        //u_printf("show_response %x %x %x %x %x %x", show_response[0],show_response[1],show_response[2],show_response[3],show_response[4],show_response[5]);
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

void XWheels::ESCHandShake(){

    for(int k=1;k<=20;k++)
    {   
        // This is like initial setup for the ESC
        _uart->putc(InitHeader1);
        _uart->putc(InitHeader2);
        _uart->putc(ForwardAcc);
        _uart->putc(ForwardDelay);
        _uart->putc(BrakeDis);
        _uart->putc(TurnAcc);
        _uart->putc(TurnDelay);
        _uart->putc(AccTimeOfStart);
        _uart->putc(SenRocker);
        _uart->putc(UnderVolt1);
        _uart->putc(UnderVolt2);
        _uart->putc(StartSpeed);
        _uart->putc(DriveMode);
        _uart->putc(PhaseLMotor);
        _uart->putc(PhaseRMotor);
        _uart->putc(MotorConfig);
        _uart->putc(InitCheckSum);

        if (k==1){
            ThisThread::sleep_for(1);
            //wait_us(300);
        }
        else{
            ThisThread::sleep_for(14);
            //wait_us(14000);
            
        }
    }
}



inline float _linear_map(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


unsigned int XWheels::RPM_to_data(float rpm) {
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


void XWheels::send_motor_command(){  
    
    uint16_t motor1_int = RPM_to_data(_rpm_motor_1);
    uint16_t motor2_int = RPM_to_data(_rpm_motor_2);

    uint8_t *Motor1SpeedByte;
    uint8_t *Motor2SpeedByte;

    unsigned char Modehibyte = 0x00;
    unsigned char Modelobyte = 0x00;

    // There is some problem with these two bytes
    // when I set to 0x00 for both like I did before, 
    // the wheels didn't stop after release stick, but it has brake
    // but with 0xB4 and 0x80, the ESC doesn't brake, not sure why
    // I fixied it temporaly by using spring mechanical brake
   

    // Type-casting trick to turn a 16 bit into a pair of 8 bits's
    Motor1SpeedByte = (uint8_t *) &motor1_int;
    Motor2SpeedByte = (uint8_t *) &motor2_int;

    unsigned char Motor1hibyte = Motor1SpeedByte[1];
    unsigned char Motor1lobyte = Motor1SpeedByte[0];

    unsigned char Motor2hibyte = Motor2SpeedByte[1];
    unsigned char Motor2lobyte = Motor2SpeedByte[0];


    // Try to change mode bytes during operation, but not worked well
    // if ((_rpm_motor_1 == 0.0) && (_rpm_motor_2 == 0.0)){
    // 	Modehibyte = 0xB4; //0xB4; //0x00;   
    // 	Modelobyte = 0x00;//0x80; //0x00;
    // } else{
    // 	Modehibyte = 0xB4; //0xB4; //0x00;   
    // 	Modelobyte = 0x80;//0x80; //0x00;
    // }
    

    unsigned char CheckSum = Header1 + Header2 + Motor1hibyte + Motor1lobyte + Motor2hibyte + Motor2lobyte + Modehibyte + Modelobyte;

    _uart->putc(Header1);
    _uart->putc(Header2);
    _uart->putc(Motor1hibyte);
    _uart->putc(Motor1lobyte);
    _uart->putc(Motor2hibyte);
    _uart->putc(Motor2lobyte);
    _uart->putc(Modehibyte);
    _uart->putc(Modelobyte);
    _uart->putc(CheckSum);

    //wait_ms(23);                      // DON'T change this delay, it's from hacking
    ThisThread::sleep_for(23);  //23
  
}

void XWheels::vehicleControl(int UD_ch, int LR_ch, float MotorRPM[2]){   
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
        MotorRPM[0] = 0.0;
        MotorRPM[1] = 0.0;
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

void XWheels::vehicleControlProportionalMixing(int UD_ch, int LR_ch, float MotorRPM[2]){   
    // UD_ch is up-down stick channel, in this case is ch2
    // LR_ch is left-right stick channel, in this case is ch4
    // MotorRPM[0] is a right wheel
    // MotorRPM[1] is a left wheel

	// I followed the method from this video 
	// https://www.youtube.com/watch?v=t1NSeMTVRH8

	// the speed changing is smooth, but just curvy backward is opposite

	if (((UD_ch > MIN_DEADBAND) && (UD_ch < MAX_DEADBAND)) && ((LR_ch > MIN_DEADBAND) && (LR_ch < MAX_DEADBAND))) {
		MotorRPM[0] = 0.0;
		MotorRPM[1] = 0.0;
	} else{

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

    
}


