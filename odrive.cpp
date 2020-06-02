#include "odrive.hpp"

extern void u_printf(const char *fmt, ...);  // Defined in main()

/////////////////////////////////// CLASS CONSTRUCTOR ///////////////////////////////////
ODrive::ODrive(PinName a, PinName b, UDPSocket *tx_sock)
{
    _tx_pin = a;
    _rx_pin = b;  

    memset(&_odriveData, 0, sizeof(_odriveData));
   _sock = tx_sock;
}

std::string ODrive::readString() {
    Timer timer;
    std::string str = "";
    static const unsigned long timeout = 1000;
    timer.reset();
    timer.start();
    for (;;) {
        while (!_uart->readable ()) {
            if (timer.read_ms() >= timeout) {
                return str;
            }
        }
        char c = _uart->getc ();
        if (c == '\n')
            break;
        str += c;
    }
    timer.stop();
    //buffered_pc.printf(str.c_str());
    return str;
}
/////////////////////////////////// SYSTEM & CONFIG ///////////////////////////////////
void ODrive::Start(){
    main_thread.start(callback(this, &ODrive::main_worker));
}

void ODrive::main_worker(){

    _uart = new RawSerial(_tx_pin, _rx_pin, 115200);

    ThisThread::sleep_for(100);
    /*------------------Initialization Process----------------*/
    bool ready = Init();

    /*---------------------odrive thread----------------------*/
    // Initially save start angle position
    ThisThread::sleep_for(100); // this delay is necessary, otherwise _startDeg[0] cannot get the correct value
    _startDeg[0] = GetDEG(0);
    _startDeg[1] = GetDEG(1);

    while(ready){

        if (!_mode_trig){
            if (_mode == 2){

               // use the new _rpm values to drive wheels
               DriveWheelsRPMs(_rpmR, _rpmL);
               //u_printf("_rpmR: %f  _rpmL: %f \n", _rpmR, _rpmL);
               //printf("_rpmR: %f  _rpmL: %f \n", _rpmR, _rpmL);
               // get the rpm value from odrive board
               // then reported _cur_rpm to outside thread
                _cur_rpmR = GetRPM(0);
                _cur_rpmL = GetRPM(1); 
            }
            else if (_mode == 3){

                // use the new _deg values to drive wheels
                DriveWheelsAngle(_degR, _degL);
                // get the deg value from odrive board
                // then reported _cur_deg to outside thread
                _cur_degR = GetDEG(0);
                _cur_degL = GetDEG(1);
            }
            else{
                // if everything is correct, it's rarely comes here 
                u_printf("something might goes wrong...\n");
            }  
            ////////////////////// ODRIVE REPORTED ///////////////////////
            //odrive.get_reported_RPMs(reported_RPMs);
            //odrive.get_reported_DEGs(reported_DEGs);
            _odriveData._mode = _mode;

            if (THOTsign > 0.0){
                _odriveData._rpmR = _cur_rpmR*THOTsign;
                _odriveData._rpmL = -_cur_rpmL*THOTsign;
            }
            else{
                _odriveData._rpmR = -_cur_rpmL*THOTsign;
                _odriveData._rpmL = _cur_rpmR*THOTsign;
            }

            _odriveData._degR = _cur_degR;
            _odriveData._degL = _cur_degL;
            _odriveData._initDegR = _startDeg[0];
            _odriveData._initDegL = _startDeg[1];

            int retval = _sock->sendto(_AUTOPILOT_IP_ADDRESS, UDP_PORT_ODRIVE, 
                (char *) &_odriveData, sizeof(_odriveData));

        }
        else{
            if (_mode == 2){
                // right now is mode 2, but want to change to mode 3
                u_printf("Change to POSITION CONTROL\n");
                switchController(3);
            }
            else if (_mode == 3) {
                // right now is mode 3, but want to change to mode 2
                u_printf("Change to VELOCITY CONTROL\n");
                switchController(2);
            }
        }
    }
}

bool ODrive::Init(){

    // strange behavior... need to read this two times to get the correct mode 
    // when changed mode from one to another one 
    _Axis0_mode = readControlMode(0);
    _Axis1_mode = readControlMode(1);
    _Axis0_mode = readControlMode(0);
    _Axis1_mode = readControlMode(1);

    if((_Axis0_mode != _Axis1_mode)){
        // for example when axis0 is mode 0 and axis1 is mode 2, this is error
        // we need to reset it again
        u_printf("ERROR: control mode are not same for both wheels!\n");
        u_printf("Axis0 mode %d \n", _Axis0_mode);
        u_printf("Axis1 mode %d \n", _Axis1_mode);
        u_printf("----------------reset board----------------\n");
        switchController(CTRL_MODE_VELOCITY_CONTROL);

        return false;
    } 
    else{ 
        // assume both axes are same mode
        _mode = _Axis0_mode;
        u_printf("Axis0 mode %d \n", _Axis0_mode);
        u_printf("Axis1 mode %d \n", _Axis1_mode);

        if (_mode == CTRL_MODE_VELOCITY_CONTROL || _mode == CTRL_MODE_POSITION_CONTROL) {

            runState(0, AXIS_STATE_CLOSED_LOOP_CONTROL);
            runState(1, AXIS_STATE_CLOSED_LOOP_CONTROL);
            SetVelRamp(0,true);
            SetVelRamp(1,true);
            u_printf("Ramp velocity enable\n");

            if((readError(0) != 0) || (readError(1) != 0)){
                u_printf("ERROR: There is some error on odrive board!\n");
                u_printf("Check with odrivetools\n");
                return false;
            }
            else{
                // There is no error, everything is perfect, ready to start while loop
                u_printf("ODrive initialization complete!\n");
                return true;
            }

        }
        else {
            // both axes are in some strange mode like 0, 1, or 4
            u_printf("ERROR: falling in strange mode\n");
            u_printf("----------------reset board----------------\n");
            switchController(CTRL_MODE_VELOCITY_CONTROL);
            return false;
        }        
    }     
}

void ODrive::switchController(int mode){

    // I found this 100ms seems to have less problem
    runState(0, AXIS_STATE_IDLE);
    runState(1, AXIS_STATE_IDLE);
    ThisThread::sleep_for(100);
    changeMode(0, mode);
    changeMode(1, mode);
    saveConfig();
    ThisThread::sleep_for(100);
    rebootODrive();
    ThisThread::sleep_for(100);
    // need to reset mbed //
    NVIC_SystemReset();
}

void ODrive::saveConfig(){
    std::stringstream ss;
    ss << "w " << "save_configuration()" << '\n';
    _uart->puts(ss.str().c_str());
}

void ODrive::rebootODrive(){
    std::stringstream ss;
    ss << "w " << "reboot()" << '\n';
    _uart->puts(ss.str().c_str()); 
}

void ODrive::changeMode(int motor_number, int control_mode){
    std::stringstream ss;
    ss << "w axis" << motor_number << ".controller" << ".config" << ".control_mode " << control_mode << '\n';   
    _uart->puts(ss.str().c_str()); 
}

int ODrive::readControlMode(int motor_number){
    std::stringstream ss;
    ss << "r axis" << motor_number << ".controller" << ".config" << ".control_mode\n";
    _uart->puts(ss.str().c_str());
    return readInt();
}

void ODrive::runState(int motor_number, int requested_state) {
    std::stringstream ss;
    ss   << "w axis" << motor_number << ".requested_state " << requested_state << '\n';
    _uart->puts(ss.str().c_str());
}

int ODrive::readError(int motor_number){
    std::stringstream ss;
    ss << "r axis" << motor_number << ".error\n";
    _uart->puts(ss.str().c_str());
    return readInt();
}

/////////////////////////////////// MATHEMATICS and NUMBERS ///////////////////////////////////
float ODrive::readFloat() {
    return atof(readString().c_str());
}
 
int32_t ODrive::readInt() {
    return atoi(readString().c_str());
}

inline float map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

inline float ODrive::cpsTOrpm(float cps){
    return (cps/cpr)*60.0;
}

inline float ODrive::rpmTOcps(float rpm){
    return (rpm/60.0)*cpr;
}

inline float ODrive::degTOcount(float deg){
    return (cpr/360.0)*(deg);
}

inline float ODrive::countTOdeg(float count){
    return (360.0/cpr)*(count);
}

/////////////////////////////////// READ, WRITE VELOCITY ///////////////////////////////////
void ODrive::SetVelRamp(int motor_number, bool flag){
    std::stringstream ss;
    ss   << "w axis" << motor_number << ".controller" << ".vel_ramp_enable " << flag << '\n';   
    _uart->puts(ss.str().c_str());
    //_uart->printf("w axis%d.controller.vel_ramp_enable %d\n", motor_number, flag);    
}

void ODrive::SetRPM_traj(int motor_number, float RPM){
    float CPS;
    CPS = rpmTOcps(RPM);
    std::stringstream ss;
    ss   << "w axis" << motor_number << ".controller" << ".vel_ramp_target " << CPS << '\n';
    _uart->puts(ss.str().c_str());
    //_uart->printf("w axis%d.controller.vel_ramp_target %f\n", motor_number, CPS); 
}
/*
void ODrive::SetRPM(int motor_number, float RPM){
    float CPS;
    CPS = rpmTOcps(RPM);
    SetVelocity(motor_number,CPS);
}
*/

float ODrive::GetVelocity(int motor_number){
    std::stringstream ss;
    ss << "r axis" << motor_number << ".encoder.vel_estimate\n";
    _uart->puts(ss.str().c_str());
    //_uart->printf("r axis%d.encoder.vel_estimate\n", motor_number); 
    return readFloat();
}

float ODrive::GetRPM(int motor_number){
    float CPS;
    float RPM;
    CPS = GetVelocity(motor_number);
    RPM = cpsTOrpm(CPS);
    ThisThread::sleep_for(1);
    return RPM;
}

void ODrive::DriveWheelsRPMs(float rpmR, float rpmL){
    SetRPM_traj(0,rpmR);
    ThisThread::sleep_for(2); // need this delay to make the frame not break
    SetRPM_traj(1,rpmL);
    ThisThread::sleep_for(2); // cannot decrease lower than this 1ms
}

/////////////////////////////////// READ, WRITE POSITION ///////////////////////////////////

void ODrive::MoveToPos(int motor_number, float deg){
    float _count;
    _count = degTOcount(deg);
    std::stringstream ss;
    ss << "t " << motor_number  << " " << _count << "\n";
    _uart->puts(ss.str().c_str());
}

float ODrive::GetDEG(int motor_number){
    float _CPR;
    _CPR = GetPosition(motor_number);
    ThisThread::sleep_for(4);
    return countTOdeg(_CPR);
}
 
float ODrive::GetPosition(int motor_number){
    std::stringstream ss;
    ss << "r axis" << motor_number << ".encoder.pos_estimate\n";
    _uart->puts(ss.str().c_str());
    //_uart->printf("r axis%d.encoder.pos_estimate\n", motor_number);     
    return readFloat();
}

void ODrive::DriveWheelsAngle(float degR, float degL){
    MoveToPos(0,degR);
    ThisThread::sleep_for(1);  // need this delay to make the frame not break
    MoveToPos(1,degL);
    ThisThread::sleep_for(1);
}

/////////////////////////////////// OUTSIDE THREAD USED ///////////////////////////////////

// Outside thread call this to "get" current mode
int ODrive::get_reported_mode(){
    return _mode;
}
// Outside thread give a trigger to odrive thread
void ODrive::mode_trigger(bool trig){
    _mode_trig = trig;
}

// Outside thread call this to "set" new RPM values to odrive thread
void ODrive::set_RPMs(float rpmR, float rpmL){
    _rpmR = rpmR;
    _rpmL = rpmL;
}

// Outside thread call this to "set" new DEG values to odrive thread
void ODrive::set_DEGs(float degR, float degL){
    _degR = degR;
    _degL = degL;
}

// Outside thread call this to "get" RPM values
void ODrive::get_reported_RPMs(float reportedRPMs[2]){
    reportedRPMs[0] = _cur_rpmR;
    reportedRPMs[1] = _cur_rpmL;
}

// Outside thread call this to "get" DEG values
void ODrive::get_reported_DEGs(float reportedDEGs[2]){
    reportedDEGs[0] = _cur_degR;
    reportedDEGs[1] = _cur_degL;
}

// Outside thread call this to "get" initial DEG values
void ODrive::get_started_DEGs(float startedDEGs[2]){
    startedDEGs[0] = _startDeg[0];
    startedDEGs[1] = _startDeg[1];
}

void ODrive::vehicle_control(int UD_ch, int LR_ch, float MotorRPM[2])
{   
    // UD_ch is up-down stick channel, in this case is ch2
    // LR_ch is left-right stick channel, in this case is ch4
    // MotorRPM[0] is a right wheel
    // MotorRPM[1] is a left wheel

    float MIN_SCALER = 1000.0;
    float MAX_SCALER = 2000.0;

    float leftWheel;
    float rightWheel;
    

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
        MotorRPM[0] = THOTsign*(float)map(UD_ch, MIN_STICK, MAX_STICK, -MAX_RPM, MAX_RPM);
        MotorRPM[1] = -MotorRPM[0];

    }
    /////////////////////////////////////////////////////////// TURNS /////////////////////////////////////////////////////////////////////
    // user push ch4 left or right, UGV turns left or right, two wheels same speed but reverse direction
    else if(UD_ch <= MAX_DEADBAND && UD_ch >= MIN_DEADBAND && (LR_ch >= MAX_DEADBAND || LR_ch <= MIN_DEADBAND))
    {
        MotorRPM[1] = SKIDsign*(float)map(LR_ch, MIN_STICK, MAX_STICK, -MAX_RPM/2, MAX_RPM/2);
        MotorRPM[0] = MotorRPM[1];
    }
    /////////////////////////////////////////////////////////// CURVES /////////////////////////////////////////////////////////////////////
    // user push both ch2 and ch4 diagonally (first quadrant), UGV curves to the right forward, one wheels is half speed of the another one
    else if(UD_ch > MAX_DEADBAND && LR_ch > MAX_DEADBAND)
    {
        if (THOTsign > 0.0){
            leftWheel = -THOTsign*(float)map(UD_ch, MAX_DEADBAND+1, MAX_STICK, ZERO_RPM, MAX_RPM);
            float SCALE = (float)map(LR_ch, MAX_DEADBAND+1, MAX_STICK, MIN_SCALER, MAX_SCALER);
            rightWheel = -leftWheel*MIN_SCALER/SCALE;
        }
        else{
            rightWheel = THOTsign*(float)map(UD_ch, MAX_DEADBAND+1, MAX_STICK, ZERO_RPM, MAX_RPM);
            float SCALE = (float)map(LR_ch, MAX_DEADBAND+1, MAX_STICK, MIN_SCALER, MAX_SCALER);
            leftWheel = -rightWheel*MIN_SCALER/SCALE;
        }

        MotorRPM[0] = rightWheel;
        MotorRPM[1] = leftWheel;

        //MotorRPM[1] = -CURVsign*(float)map(UD_ch, MAX_DEADBAND+1, MAX_STICK, ZERO_RPM, MAX_RPM);
        //float SCALE = (float)map(LR_ch, MAX_DEADBAND+1, MAX_STICK, MIN_SCALER, MAX_SCALER);
        //MotorRPM[0] = -MotorRPM[1]*MIN_SCALER/SCALE;
        //printf("SCALE %f\n",SCALE);
    } 

     // user push both ch2 and ch4 diagonally (second quadrant), UGV curves to the left forward, one wheels is half speed of the another one
    else if(UD_ch > MAX_DEADBAND && LR_ch < MIN_DEADBAND)
    {
        if (THOTsign > 0.0){
            rightWheel = THOTsign*(float)map(UD_ch, MAX_DEADBAND+1, MAX_STICK, ZERO_RPM, MAX_RPM);
            float SCALE = (float)map(LR_ch, MIN_DEADBAND-1, MIN_STICK, MIN_SCALER, MAX_SCALER);
            leftWheel = -rightWheel*MIN_SCALER/SCALE;
        }
        else{
            leftWheel = -THOTsign*(float)map(UD_ch, MAX_DEADBAND+1, MAX_STICK, ZERO_RPM, MAX_RPM);
            float SCALE = (float)map(LR_ch, MIN_DEADBAND-1, MIN_STICK, MIN_SCALER, MAX_SCALER);
            rightWheel = -leftWheel*MIN_SCALER/SCALE;
        }

        MotorRPM[0] = rightWheel;
        MotorRPM[1] = leftWheel;

        //MotorRPM[0] = CURVsign*(float)map(UD_ch, MAX_DEADBAND+1, MAX_STICK, ZERO_RPM, MAX_RPM);
        //float SCALE = (float)map(LR_ch, MIN_DEADBAND-1, MIN_STICK, MIN_SCALER, MAX_SCALER);
        //MotorRPM[1] = -MotorRPM[0]*MIN_SCALER/SCALE;
        //printf("SCALE %f\n",SCALE);
    }   

    // user push both ch2 and ch4 diagonally (third quadrant), UGV curves to the left backward, one wheels is half speed of the another one
    else if(UD_ch < MIN_DEADBAND && LR_ch < MIN_DEADBAND)
    {
        if (THOTsign > 0.0){
            rightWheel = THOTsign*(float)map(UD_ch, MIN_DEADBAND-1, MIN_STICK, ZERO_RPM, -MAX_RPM);
            float SCALE = (float)map(LR_ch, MIN_DEADBAND-1, MIN_STICK, MIN_SCALER, MAX_SCALER);
            leftWheel = -rightWheel*MIN_SCALER/SCALE;
        }
        else{
            leftWheel = -THOTsign*(float)map(UD_ch, MAX_DEADBAND+1, MAX_STICK, ZERO_RPM, MAX_RPM);
            float SCALE = (float)map(LR_ch, MIN_DEADBAND-1, MIN_STICK, MIN_SCALER, MAX_SCALER);
            rightWheel = -leftWheel*MIN_SCALER/SCALE;
        }

        MotorRPM[0] = rightWheel;
        MotorRPM[1] = leftWheel;

        //MotorRPM[0] = CURVsign*(float)map(UD_ch, MIN_DEADBAND-1, MIN_STICK, ZERO_RPM, -MAX_RPM);
        //float SCALE = (float)map(LR_ch, MIN_DEADBAND-1, MIN_STICK, MIN_SCALER, MAX_SCALER);
        //MotorRPM[1] = -MotorRPM[0]*MIN_SCALER/SCALE;
        //printf("SCALE %f\n",SCALE);
    }

     // user push both ch2 and ch4 diagonally (fourth quadrant), UGV curves to the right backward, one wheels is half speed of the another one
    else if(UD_ch < MIN_DEADBAND && LR_ch > MAX_DEADBAND)
    {
        if (THOTsign > 0.0){
            leftWheel = -THOTsign*(float)map(UD_ch, MIN_DEADBAND-1, MIN_STICK, ZERO_RPM, -MAX_RPM);
            float SCALE = (float)map(LR_ch, MAX_DEADBAND+1, MAX_STICK, MIN_SCALER, MAX_SCALER);
            rightWheel = -leftWheel*MIN_SCALER/SCALE;
        }
        else{
            rightWheel = THOTsign*(float)map(UD_ch, MIN_DEADBAND-1, MIN_STICK, ZERO_RPM, -MAX_RPM);
            float SCALE = (float)map(LR_ch, MAX_DEADBAND+1, MAX_STICK, MIN_SCALER, MAX_SCALER);
            leftWheel = -rightWheel*MIN_SCALER/SCALE;
        }

        MotorRPM[0] = rightWheel;
        MotorRPM[1] = leftWheel;

        //MotorRPM[1] = -CURVsign*(float)map(UD_ch, MIN_DEADBAND-1, MIN_STICK, ZERO_RPM, -MAX_RPM);
        //float SCALE = (float)map(LR_ch, MAX_DEADBAND+1, MAX_STICK, MIN_SCALER, MAX_SCALER);
        //MotorRPM[0] = -MotorRPM[1]*MIN_SCALER/SCALE;
        //printf("SCALE %f\n",SCALE);
    }  
   
}

void ODrive::vehicle_angle_control(int UD_ch, int LR_ch, float MotorDEG[2], float initDeg[2])
{   
    // UD_ch is up-down stick channel, in this case is ch2
    // LR_ch is left-right stick channel, in this case is ch4
    // MotorDEG[0] is a right wheel
    // MotorDEG[1] is a left wheel
    // we need initDeg because it'not start from zeros, this initDeg is for zero-offset

    float MIN_SCALER = 1000.0;
    float MAX_SCALER = 2000.0;
    
    /////////////////////////////////////////////////////// STRAIGHT DRIVE ////////////////////////////////////////////////////////////////
    // In case the stick near mid for both ch2 and ch4
    if (LR_ch <= MAX_DEADBAND && LR_ch >= MIN_DEADBAND && UD_ch <= MAX_DEADBAND && UD_ch >= MIN_DEADBAND)
    {
        MotorDEG[0] = 0.0+initDeg[0];
        MotorDEG[1] = 0.0+initDeg[1];
    }

    // user push ch2 up or down, UGV drive forward or backward, two wheels same speed and direction
    else if(LR_ch <= MAX_DEADBAND && LR_ch >= MIN_DEADBAND && (UD_ch > MAX_DEADBAND || UD_ch < MIN_DEADBAND))
    {
        MotorDEG[0] = -(float)map(UD_ch, MIN_STICK, MAX_STICK, (-MAX_DEG), (MAX_DEG));
        MotorDEG[1] = (float)map(UD_ch, MIN_STICK, MAX_STICK, (-MAX_DEG), (MAX_DEG));;

        MotorDEG[0] = MotorDEG[0]+initDeg[0];
        MotorDEG[1] = MotorDEG[1]+initDeg[1];
    }
    /////////////////////////////////////////////////////////// TURNS /////////////////////////////////////////////////////////////////////
    // user push ch4 left or right, UGV turns left or right, two wheels same speed but reverse direction
    else if(UD_ch <= MAX_DEADBAND && UD_ch >= MIN_DEADBAND && (LR_ch >= MAX_DEADBAND || LR_ch <= MIN_DEADBAND))
    {
        MotorDEG[1] = (float)map(LR_ch, MIN_STICK, MAX_STICK, (-MAX_DEG), (MAX_DEG));
        MotorDEG[0] = (float)map(LR_ch, MIN_STICK, MAX_STICK, (-MAX_DEG), (MAX_DEG));
        MotorDEG[0] = MotorDEG[0]+initDeg[0];
        MotorDEG[1] = MotorDEG[1]+initDeg[1];
    }
    /////////////////////////////////////////////////////////// CURVES /////////////////////////////////////////////////////////////////////
    // user push both ch2 and ch4 diagonally (first quadrant), UGV curves to the right forward, one wheels is half speed of the another one
    else if(UD_ch > MAX_DEADBAND && LR_ch > MAX_DEADBAND)
    {
        MotorDEG[1] = (float)map(UD_ch, MAX_DEADBAND+1, MAX_STICK, ZERO_DEG, (MAX_DEG));
        float SCALE = (float)map(LR_ch, MAX_DEADBAND+1, MAX_STICK, MIN_SCALER, MAX_SCALER);
        MotorDEG[0] = -MotorDEG[1]*MIN_SCALER/SCALE;

        MotorDEG[0] = MotorDEG[0]+initDeg[0];
        MotorDEG[1] = MotorDEG[1]+initDeg[1];
        //printf("SCALE %f\n",SCALE);
    } 

     // user push both ch2 and ch4 diagonally (second quadrant), UGV curves to the left forward, one wheels is half speed of the another one
    else if(UD_ch > MAX_DEADBAND && LR_ch < MIN_DEADBAND)
    {
        MotorDEG[0] = -(float)map(UD_ch, MAX_DEADBAND+1, MAX_STICK, ZERO_DEG, (MAX_DEG));
        float SCALE = (float)map(LR_ch, MIN_DEADBAND-1, MIN_STICK, MIN_SCALER, MAX_SCALER);
        MotorDEG[1] = -MotorDEG[0]*MIN_SCALER/SCALE;

        MotorDEG[0] = MotorDEG[0]+initDeg[0];
        MotorDEG[1] = MotorDEG[1]+initDeg[1];
        //printf("SCALE %f\n",SCALE);
    }   

    // user push both ch2 and ch4 diagonally (third quadrant), UGV curves to the left backward, one wheels is half speed of the another one
    else if(UD_ch < MIN_DEADBAND && LR_ch < MIN_DEADBAND)
    {
        MotorDEG[0] = -(float)map(UD_ch, MIN_DEADBAND-1, MIN_STICK, ZERO_DEG, (-MAX_DEG));
        float SCALE = (float)map(LR_ch, MIN_DEADBAND-1, MIN_STICK, MIN_SCALER, MAX_SCALER);
        MotorDEG[1] = -MotorDEG[0]*MIN_SCALER/SCALE;

        MotorDEG[0] = MotorDEG[0]+initDeg[0];
        MotorDEG[1] = MotorDEG[1]+initDeg[1];
        //printf("SCALE %f\n",SCALE);
    }

     // user push both ch2 and ch4 diagonally (fourth quadrant), UGV curves to the right backward, one wheels is half speed of the another one
    else if(UD_ch < MIN_DEADBAND && LR_ch > MAX_DEADBAND)
    {
        MotorDEG[1] = (float)map(UD_ch, MIN_DEADBAND-1, MIN_STICK, ZERO_DEG, (-MAX_DEG));
        float SCALE = (float)map(LR_ch, MAX_DEADBAND+1, MAX_STICK, MIN_SCALER, MAX_SCALER);
        MotorDEG[0] = -MotorDEG[1]*MIN_SCALER/SCALE;

        MotorDEG[0] = MotorDEG[0]+initDeg[0];
        MotorDEG[1] = MotorDEG[1]+initDeg[1];
        //printf("SCALE %f\n",SCALE);
    }  
}