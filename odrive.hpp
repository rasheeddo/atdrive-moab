#ifndef ODrive_h
#define ODrive_h
#include <string>
#include <sstream>
#include <stdint.h>
#include "mbed.h"
#include <cstdlib>
#include "ROBOT_CONFIG.hpp"
#include "MOAB_DEFINITIONS.h"

struct odrive_data{
        int _mode;
        float _rpmR;
        float _rpmL;
        float _degR;
        float _degL;
        float _initDegR;
        float _initDegL;
};
   

class ODrive {

    public:
        /////////////////////////////////// ENUMERATION ///////////////////////////////////
        enum AxisState_t {
            AXIS_STATE_UNDEFINED = 0,           //<! will fall through to idle
            AXIS_STATE_IDLE = 1,                //<! disable PWM and do nothing
            AXIS_STATE_STARTUP_SEQUENCE = 2, //<! the actual sequence is defined by the config.startup_... flags
            AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3,   //<! run all calibration procedures, then idle
            AXIS_STATE_MOTOR_CALIBRATION = 4,   //<! run motor calibration
            AXIS_STATE_SENSORLESS_CONTROL = 5,  //<! run sensorless control
            AXIS_STATE_ENCODER_INDEX_SEARCH = 6, //<! run encoder index search
            AXIS_STATE_ENCODER_OFFSET_CALIBRATION = 7, //<! run encoder offset calibration
            AXIS_STATE_CLOSED_LOOP_CONTROL = 8  //<! run closed loop control
        };

        enum ControlMode {
            CTRL_MODE_VELOCITY_CONTROL = 2,
            CTRL_MODE_POSITION_CONTROL = 3,
        };
         /////////////////////////////////// CLASS CONSTRUCTOR ///////////////////////////////////
        ODrive(PinName, PinName, UDPSocket*);
        std::string readString();

        /////////////////////////////////// SYSTEM & CONFIG ///////////////////////////////////

        void Start();

        /////////////////////////////////// OUTSIDE THREAD USED ///////////////////////////////////
        int get_reported_mode();
        void mode_trigger(bool trig);
        void set_RPMs(float rpmR, float rpmL);
        void set_DEGs(float degR, float degL);
        void get_reported_RPMs(float reportedRPMs[2]);
        void get_reported_DEGs(float reportedDEGs[2]);
        void get_started_DEGs(float startedDEGs[2]);

        void vehicle_control(int UD_ch, int LR_ch, float MotorRPM[2]);
        void vehicle_angle_control(int UD_ch, int LR_ch, float MotorDEG[2], float initDeg[2]);


        float THOTsign = 1.0;      //Normal forward: bigWheel 1.0,  smallWheel -1.0
        float SKIDsign = -1.0;     //bigWheel -1.0, smallWheel 1.0
        //float CURVsign = 1.0;      //Normarl forward: bigWheel 1.0  smalWheel -1.0

    private:

        /////////////////////////////////// SYSTEM & CONFIG ///////////////////////////////////
        PinName _tx_pin;
        PinName _rx_pin;

        RawSerial *_uart;
        UDPSocket *_sock;

        Thread main_thread;

        struct odrive_data _odriveData;

        void main_worker();
        bool Init();
        void switchController(int mode);
        void saveConfig();
        void rebootODrive();
        void changeMode(int motor_number, int change_mode);
        int readControlMode(int motor_number);
        void runState(int axis, int requested_state);   // State helper
        int readError(int motor_number);

        /////////////////////////////////// MATHEMATICS and NUMBERS ///////////////////////////////////
        float readFloat();
        int32_t readInt();
        //long map(long x, long in_min, long in_max, long out_min, long out_max);
        float cpsTOrpm(float cps);
        float rpmTOcps(float rpm); 
        float degTOcount(float deg);
        float countTOdeg(float count);

        /////////////////////////////////// READ, WRITE VELOCITY ///////////////////////////////////
        void SetVelRamp(int motor_number, bool flag);
        void SetRPM_traj(int motor_number, float RPM);
        //void SetRPM(int motor_number, float RPM);
        float GetVelocity(int motor_number);
        float GetRPM(int motor_number);
        void DriveWheelsRPMs(float rpmR, float rpmL);

        /////////////////////////////////// READ, WRITE POSITION ///////////////////////////////////
        void MoveToPos(int motor_number, float deg);
        float GetDEG(int motor_number);
        float GetPosition(int motor_number);
        void DriveWheelsAngle(float degR, float degL);


        /////////////////////////////////// PARAMETERS, VARIABLES ///////////////////////////////////
        int _Axis0_mode;
        int _Axis1_mode;
        float cpr = 839.16;   //258.0 for small wheels  839.16 for big wheels

#ifdef _FUTABA        
        float MAX_RPM = 110.0;  //200.0 for small wheels  1110.0 for big wheels // Max RPM of the wheels, this is limited by wheels itself. Default is 144
        float ZERO_RPM = 0.0;          // No speed
        
        int MIN_STICK = 360;       
        int MAX_STICK = 1673;     

        int MIN_DEADBAND = 1014;
        int MAX_DEADBAND = 1034;

        int MID_STICK = 1024;
#endif

#ifdef _LTE_PROPO

        float MAX_RPM = 80.0;
        float ZERO_RPM = 0.0;

        int MIN_STICK = 283;     
        int MAX_STICK = 1758;    

        int MIN_DEADBAND = 924;
        int MAX_DEADBAND = 1124;

        int MID_STICK = 1024;
#endif
        
        int DIVIDER = 2;           // a divider of another wheel's speed, e.g. 2 is half speed of the another wheel's speed

        float R_wheel = 0.15;    // small wheel 10.34cm, big wheel 15cm  
        float MAX_DEG = 144.0;   // turn the wheel with this angle will make the robot turn 90deg (skidding)
                                 // small wheel 201.0 deg,  big wheel 144.0 deg
        float ZERO_DEG = 0.0;

        // update var
        float _cur_rpmR;
        float _cur_rpmL;
        float _cur_degR;
        float _cur_degL;

        // new
        float _startDeg[2];
        int _mode;
        float _rpmR;
        float _rpmL;
        float _degR;
        float _degL;

        bool _mode_trig = false;

};
 
#endif //ODrive_h