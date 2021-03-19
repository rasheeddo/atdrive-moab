#ifndef __X_WHEELS_HPP
#define __X_WHEELS_HPP

#include "mbed.h"
#include "rtos.h"
#include "ROBOT_CONFIG.hpp"

class XWheels
{
    public:
        XWheels(PinName, PinName);
        //class constructor to initialize baudrate setting 

        void Start();
        
        ////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////// Drive function ////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////
        void setRPMs(float, float);
        // setRPMs: copy RPM values from main thread to XWheels's thread

        void vehicleControl(int UD_ch, int LR_ch, float MotorRPM[2]);
        // vehcileControl: Convert the stick value to motor's RPM value and pass it to DriveWheels function

        void vehicleControlProportionalMixing(int UD_ch, int LR_ch, float MotorRPM[2]);
        // vehicleControlProportionalMixing: another option of stick mixing mode, this is more natural feeling
#ifdef _KO_PROPO

        int MIN_STICK = 430;     
        int MAX_STICK = 1600;    

        int MIN_DEADBAND = 999;
        int MAX_DEADBAND = 1029; //1019

        int MID_STICK = 1019;   //1009
        int DIVIDER = 2;           // a divider of another wheel's speed, e.g. 2 is half speed of the another wheel's speed

        float MAX_RPM = 100.0;
        float ZERO_RPM = 0.0;

#endif

#ifdef _LTE_PROPO

        int MIN_STICK = 283;     
        int MAX_STICK = 1758;    

        int MIN_DEADBAND = 924;
        int MAX_DEADBAND = 1124;

        int MID_STICK = 1024;
        int DIVIDER = 2;           // a divider of another wheel's speed, e.g. 2 is half speed of the another wheel's speed

        float MAX_RPM = 130.0;
        float ZERO_RPM = 0.0;  //1.4720
#endif

#ifdef _FUTABA

        int MIN_STICK = 360;       
        int MAX_STICK = 1673;      

        int MIN_DEADBAND = 1014;
        int MAX_DEADBAND = 1034;

        int MID_STICK = 1024;
        int DIVIDER = 2;           // a divider of another wheel's speed, e.g. 2 is half speed of the another wheel's speed

        float MAX_RPM = 130.0;         // Max RPM of the wheels, this is limited by wheels itself. Default is 144
        float ZERO_RPM = 0.0;        // this may need to adjust according to each robot,
                                       // it's the minimum value that the robot will keep click the relay but the wheel doesn't turn
                                       // This will trick the ESC that the robot pretend to stop
                                       // because when it comes back to move from a long rest, it might cause error without any reason...
                                       // one way I fixed that is to check th Rx_Interrupt reply and see the 4th byte
                                       // another way to fix that is to use this minimum ZERO_RPM

                                       // 1.472 for Yorii UGV
                                       // 0.0 for ATCart test-ugv  there is no problem with this one
#endif

        
 

    private:
        
        PinName _tx_pin;
        PinName _rx_pin;

        RawSerial *_uart;

        float _rpm_motor_1;
        float _rpm_motor_2;

        Thread main_thread;

        void main_worker();
        // main thread on this XWheels
        void Rx_Interrupt();
        ////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////// Data Converting function /////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////

        unsigned int RPM_to_data(float rpm);
        // RPM_to_data: convert rotation per minute value to raw value
        // input is a float number between -MaxRPM to MaxRPM, minus sign means reverse rotation
        // output is a value from 0 to 3200 for forward rotation and 32769 to 35968 for reverse rotation
        // Not using due to the there are two linear behavior of the wheels for low and high speed

        ////////////////////////////////////////////////////////////////////////////////////////////////////
        ///////////////////////////////////// Hand Shake with ESC //////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////

        void waitUntilFourZero();
        // waitUntilFourZero: when start, ESC will send four bytes of zero then wait for our controller to response
        // if our control received four bytes of zero, then it will allow next command to run after

        void ESCHandShake();
        // ESCHandShake: from hack, we need to send 17bytes of "Hand-Shake" style 20times with specific delay


        ////////////////////////////////////////////////////////////////////////////////////////////////////
        //////////////////////////////////////// Drive function ////////////////////////////////////////////
        ////////////////////////////////////////////////////////////////////////////////////////////////////

        void send_motor_command();
        // send_motor_command: drive both wheels with desired rpm, + for forward - for reverse rotations

        // normal sending
        unsigned char Header1;
        unsigned char Header2;
        // handshake sending
        unsigned char InitHeader1;
        unsigned char InitHeader2;
        unsigned char ForwardAcc;
        unsigned char ForwardDelay;
        unsigned char BrakeDis;
        unsigned char TurnAcc;
        unsigned char TurnDelay;
        unsigned char AccTimeOfStart;
        unsigned char SenRocker;
        unsigned char UnderVolt1;
        unsigned char UnderVolt2;
        unsigned char StartSpeed;
        unsigned char DriveMode;
        unsigned char PhaseLMotor;
        unsigned char PhaseRMotor;
        unsigned char MotorConfig;
        unsigned char InitCheckSum;

        float _pre_Y;


};


#endif