/*
 * atdrive-moab
 *
 * Firmware (using mbed-os) for the Motor and I/O Control Board
 *    (called  AT-Drive, Moab)
 *
 * Copyright 2019 Mark Fassler
 * Licensed under the GPLv3
 *
 */


#include "mbed.h"
#include "rtos.h"
#include "EthernetInterface.h"

#include "ROBOT_CONFIG.hpp"
#include "EVENT_FLAGS.hpp"
#include "MOAB_DEFINITIONS.h"

//#include "misc_math.hpp"

#include "daemons/IMU_daemon.hpp"
#include "daemons/GPS_daemon.hpp"
#include "daemons/RTCM3_daemon.hpp"
#include "daemons/PushButton_daemon.hpp"

#include "SbusParser.hpp"
//#include "MotorControl.hpp"
//#include "XWheels.hpp"
#include "odrive.hpp"

EventFlags event_flags;

bool NETWORK_IS_UP = false;
 
// Network interface
EthernetInterface net;
UDPSocket rx_sock; // one, single thread for RX
UDPSocket tx_sock; // tx will be completely non-blocking

Thread udp_rx_thread;
Thread sbus_reTx_thread;

// Heartbeat LED:
PwmOut hb_led(PA_6);

// Flight-mode LEDs:
DigitalOut myledR(LED3, 0);
DigitalOut myledG(LED1, 0);
DigitalOut myledB(LED2, 0);
DigitalOut velocityModeLED(PC_7, 0);


uint16_t sbus_a_forImuPacket = 0;
uint16_t sbus_b_forImuPacket = 0;

// *** NOTE: By default, mbed-os only allows for 4 sockets, so
// *** we will re-use the tx_sock (non-blocking) wherever possible.

// Background I/O processes:
//  (minimal inter-dependence; mostly independent of anything else)
IMU_daemon imu_daemon(&tx_sock, &sbus_a_forImuPacket, &sbus_b_forImuPacket);
GPS_daemon gps_daemon(PE_8, PE_7, &net);
//RTCM3_daemon rtcm3_daemon(PD_5, PD_6, &tx_sock);
PushButton_daemon pushButton_daemon(PE_9, &tx_sock);


// Motors:
//MotorControl motorControl(PD_14, PD_15);
//XWheels drive(PD_1, PD_0);
//float motorRPM[2];
ODrive odrive(PD_1,PD_0, &tx_sock);

// S.Bus is 100000Hz, 8E2, electrically inverted
RawSerial sbus_in(NC, PD_2, 100000);  // tx, then rx


void u_printf(const char *fmt, ...) {
	va_list args;
	char buffer[1500];
	int bufLen;

	va_start(args, fmt);
	bufLen = vsnprintf(buffer, 1499, fmt, args);
	va_end(args);

	int retval = tx_sock.sendto(_BROADCAST_IP_ADDRESS, UDP_PORT_DEBUG, buffer, bufLen);

	if (retval < 0 && NETWORK_IS_UP) {
		printf("sock.err in u_printf(): %d\r\n", retval);
		return;
	}
}


//uint16_t auto_ch1 = 1024;
//uint16_t auto_ch2 = 1024;
float rpmR;
float rpmL;

////////////// ODrive related parameters ///////////////////
uint32_t des_com_mode;  // desired command mode from autopilot script
float motorRPM[2]; 		// pass by refenrece to odrive method
float degR; 			// for ODrive used (position control)
float degL; 			// for ODrive used (position control)
float motorDEG[2]; 		// pass by refenrece to odrive method
float initDeg[2];		
float reported_RPMs[2];
float reported_DEGs[2];
int reported_odrive_mode;

void udp_rx_worker() {
	/*
	 * Here we receive throttle and steering control from the auto-pilot computer
	 */
	SocketAddress sockAddr;
	// for Odrive, there are (mode, rpmR, rpmL) that we will receive from autopilot script
	char inputBuffer[96];		//char inputBuffer[33];
	inputBuffer[95] = 0; 		//inputBuffer[32] = 0;
	uint64_t _last_autopilot = 0;

	float *control = (float *) &(inputBuffer[0]);

	rx_sock.set_blocking(true);
	rx_sock.set_timeout(500);

	while (true) {

		int n = rx_sock.recvfrom(&sockAddr, inputBuffer, 96);
		uint64_t ts = rtos::Kernel::get_ms_count();
		if (ts - _last_autopilot > 500) {
			if (rpmR != 0.0 || rpmL != 0.0) {
				u_printf("Timeout: resetting auto sbus values\n");
				//auto_ch1 = 1024;
				//auto_ch2 = 1024;
				rpmR = 0.0;
				rpmL = 0.0;
				degR = 0.0;
				degL = 0.0;
			}
		}
			//n == 2*sizeof(float)
		if (n == 3*sizeof(float)) {
			_last_autopilot = ts;
			//auto_ch1 = control[0];
			//auto_ch2 = control[1];

			des_com_mode = (uint32_t)control[0];
			
			if (des_com_mode == 2.0){
				rpmR = control[1];
				rpmL = control[2];
			}
			else if (des_com_mode == 3.0){
				degR = control[1];
				degL = control[2];
			}
			//u_printf("des_com_mode: %d  R:%f  L:%f \n", des_com_mode,control[1],control[2]);
			
		} else if (n > 0) {
			inputBuffer[n] = 0;
			printf("rx %d bytes:\r\n", n);
			printf(inputBuffer);
			printf("\r\n");
		} else {
			//printf("empty packet\n");
		}
	}
}


struct sbus_udp_payload sbup;
SbusParser sbusParser(&sbup);



void set_mode_sbus_failsafe() {
	myledR = 0;
	myledG = 0;
	myledB = 0;

	//motorControl.set_steering(1024);
	//motorControl.set_throttle(352);
	//drive.setRPMs(drive.ZERO_RPM, drive.ZERO_RPM);
	odrive.set_RPMs(0.0,0.0);

	sbus_a_forImuPacket = 1024;
	sbus_b_forImuPacket = 352;
}

void set_mode_stop() {
	myledR = 1;
	myledG = 0;
	myledB = 0;

	//motorControl.set_steering(sbup.ch1);
	//motorControl.set_throttle(352);
	//drive.setRPMs(drive.ZERO_RPM, drive.ZERO_RPM);
	odrive.set_RPMs(0.0,0.0);

	sbus_a_forImuPacket = 1024;
	sbus_b_forImuPacket = 352;
}

void set_mode_manual() {
	myledR = 0;
	myledG = 1;
	myledB = 0;

#ifdef _FUTABA
	if (reported_odrive_mode == 2){
		//this LED is next to the hb_led, 1 means VELOCITY CONTROL
		velocityModeLED = 1; 
		odrive.vehicle_control(sbup.ch2, sbup.ch4, motorRPM);
		odrive.set_RPMs(motorRPM[0], motorRPM[1]);
		//u_printf("rpmR: %f   rpmL: %f\n", motorRPM[0], motorRPM[1]);
		//odrive.get_reported_RPMs(reported_RPMs);
		
	}
	else if (reported_odrive_mode == 3){
		//this LED is next to the hb_led, 0 means POSITION CONTROL
		velocityModeLED = 0; 
		// when we changed the mode from vel to pos and reset, the value of position doesn't reset
		// so we need to read the initial value for an offset of zero degree
		odrive.get_started_DEGs(initDeg);
		// we also input initDeg to use it as an offset (like starting point reference)
		odrive.vehicle_angle_control(sbup.ch2, sbup.ch4, motorDEG, initDeg);
		odrive.set_DEGs(motorDEG[0],motorDEG[1]);
		//odrive.get_reported_DEGs(reported_DEGs);
	}
	else{
		u_printf("...something wrong in manual mode\n");
	}
	sbus_a_forImuPacket = sbup.ch4;
	sbus_b_forImuPacket = sbup.ch2;
#endif

#ifdef _LTE_PROPO
	velocityModeLED = 1; 
	if (reported_odrive_mode == 2){
		odrive.vehicle_control(sbup.ch2, sbup.ch1, motorRPM);
		odrive.set_RPMs(motorRPM[0], motorRPM[1]);
	}
	else{
		u_printf("...something wrong in manual mode\n");
	}
	
	// right now, only velocity control

	sbus_a_forImuPacket = sbup.ch1;
	sbus_b_forImuPacket = sbup.ch2;
#endif


}

void set_mode_auto() {
	myledR = 0;
	myledG = 0;
	myledB = 1;

#ifdef _FUTABA
	if (reported_odrive_mode == 2){
		velocityModeLED = 1; //this LED is next to the hb_led, 1 means VELOCITY CONTROL

		// In case changed the direction of UGV
		if (odrive.THOTsign > 0.0){
			odrive.set_RPMs(odrive.THOTsign*rpmR, -odrive.THOTsign*rpmL);       //odrive.set_RPMs(-rpmR, rpmL);
		}
		else{
			odrive.set_RPMs(odrive.THOTsign*rpmL, -odrive.THOTsign*rpmR);       //odrive.set_RPMs(-rpmR, rpmL);
		}
		//odrive.get_reported_RPMs(reported_RPMs);
	}
	else if (reported_odrive_mode == 3){
		velocityModeLED = 0; //this LED is next to the hb_led, 0 means POSITION CONTROL
		odrive.get_started_DEGs(initDeg);
		odrive.set_DEGs((-degR+initDeg[0]),(-degL+initDeg[1]));
		//odrive.get_reported_DEGs(reported_DEGs);

	}
	else{
		u_printf("...something wrong in automode\n");
	}
	sbus_a_forImuPacket = sbup.ch4;
	sbus_b_forImuPacket = sbup.ch2;
#endif

#ifdef _LTE_PROPO
	if (reported_odrive_mode == 2){
		velocityModeLED = 1; 
		// In case changed the direction of UGV
		if (odrive.THOTsign > 0.0){
			odrive.set_RPMs(odrive.THOTsign*rpmR, -odrive.THOTsign*rpmL);       //odrive.set_RPMs(-rpmR, rpmL);
		}
		else{
			odrive.set_RPMs(odrive.THOTsign*rpmL, -odrive.THOTsign*rpmR);       //odrive.set_RPMs(-rpmR, rpmL);
		}
		
	}
	else{
		u_printf("...something wrong in automode\n");
	}
	sbus_a_forImuPacket = sbup.ch1;
	sbus_b_forImuPacket = sbup.ch2;
#endif

}



void Sbus_Rx_Interrupt() {

	int c;

	while (sbus_in.readable()) {

		c = sbus_in.getc();
		int status = sbusParser.rx_char(c);

		if (status == 1) {
			event_flags.set(_EVENT_FLAG_SBUS);
		}
	}
}



void sbus_reTx_worker() {

	uint32_t flags_read;
#ifdef _FUTABA
	bool inter_lock_man = true;
	bool inter_lock_auto = true;
#endif
#ifdef _LTE_PROPO
	bool man_lock = true;
	bool stop_lock = false;
	bool auto_lock= false;
#endif

	while (true) {
		flags_read = event_flags.wait_any(_EVENT_FLAG_SBUS, 100);

		if (flags_read & osFlagsError) {
			u_printf("S.Bus timeout!\n");
			sbup.failsafe = true;
			set_mode_sbus_failsafe();
		} else if (sbup.failsafe) {
			u_printf("S.Bus failsafe!\n");
			set_mode_sbus_failsafe();
		} else {

			reported_odrive_mode = odrive.get_reported_mode();

#ifdef _FUTABA
		
			if (sbup.ch5 < 688) {
				set_mode_stop();
			} 
			else if (sbup.ch5 < 1360) {
				
				// This condition below is to check and to change controller mode (in MANUAL case)//
				if ((sbup.ch9 > 1500) && inter_lock_man) {

					// send a trig to let odrive thread knows there is some mode-change request
					odrive.mode_trigger(true);
					inter_lock_man = false;
				}
				set_mode_manual();

			} 
			else {

				if ((des_com_mode != 0) && (reported_odrive_mode != 0)){

					if ((reported_odrive_mode != des_com_mode) && inter_lock_auto) {
						
						// similar to manual, this will send a trig to odrive thread
						odrive.mode_trigger(true);
						inter_lock_auto = false;
					}

					set_mode_auto();
				}
	
			}
#endif

#ifdef _LTE_PROPO
			if (sbup.ch8 > 1024 || sbup.ch7 > 1024 || stop_lock == true && ((sbup.ch5 < 1024) && (sbup.ch6 < 1024)) ) {
				
				set_mode_stop();
				man_lock = false;
				stop_lock = true;
				auto_lock = false;
				//u_printf("stop_mode");

			} else if (sbup.ch5 > 1024 || man_lock == true && !(sbup.ch6 > 1024)) {
				
				set_mode_manual();
				man_lock = true;
				stop_lock = false;
				auto_lock = false;
				//u_printf("manual_mode");

			} else if (sbup.ch6 > 1024 || auto_lock == true){
				set_mode_auto();
				man_lock = false;
				stop_lock = false;
				auto_lock= true;
				//u_printf("auto_mode");
			}
			else{
				u_printf("none_mode");
			}
#endif

			int retval = tx_sock.sendto(_AUTOPILOT_IP_ADDRESS, UDP_PORT_SBUS,
					(char *) &sbup, sizeof(struct sbus_udp_payload));

			if (retval < 0 && NETWORK_IS_UP) {
				printf("UDP socket error in sbus_reTx_worker\r\n");
			}
		}
	}
}


void eth_callback(nsapi_event_t status, intptr_t param) {
	const char *ip;

	printf("Connection status changed!\r\n");
	switch(param) {
		case NSAPI_STATUS_LOCAL_UP:
			NETWORK_IS_UP = false;
			printf("Local IP address set.\r\n");
			break;
		case NSAPI_STATUS_GLOBAL_UP:
			printf("Global IP address set.  ");
			NETWORK_IS_UP = true;
			ip = net.get_ip_address();  // <--dhcp
			if (ip) {
				printf("IP address is: %s\r\n", ip);
			} else {
				printf("no IP address... we're screwed\r\n");
				//return -1;
			}
			break;
		case NSAPI_STATUS_DISCONNECTED:
			NETWORK_IS_UP = false;
			printf("No connection to network.\r\n");
			break;
		case NSAPI_STATUS_CONNECTING:
			NETWORK_IS_UP = false;
			printf("Connecting to network...\r\n");
			break;
		default:
			NETWORK_IS_UP = false;
			printf("Not supported\r\n");
			break;
	}
}

 
int main() {
	printf("\r\n");
	printf("   ##### This is AT-Drive Moab #####\r\n");

	//  ######################################
	//  #########################################
	//  ###########################################
	//   BEGIN:  setup network and udp socket
	//  ############################################

	printf("Starting the network...\r\n");

	net.attach(&eth_callback);
	net.set_dhcp(false);
	net.set_network(_MOAB_IP_ADDRESS, _NETMASK, _DEFUALT_GATEWAY);
	net.set_blocking(false);

	net.connect();

	//  ############################################
	//   END:  setup network and udp socket
	//  ###########################################
	//  #########################################
	//  ######################################


	// UDP Sockets
	rx_sock.open(&net);
	rx_sock.bind(12346);

	tx_sock.open(&net);
	tx_sock.bind(12347);
	tx_sock.set_blocking(false);


	// Serial ports
	sbus_in.format(8, SerialBase::Even, 2);  // S.Bus is 8E2
	sbus_in.attach(&Sbus_Rx_Interrupt);


	// Background threads
	udp_rx_thread.start(udp_rx_worker);
	sbus_reTx_thread.start(sbus_reTx_worker);

	imu_daemon.Start();  // will start a separate thread
	gps_daemon.Start();  // will start a separate thread
	//rtcm3_daemon.Start();  // will start a separate thread
	//pushButton_daemon.Start();  // will start a separate thread


	//drive.Start(); // will start a separate thread
	odrive.Start();

	hb_led.period(0.02);
	hb_led.write(0.0);

	for (int ct=0; true; ++ct){

		// Heartbeat LED glows brighter:
		for (int i=0; i < 11; ++i) {
				float brightness = i/10.0;
				hb_led.write(brightness);
				ThisThread::sleep_for(20);
		}
		// Heartbeat LED dims darker:
		for (int i=0; i < 11; ++i) {
				float brightness = 1.0 - i/10.0;
				hb_led.write(brightness);
				ThisThread::sleep_for(20);
		}

		u_printf("heartbeat: %d\n", ct);

		// Report motor values (for convience when setting trim)
		//uint16_t sbus_a = motorControl.get_value_a();
		//float pw_a = motorControl.get_pw_a();
		//u_printf("steering: %d %f\n", sbus_a, pw_a);

		//uint16_t sbus_b = motorControl.get_value_b();
		//float pw_b = motorControl.get_pw_b();
		//u_printf("throttle: %d %f\n", sbus_b, pw_b);


	}

   
	// Close the socket and bring down the network interface
	rx_sock.close();
	tx_sock.close();
	net.disconnect();
	return 0;
}