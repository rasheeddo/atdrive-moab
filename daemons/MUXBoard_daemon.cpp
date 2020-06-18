
#include "MUXBoard_daemon.hpp"


extern void u_printf(const char *fmt, ...);  // Defined in main()


MUXBoard_daemon::MUXBoard_daemon(PinName tx, PinName rx, UDPSocket *tx_sock) {
	
	_bat_mon = new RawSerial(tx, rx, 115200);  //tx, then rx
	_usb_debug = new RawSerial(USBTX,USBRX,115200);
    _rxbuf = new CircularBuffer<char, MUX_BUF_SIZE>;
	_sock = tx_sock;
}

void MUXBoard_daemon::Rx_Interrupt(){
    char c;
    while (_bat_mon->readable()) {
        c = _bat_mon->getc();
        if(_rxbuf->full() == false){
            _rxbuf->push(c);
        }
    }    
}

void MUXBoard_daemon::Start() {
	main_thread.start(callback(this, &MUXBoard_daemon::main_worker));
}


void MUXBoard_daemon::main_worker() {

    uint8_t noResponceCount = 0;
    uint8_t state = 0;
    uint8_t f_waitRcev = 0;
    uint8_t printCount = 0;

	ThisThread::sleep_for(1000);
    _bat_mon->attach(callback(this, &MUXBoard_daemon::Rx_Interrupt));


	while(1){
        if(f_waitRcev){
            switch(recvInfo()){
                case 0:
                    if(noResponceCount >= 10){
                        _usb_debug->printf("MUX board no response!\n");
                        noResponceCount = 0;
                        f_waitRcev = 0;
                    }
                    else{
                        noResponceCount++;
                    }
                    break;
                case 1:
                case -1:
                    f_waitRcev = 0;
                    noResponceCount = 0;
                    break;
            }
        }
        else {
            switch(state){
                case 0:
                    _bat_mon->puts("g current 1\n");
                    f_waitRcev = 1;
                    break;
                case 1:
                    _bat_mon->puts("g current 2\n");
                    f_waitRcev = 1;
                    break;
                case 2:
                    _bat_mon->puts("g tmp 1\n");
                    f_waitRcev = 1;
                    break;
                case 3:
                    _bat_mon->puts("g tmp 2\n");
                    f_waitRcev = 1;
                    break;
                case 4:
                    _bat_mon->puts("g bat 1\n");
                    f_waitRcev = 1;
                    break;
                case 5:
                    _bat_mon->puts("g bat 5\n");
                    f_waitRcev = 1;
                    break;
            }
            if(++state>5){
                state = 0;
            }
        }

        if((++printCount)>10){
            printCount = 0;
            //u_printf("currentSensor1 = %.3f A\n", I2C_MUX_BOARD_common.current1/1000.0);
            _usb_debug->printf("currentSensor1 = %.3f A\n", I2C_MUX_BOARD_common.current1/1000.0);
            //u_printf("currentSensor2 = %.3f A\n", I2C_MUX_BOARD_common.current2/1000.0);
            _usb_debug->printf("currentSensor2 = %.3f A\n", I2C_MUX_BOARD_common.current2/1000.0);
            //u_printf("tmpSensor1 = %.2f degC\n", I2C_MUX_BOARD_common.tmp1/100.0);
            _usb_debug->printf("tmpSensor1 = %.2f degC\n", I2C_MUX_BOARD_common.tmp1/100.0);
            //u_printf("tmpSensor2 = %.2f degC\n", I2C_MUX_BOARD_common.tmp2/100.0);
            _usb_debug->printf("tmpSensor2 = %.2f degC\n", I2C_MUX_BOARD_common.tmp2/100.0);       
            _usb_debug->printf("BAT1 V:%d mV, I:%d mA, SOC:%d \n",
            I2C_MUX_BOARD_BattInfo[0].volt,
                I2C_MUX_BOARD_BattInfo[0].current,
            I2C_MUX_BOARD_BattInfo[0].SOC);
            _usb_debug->printf("BAT5 V:%d mV, I:%d mA, SOC:%d \n",
            I2C_MUX_BOARD_BattInfo[4].volt,
                I2C_MUX_BOARD_BattInfo[4].current,
            I2C_MUX_BOARD_BattInfo[4].SOC);
            _usb_debug->printf("\n");
        }

#if 0
        if(_flag_rcevData == 0){
            if(noResponceCount>=30){
                _usb_debug->printf("MUX board no response!\n");
            }
            else{
                noResponceCount++;
            }
        }
        else{
            noResponceCount = 0;
            _flag_rcevData = 0;
        }
#endif
        /*
        for(int i=1;i<=6;i++)
        {
            _bat_mon->printf("g bat %d\n",i);
            recvInfo();
            if(I2C_MUX_BOARD_BattInfo[i-1].volt != 0){
            	
                //u_printf("BAT%d V:%d mV, I:%d mA, SOC:%d %%\n",
                 //   i,
                //   I2C_MUX_BOARD_BattInfo[i-1].volt,
                 //   I2C_MUX_BOARD_BattInfo[i-1].current,
                //   I2C_MUX_BOARD_BattInfo[i-1].SOC);
				
                _usb_debug->printf("BAT%d V:%d mV, I:%d mA, SOC:%d %%\n",
                    i,
                   I2C_MUX_BOARD_BattInfo[i-1].volt,
                    I2C_MUX_BOARD_BattInfo[i-1].current,
                   I2C_MUX_BOARD_BattInfo[i-1].SOC);
            }
        }*/

        //u_printf("\n");
        ThisThread::sleep_for(100);
	}
}

void MUXBoard_daemon::recvBatteryInfo(char *cstr, uint8_t ch){ 

	char *token;
    ch = ch -1;

	token = strtok(cstr, ",");
    I2C_MUX_BOARD_BattInfo[ch].temp = atoi(token);
    token = strtok(NULL, ",");
    I2C_MUX_BOARD_BattInfo[ch].volt = atoi(token);
    token = strtok(NULL, ",");
    I2C_MUX_BOARD_BattInfo[ch].current = atoi(token)*2;
    token = strtok(NULL, ",");
    I2C_MUX_BOARD_BattInfo[ch].SOC = atoi(token);
    token = strtok(NULL, ",");
    I2C_MUX_BOARD_BattInfo[ch].SOH = atoi(token);
    token = strtok(NULL, ",");
    I2C_MUX_BOARD_BattInfo[ch].CellVoltage6 = atoi(token);
    token = strtok(NULL, ",");
    I2C_MUX_BOARD_BattInfo[ch].CellVoltage5 = atoi(token);
    token = strtok(NULL, ",");
    I2C_MUX_BOARD_BattInfo[ch].CellVoltage4 = atoi(token);
    token = strtok(NULL, ",");
    I2C_MUX_BOARD_BattInfo[ch].CellVoltage3 = atoi(token);
    token = strtok(NULL, ",");
    I2C_MUX_BOARD_BattInfo[ch].CellVoltage2 = atoi(token);
    token = strtok(NULL, ",");
    I2C_MUX_BOARD_BattInfo[ch].CellVoltage1 = atoi(token);
    token = strtok(NULL, ",");
    I2C_MUX_BOARD_BattInfo[ch].PFAlert = atoi(token);
    token = strtok(NULL, ",");
    I2C_MUX_BOARD_BattInfo[ch].PFStatus = atoi(token);
    token = strtok(NULL, ",");
    I2C_MUX_BOARD_BattInfo[ch].CycleCount = atoi(token);
    token = strtok(NULL, ",");
    I2C_MUX_BOARD_BattInfo[ch].DesignVoltage = atoi(token);
    token = strtok(NULL, ",");
    I2C_MUX_BOARD_BattInfo[ch].SerialNumber = atoi(token);
    token = strtok(NULL, ",");
    I2C_MUX_BOARD_BattInfo[ch].MaxTempCell = atoi(token);
    token = strtok(NULL, ",");
    I2C_MUX_BOARD_BattInfo[ch].LastFCCUpdate = atoi(token);
    token = strtok(NULL, ",");
    I2C_MUX_BOARD_BattInfo[ch].TotalFwRuntime = atoi(token);
    token = strtok(NULL, ",");
    I2C_MUX_BOARD_BattInfo[ch].TimeSpentinOT = atoi(token);

}

void MUXBoard_daemon::recvCurrentInfo(char *cstr, uint8_t ch){

    switch(ch)
    {
        case 1:
            I2C_MUX_BOARD_common.current1 = atoi(cstr);
            break;
        case 2:
            I2C_MUX_BOARD_common.current2 = atoi(cstr);
            break;
    }    
}

void MUXBoard_daemon::recvTmpInfo(char *cstr, uint8_t ch){

    switch(ch)
    {
        case 1:
            I2C_MUX_BOARD_common.tmp1 = atoi(cstr);
            break;
        case 2:
            I2C_MUX_BOARD_common.tmp2 = atoi(cstr);
            break;
    }
}

/*
return vel
 -1:erro ditect
  0:receve none
  1:receve success
*/
int8_t MUXBoard_daemon::recvInfo(void){

    char *strCmd;
    char *strCh;
    char *parseTargetStr;
    char c;
    uint8_t ch;
    uint8_t f_rcevData = 0;

    const static uint8_t response_size =128;
    static uint8_t _len=0;
    static char response[128];

    while(_rxbuf->empty() == false){
        _rxbuf->pop(c);
        if(c == '\n'){
            response[_len] = '\0';
            f_rcevData = 1;
            _len = 0;
            break;
        }
        else{
            response[_len] = c;
        }
        if((++_len)>response_size){
            _len = 0;
        }
    }

    if(f_rcevData){
        f_rcevData = 0;

        strCmd = strtok((char *)response, ",");
        strCh = strtok(NULL, ",");
        ch = atoi(strCh);

        if(strstr(strCmd,"batt") != NULL){
            if( (ch >= 1) && (ch <= 6) ){
                recvBatteryInfo(strCh+2,ch);
            }
        }
        else if(strstr(strCmd,"tmp") != NULL){
            if( (ch >= 1) && (ch <= 2) ){
                recvTmpInfo(strCh+2,ch);
            }
        }
        else if(strstr(strCmd,"current") != NULL){
            if( (ch >= 1) && (ch <= 2) ){
                recvCurrentInfo(strCh+2,ch);
            }
        }
        else if(strstr(strCmd,"ERROR") != NULL){
            _usb_debug->printf("error ditect\n");
            u_printf("MUXBoard_daemon: error ditect\n");
            return -1; 
        }
        else
        {

        }
        return 1;
    }

    return 0;
}

