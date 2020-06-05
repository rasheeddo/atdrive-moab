
#include "MUXBoard_daemon.hpp"

volatile static uint8_t _flag_rcevData = 0;
volatile static uint8_t _len=0;
volatile static char response_tmp[128];
volatile static char response[128];

extern void u_printf(const char *fmt, ...);  // Defined in main()


MUXBoard_daemon::MUXBoard_daemon(PinName tx, PinName rx, UDPSocket *tx_sock) {
	
	_bat_mon = new RawSerial(tx, rx, 115200);  //tx, then rx
	_usb_debug = new RawSerial(USBTX,USBRX,115200);

	_sock = tx_sock;
}

void MUXBoard_daemon::Rx_Interrupt(){
    char c;

    while (_bat_mon->readable()) {
        c = _bat_mon->getc();

        response_tmp[_len] = c;
        if (c == '\n'){
            memset((char *)response,0,sizeof(response));
            memcpy((char *)response,(char *)response_tmp,_len);
            _len = 0;
            _flag_rcevData = 1;
            recvInfo();
        }
        else{
            if(_len > 128){
                _len = 0;
            }
            else{
                _len++;
            }
        }
    }    
}

void MUXBoard_daemon::Start() {
	main_thread.start(callback(this, &MUXBoard_daemon::main_worker));
}


void MUXBoard_daemon::main_worker() {

    static uint8_t noResponceCount = 0;

	ThisThread::sleep_for(1000);
    _bat_mon->attach(callback(this, &MUXBoard_daemon::Rx_Interrupt));

	while(1){
        _bat_mon->puts("g current 1\n");
        //u_printf("currentSensor1 = %.3f A\n", I2C_MUX_BOARD_common.current1/1000.0);
        _usb_debug->printf("currentSensor1 = %.3f A\n", I2C_MUX_BOARD_common.current1/1000.0);

        _bat_mon->puts("g current 2\n");
        //u_printf("currentSensor2 = %.3f A\n", I2C_MUX_BOARD_common.current2/1000.0);
        _usb_debug->printf("currentSensor2 = %.3f A\n", I2C_MUX_BOARD_common.current2/1000.0);

        _bat_mon->puts("g tmp 1\n");
        //u_printf("tmpSensor1 = %.2f degC\n", I2C_MUX_BOARD_common.tmp1/100.0);
		_usb_debug->printf("tmpSensor1 = %.2f degC\n", I2C_MUX_BOARD_common.tmp1/100.0);

        _bat_mon->puts("g tmp 2\n");
        //u_printf("tmpSensor2 = %.2f degC\n", I2C_MUX_BOARD_common.tmp2/100.0);
		_usb_debug->printf("tmpSensor2 = %.2f degC\n", I2C_MUX_BOARD_common.tmp2/100.0);
        
        _bat_mon->puts("g bat 1\n");
        _usb_debug->printf("BAT1 V:%d mV, I:%d mA, SOC:%d \n",
           I2C_MUX_BOARD_BattInfo[0].volt,
            I2C_MUX_BOARD_BattInfo[0].current,
           I2C_MUX_BOARD_BattInfo[0].SOC);

        _bat_mon->puts("g bat 5\n");
        _usb_debug->printf("BAT5 V:%d mV, I:%d mA, SOC:%d \n",
           I2C_MUX_BOARD_BattInfo[4].volt,
            I2C_MUX_BOARD_BattInfo[4].current,
           I2C_MUX_BOARD_BattInfo[4].SOC);

        if(_flag_rcevData == 0){
            if(noResponceCount>=3){
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
        _usb_debug->printf("\n");
        ThisThread::sleep_for(1000);
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
    uint8_t ch;

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

    return 0;
}

