//Author: Jiri Konecny 

#include "PICF18LF46K22_ConfigSFR.h"
#include "DS18B20_OneDevice_FunctionLib.h"


void InitDevice(){
    OSCCON=0b01111100; //osc setting, 16 MHz, internal by FOSCH
    
    ANSELD=0;
    TRISD=0;
    
    Init_DS18B20();
}

void ClearDevice(){
    PORTD=0x00;
}
void main(void) {
    InitDevice();
    ClearDevice();

    while(configTemperatureSenzor(res_12b));   
    while(1){
        readTemperature(); 
        asm("NOP");
        PORTD=getTempCelsia();
        __delay_ms(250);
    }
}
