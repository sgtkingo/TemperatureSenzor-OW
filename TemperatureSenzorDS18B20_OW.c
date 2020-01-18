//Author: Jiri Konecny 

#include "PICF18LF46K22_ConfigSFR.h"
#include "DS18B20_OneDevice_FunctionLib.h"
#include "LCD_Engine4bit.h"

char TEMP_CELSIA=0;
char TEMP_MILICELSIA=0;

#define NumberBuffer_Size 4

char NumberBuffer[N]={};


void Clear_Buffer(){
    for(char i=0;i<NumberBuffer_Size;i++){
        NumberBuffer[i]=0;
    }
}

void Update_LCD(){
    Clear_Buffer();
    
    LCDClearLine(0);
    LCDGoto(0,0);
    LCDPutStr("Temp: ");
    convertNumber(TEMP_CELSIA, NumberBuffer,NumberBuffer_Size);
    LCDPutStr(NumberBuffer);
    LCDPutStr(".");
    convertNumber(TEMP_MILICELSIA, NumberBuffer,NumberBuffer_Size);
    LCDPutStr(NumberBuffer);
    LCDPutStr(" C");
}

void InitDevice(){
    OSCCON=0b01111100; //osc setting, 16 MHz, internal by FOSCH
    
    ANSELD=0;
    TRISD=0;
    
    Init_DS18B20();
    __delay_ms(10);
    LCD_Initialize();
    __delay_ms(10);
}

void ClearDevice(){
    PORTD=0x00;
    LCD_Clear();
}

void TestDevice(){
    LCDPutStr("Preparing...");
    delay_ms(1000);
    LCDClearLine(0);
    LCDGoto(5,0);
    LCDPutStr("READY!");
    delay_ms(2500);
    
    LCD_Clear();
    Clear_Buffer();
}
void main(void) {
    InitDevice();
    ClearDevice();
    TestDevice();
    NOP();

    while(configTemperatureSenzor(res_12b));   
    while(1){
        readTemperature(); 
        asm("NOP");
        TEMP_CELSIA=getTempCelsia();
        TEMP_MILICELSIA=getTempMiliCelsia();
        __delay_ms(100);
        Update_LCD();
        NOP();
    }
}
