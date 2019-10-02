//Author: Jiri Konecny 
// PIC18LF46K22 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config FOSC = INTIO67   // Oscillator Selection bits (Internal oscillator block)
#pragma config PLLCFG = OFF     // 4X PLL Enable (Oscillator used directly)
#pragma config PRICLKEN = ON    // Primary clock enable bit (Primary clock is always enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power-up Timer Enable bit (Power up timer disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled and controlled by software (SBOREN is enabled))
#pragma config BORV = 190       // Brown Out Reset Voltage bits (VBOR set to 1.90 V nominal)

// CONFIG2H
#pragma config WDTEN = ON       // Watchdog Timer Enable bits (WDT is always enabled. SWDTEN bit has no effect)
#pragma config WDTPS = 16384    // Watchdog Timer Postscale Select bits (1:16384)

// CONFIG3H
#pragma config CCP2MX = PORTC1  // CCP2 MUX bit (CCP2 input/output is multiplexed with RC1)
#pragma config PBADEN = ON      // PORTB A/D Enable bit (PORTB<5:0> pins are configured as analog input channels on Reset)
#pragma config CCP3MX = PORTB5  // P3A/CCP3 Mux bit (P3A/CCP3 input/output is multiplexed with RB5)
#pragma config HFOFST = ON      // HFINTOSC Fast Start-up (HFINTOSC output and ready status are not delayed by the oscillator stable status)
#pragma config T3CMX = PORTC0   // Timer3 Clock input mux bit (T3CKI is on RC0)
#pragma config P2BMX = PORTD2   // ECCP2 B output mux bit (P2B is on RD2)
#pragma config MCLRE = EXTMCLR  // MCLR Pin Enable bit (MCLR pin enabled, RE3 input pin disabled)

// CONFIG4L
#pragma config STVREN = ON      // Stack Full/Underflow Reset Enable bit (Stack full/underflow will cause Reset)
#pragma config LVP = ON         // Single-Supply ICSP Enable bit (Single-Supply ICSP enabled if MCLRE is also 1)
#pragma config XINST = OFF      // Extended Instruction Set Enable bit (Instruction set extension and Indexed Addressing mode disabled (Legacy mode))

// CONFIG5L
#pragma config CP0 = OFF        // Code Protection Block 0 (Block 0 (000800-003FFFh) not code-protected)
#pragma config CP1 = OFF        // Code Protection Block 1 (Block 1 (004000-007FFFh) not code-protected)
#pragma config CP2 = OFF        // Code Protection Block 2 (Block 2 (008000-00BFFFh) not code-protected)
#pragma config CP3 = OFF        // Code Protection Block 3 (Block 3 (00C000-00FFFFh) not code-protected)

// CONFIG5H
#pragma config CPB = OFF        // Boot Block Code Protection bit (Boot block (000000-0007FFh) not code-protected)
#pragma config CPD = OFF        // Data EEPROM Code Protection bit (Data EEPROM not code-protected)

// CONFIG6L
#pragma config WRT0 = OFF       // Write Protection Block 0 (Block 0 (000800-003FFFh) not write-protected)
#pragma config WRT1 = OFF       // Write Protection Block 1 (Block 1 (004000-007FFFh) not write-protected)
#pragma config WRT2 = OFF       // Write Protection Block 2 (Block 2 (008000-00BFFFh) not write-protected)
#pragma config WRT3 = OFF       // Write Protection Block 3 (Block 3 (00C000-00FFFFh) not write-protected)

// CONFIG6H
#pragma config WRTC = OFF       // Configuration Register Write Protection bit (Configuration registers (300000-3000FFh) not write-protected)
#pragma config WRTB = OFF       // Boot Block Write Protection bit (Boot Block (000000-0007FFh) not write-protected)
#pragma config WRTD = OFF       // Data EEPROM Write Protection bit (Data EEPROM not write-protected)

// CONFIG7L
#pragma config EBTR0 = OFF      // Table Read Protection Block 0 (Block 0 (000800-003FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR1 = OFF      // Table Read Protection Block 1 (Block 1 (004000-007FFFh) not protected from table reads executed in other blocks)
#pragma config EBTR2 = OFF      // Table Read Protection Block 2 (Block 2 (008000-00BFFFh) not protected from table reads executed in other blocks)
#pragma config EBTR3 = OFF      // Table Read Protection Block 3 (Block 3 (00C000-00FFFFh) not protected from table reads executed in other blocks)

// CONFIG7H
#pragma config EBTRB = OFF      // Boot Block Table Read Protection bit (Boot Block (000000-0007FFh) not protected from table reads executed in other blocks)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

#include "OneWireLib.h"
#include <xc.h>

#define DMX_A PORTEbits.RE0
#define DMX_B PORTEbits.RE1

#define OneWirePort PORTBbits.RB4

#define demuxSelectSize 4
#define numbersValueSize 4
#define numbersDataSize 10
#define testDataSize 8

#define true 1
#define false 0

#define _XTAL_FREQ 4000000

enum commands { cmd_convert=0x44, cmd_copytoeeprom=0x48, cmd_skiprom=0xCC, cmd_readpowersuply=0xB4,
cmd_8bitfamilycode=0x28, cmd_readROM=0x33, cmd_matchROM=0x55, cmd_writetoSRC=0x4E, cmd_readfromSRC=0xBE };

char demuxSelect[]={0b00000000, 0b00000001, 0b00000010, 0b00000011};
char demuxData[]={0xFF, 0xFF, 0xFF, 0xFF};

char numbersData[]={ 0b11000000 /*0*/ ,0b11111001 /*1*/, 0b10100100  /*2*/,0b10110000 /*3*/,0b10011001 /*4*/,0b10010010 /*5*/,0b10000010 /*6*/,0b11111000 /*7*/,0b10000000 /*8*/,0b10010000 /*9*/};
int numbersValue[]={0,0,0,0};

char charsetData[]={0b10111111 /*-*/};

char selectDot[]={0b11111111, 0b01111111};
char dotSelector[]={false,true,false,false};
int indexer=0;

unsigned int temperatureData, senzorPwrSupply, senzorROM;
unsigned char familyName;

bit readyFlag=false;

void Init(){
    OSCCON=0b01011100; //osc setting, 4 MHz, internal by FOSCH
    INTCON=0b11100000; //int setting, global,pir, timer0
    
    INTCON2bits.NOT_RBPU=1; //potrB pullup enable
    //WPUBbits.WPUB4=1;
    
    T0CON=0b01000011; //timer0 setting, PS 1/16 * 2 (FOST/4) 128us
    
    ANSELD=0;
    TRISD=0;
        
    ANSELEbits.ANSE0=0;
    ANSELEbits.ANSE1=0;
    TRISEbits.RE0=0;
    TRISEbits.RE1=0;
    
    ANSELBbits.ANSB4=0;
    TRISBbits.RB4=1;   
}

void ClearDevice(){
    PORTD=0xFF;
    DMX_A=DMX_B=0;
    indexer=0;
    readyFlag=false;
    
    senzorPwrSupply=0;
    temperatureData=0;
    familyName=0;
    senzorROM=0;
}
void ClearDisplay(){
    for(int i=0;i<demuxSelectSize;i++){
        PORTE=demuxSelect[i]; 
        __delay_ms(1);
        PORTD=demuxData[i]=0xFF;
    }    
}

void TestDevice(){
    for(int i=0;i<demuxSelectSize;i++){
        PORTD=0xFF;
        __delay_ms(1);
        
        PORTE=demuxSelect[i];
        PORTD=0x00;
        __delay_ms(500);
    }
    for(int i=0;i<demuxSelectSize;i++){
        PORTD=0xFF;
        __delay_ms(1);
        PORTE=demuxSelect[i];
       
        PORTD=0b01111111;
        for(int j=0;j<testDataSize;j++){
            PORTD<<=1;
            __delay_ms(100);
        }
        PORTD=0x00;
        __delay_ms(250);
    }
    PORTD=0xFF;
}
void parseValues(int F, int S){
        numbersValue[2]=S/10; 
        numbersValue[3]=S%10;
        numbersValue[0]=F/10; 
        numbersValue[1]=F%10;      
}
void temeperatureDataConvertor(int data){
    int H=0,L=0;
    H= data & 0x07F0;
    L= data & 0x000F;
    parseValues(H,L);    
}
void readTemperature(){
    //T0CONbits.TMR0ON=0;     
    ClearDisplay();   
    
    OWWriteByte(cmd_matchROM);
    for(int i=1;i<=4;i++){
        OWWriteByte((senzorROM>>(i*8)&0x00000000000000FF));
    }
    
            
    temperatureData=0;
    OWWriteByte(cmd_convert); //Conver cmd
    while(!OWReadBit()){
        __delay_us(1);
    }
    OWWriteByte(cmd_readfromSRC); //Read memory 
    
    temperatureData|=OWReadByte();
    temperatureData<<=8;
    temperatureData|=OWReadByte();
    temeperatureDataConvertor(temperatureData);
    
    //T0CONbits.TMR0ON=1;
}

void readPowerSuply(){
    OWWriteByte(cmd_skiprom); //Skip-EEPROM
    OWWriteByte(cmd_readpowersuply); //Read power supply
    senzorPwrSupply==OWReadByte();
    __delay_ms(1);
}
void readROM(){
    
    OWWriteByte(cmd_readROM); //Read ROM 
    senzorROM|=OWReadByte();
    senzorROM<<=8;
    senzorROM|=OWReadByte();
    senzorROM<<=8;
    senzorROM|=OWReadByte();
    senzorROM<<=8;
    senzorROM|=OWReadByte();
    
    OWWriteByte(cmd_8bitfamilycode); //Read 8 bit family name
    familyName=OWReadByte();
    __delay_ms(1);
}
int configTemperatureSenzor(){
    if(OWTouchReset())return 0;
    readROM();
    readPowerSuply();
    return 1;
}

void main(void) {
    Init();
    ClearDevice();
    TestDevice();
    
    
    //OneWire setting
    SetSpeed(true);
    portSetting(OneWirePort,TRISBbits.RB4);
    
    for(int i=0;i<demuxSelectSize;i++){
        demuxData[i]=charsetData[0];
    }
    while(1){
        asm("NOP");
        asm("CLRWDT");
        
        while(!readyFlag){
            readyFlag=configTemperatureSenzor();
            T0CONbits.TMR0ON=1;
            __delay_ms(1000);
            T0CONbits.TMR0ON=0;
        }
        T0CONbits.TMR0ON=0;
        readTemperature();  
        T0CONbits.TMR0ON=1;
        __delay_ms(500);
    }
}

void interrupt IRS(void){
    if(INTCONbits.TMR0IF){
        INTCONbits.TMR0IF=0;
        
        
        for(int i=0;i<demuxSelectSize;i++){
            if(demuxData[i]==charsetData[0])continue;
            
            if(numbersValue[i]>=numbersDataSize)numbersValue[i]=numbersDataSize-1;
            if(numbersValue[i]<0)numbersValue[i]=0;
            
            demuxData[i]=numbersData[numbersValue[i]];
            demuxData[i]&=selectDot[dotSelector[i]];
        }  
        
        PORTD=0xFF;
        PORTE=demuxSelect[indexer];
        __delay_us(100);
        
        PORTD=demuxData[indexer];           
         
         indexer++;
         if(indexer>=demuxSelectSize)indexer=0;
     }
}
