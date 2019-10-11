//Author: Jiri Konecny 
// PIC18LF46K22 Configuration Bit Settings

// 'C' source line config statements

// CONFIG1H
#pragma config FOSC = INTIO67   // Oscillator Selection bits (Internal oscillator block)
#pragma config PLLCFG = ON      // 4X PLL Enable (Oscillator multiplied by 4)
#pragma config PRICLKEN = ON    // Primary clock enable bit (Primary clock is always enabled)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (Fail-Safe Clock Monitor enabled)
#pragma config IESO = OFF       // Internal/External Oscillator Switchover bit (Oscillator Switchover mode disabled)

// CONFIG2L
#pragma config PWRTEN = OFF     // Power-up Timer Enable bit (Power up timer disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable bits (Brown-out Reset enabled and controlled by software (SBOREN is enabled))
#pragma config BORV = 190       // Brown Out Reset Voltage bits (VBOR set to 1.90 V nominal)

// CONFIG2H
#pragma config WDTEN = OFF      // Watchdog Timer Enable bits (Watch dog timer is always disabled. SWDTEN has no effect.)
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

#define true 1
#define false 0

#define N 8


#define _XTAL_FREQ 16000000 //internal macro for __delay_xs() function

//All CMD must be send in direct communication order: 
//**** 1: INIT (Reset)
//**** 2: ROM CMD (can be SKIP)
//**** 3: FCE CMD (call some internal function)

enum ROM_commands { ROM_searchROM=0xF0, ROM_skipROM=0xCC, ROM_readROM=0x33, 
ROM_matchROM=0x55, ROM_alarmSearch=0xEC};

enum FCE_commands { FCE_convertT=0x44, FCE_writeToSCR=0x4E, FCE_readFromSCR=0xBE,
FCE_copytoSCRtoEEPROM=0x48, FCE_reloadEEPROMDataToSCR=0xB8, FCE_readPowerSupply=0xB4,
FCE_8bitFamilyCode=0x28 };

//FCE_writetoSCR=0x4E command send 3 BYTE! All 3 must be send, in LSB mode! 
//*** FIRST BYTE - > 2 BYTE of SCR (TH)
//*** SECOND BYTE - > 3 BYTE of SCR (TL)
//*** THIRT BYTE - > 4 BYTE of SCR (CONFIG)

//FCE_readFromSCR=0xBE command read all 9 BYTES prom SCR, in LSB mode! 
//*** 0 BYTE - > Temp LSB (0x50) 
//*** 1 BYTE - > Temp MSB (0x05) 
//*** 2 BYTE - > TH or GP (Generel Purpose) Byte 1
//*** 3 BYTE - > TL or GP (Generel Purpose) Byte 2
//*** 4 BYTE - > Config Register
//*** 5 BYTE - > Reserved! (R-O)
//*** 6 BYTE - > Reserved! (R-O)
//*** 7 BYTE - > Reserved! (R-O)
//*** 8 BYTE - > CRC (Generated by ROM 64bit mem and SCR 8BYTE)
//This CMD may be termined by RESET sequence if only some bytes needed

//FCE_copytoSCRtoEEPROM=0x48 command copy 3 defined byte from SCR to EEPROM 
//*** FIRST BYTE - > 2 BYTE of SCR (TH)
//*** SECOND BYTE - > 3 BYTE of SCR (TL)
//*** THIRT BYTE - > 4 BYTE of SCR (CONFIG)

//FCE_readPowerSupply=0xB4 command return information about powering device
//RETURN 0 - Parasite Power
//RETURN 1 - External Power

enum resolutions { res_9b=0b0001111, res_10b=0b0011111, res_11b=0b0101111 , res_12b=0b0111111 };
unsigned int temperatureData, senzorPwrSupply;
unsigned char senzorROMasArray[N]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char familyName, CRC, SN;

bit readyFlag=false;

void Init(){
    OSCCON=0b01111100; //osc setting, 16 MHz, internal by FOSCH
    
    ANSELD=0;
    TRISD=0;
    
    ANSELAbits.ANSA2=0; //Set I/O OW port as Din 
    ANSELCbits.ANSC2=0; //Set I/O OW port as Din 
    TRISCbits.RC2=1; //OW IN
    TRISAbits.RA2=0; //OW OUT
}

void ClearDevice(){
    PORTD=0x00;
    readyFlag=false;
    
    senzorPwrSupply=0;
    temperatureData=0;
    
    familyName=0;
    SN=0;
    CRC=0;
}


void temeperatureDataConvertor(int data){
    int H=0,L=0;
    H= data & 0x07F0;
    L= data & 0x000F;
}
void readTemperature(){
    if(OWTouchReset())return;
    OWWriteByte(ROM_skipROM);  
    
    temperatureData=0;
    OWWriteByte(FCE_convertT); //Conver cmd
    while(!OWReadBit()){
        __delay_us(1);
    }
    
    if(OWTouchReset())return;
    OWWriteByte(ROM_skipROM); 
    
    OWWriteByte(FCE_readFromSCR); //Read memory
    
    temperatureData|=OWReadByte();
    temperatureData<<=8;
    temperatureData|=OWReadByte();
    if(OWTouchReset())return;
    
    temeperatureDataConvertor(temperatureData);
}

void readPowerSuply(){
    if(OWTouchReset())return;
    OWWriteByte(ROM_skipROM); //Skip-EEPROM
    OWWriteByte(FCE_readPowerSupply); //Read power supply
    
    for(int i=0;i<10;i++)senzorPwrSupply=OWReadBit();
}
void readROM(){ 
    if(OWTouchReset())return;
    
    int lastReadByte=0;
    OWWriteByte(ROM_readROM); //Read ROM 
    for(int i=0;i<N;i++){
        lastReadByte=OWReadByte();
        senzorROMasArray[i]=lastReadByte;
    }
    
    
    /*OWWriteByte(FCE_8bitFamilyCode); //Read 8 bit family name
    familyName=OWReadByte();*/
}
void setResultResolution(unsigned char resolution){
    if(OWTouchReset())return;
    
    OWWriteByte(ROM_skipROM); //Skip-EEPROM
    OWWriteByte(FCE_writeToSCR); //Write to SRAM memory 
    OWWriteByte(0x00); //TH byte 
    OWWriteByte(0x00); //TL byte
    OWWriteByte(resolution); //Config resolution 
}
int configTemperatureSenzor(){
    if(OWTouchReset())return 0;
    
    readROM();
    readPowerSuply();
    setResultResolution(res_12b); //Resolution to 12b
    return 1;
}

void main(void) {
    Init();
    ClearDevice();
    InitOneWire();

    while(1){
        //asm("CLRWDT");
        //OWWriteBit(1);
        //OWWriteBit(0);
        //OWReadBit();
        if(!OWTouchReset()){
            PORTD=0x55;
        }
        else PORTD=0x00;
        //__delay_ms(100);
    }
}
