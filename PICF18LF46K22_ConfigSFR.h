
# 1 "RealTimeClock.c"

# 26 "C:\Program Files (x86)\Microchip\xc8\v1.44\include\htc.h"
extern const char __xc8_OPTIM_SPEED;

extern double __fpnormalize(double);


# 13 "C:\Program Files (x86)\Microchip\xc8\v1.44\include\xc8debug.h"
#pragma intrinsic(__builtin_software_breakpoint)
extern void __builtin_software_breakpoint(void);

# 50 "C:\Program Files (x86)\Microchip\xc8\v1.44\include\pic18lf46k22.h"
extern volatile unsigned char ANSELA @ 0xF38;

asm("ANSELA equ 0F38h");


typedef union {
struct {
unsigned ANSA0 :1;
unsigned ANSA1 :1;
unsigned ANSA2 :1;
unsigned ANSA3 :1;
unsigned :1;
unsigned ANSA5 :1;
};
} ANSELAbits_t;
extern volatile ANSELAbits_t ANSELAbits @ 0xF38;

# 95
extern volatile unsigned char ANSELB @ 0xF39;

asm("ANSELB equ 0F39h");


typedef union {
struct {
unsigned ANSB0 :1;
unsigned ANSB1 :1;
unsigned ANSB2 :1;
unsigned ANSB3 :1;
unsigned ANSB4 :1;
unsigned ANSB5 :1;
};
} ANSELBbits_t;
extern volatile ANSELBbits_t ANSELBbits @ 0xF39;

# 145
extern volatile unsigned char ANSELC @ 0xF3A;

asm("ANSELC equ 0F3Ah");


typedef union {
struct {
unsigned :2;
unsigned ANSC2 :1;
unsigned ANSC3 :1;
unsigned ANSC4 :1;
unsigned ANSC5 :1;
unsigned ANSC6 :1;
unsigned ANSC7 :1;
};
} ANSELCbits_t;
extern volatile ANSELCbits_t ANSELCbits @ 0xF3A;

# 196
extern volatile unsigned char ANSELD @ 0xF3B;

asm("ANSELD equ 0F3Bh");


typedef union {
struct {
unsigned ANSD0 :1;
unsigned ANSD1 :1;
unsigned ANSD2 :1;
unsigned ANSD3 :1;
unsigned ANSD4 :1;
unsigned ANSD5 :1;
unsigned ANSD6 :1;
unsigned ANSD7 :1;
};
} ANSELDbits_t;
extern volatile ANSELDbits_t ANSELDbits @ 0xF3B;

# 258
extern volatile unsigned char ANSELE @ 0xF3C;

asm("ANSELE equ 0F3Ch");


typedef union {
struct {
unsigned ANSE0 :1;
unsigned ANSE1 :1;
unsigned ANSE2 :1;
};
} ANSELEbits_t;
extern volatile ANSELEbits_t ANSELEbits @ 0xF3C;

# 290
extern volatile unsigned char PMD2 @ 0xF3D;

asm("PMD2 equ 0F3Dh");


typedef union {
struct {
unsigned ADCMD :1;
unsigned CMP1MD :1;
unsigned CMP2MD :1;
unsigned CTMUMD :1;
};
} PMD2bits_t;
extern volatile PMD2bits_t PMD2bits @ 0xF3D;

# 328
extern volatile unsigned char PMD1 @ 0xF3E;

asm("PMD1 equ 0F3Eh");


typedef union {
struct {
unsigned CCP1MD :1;
unsigned CCP2MD :1;
unsigned CCP3MD :1;
unsigned CCP4MD :1;
unsigned CCP5MD :1;
unsigned :1;
unsigned MSSP1MD :1;
unsigned MSSP2MD :1;
};
struct {
unsigned EMBMD :1;
};
} PMD1bits_t;
extern volatile PMD1bits_t PMD1bits @ 0xF3E;

# 393
extern volatile unsigned char PMD0 @ 0xF3F;

asm("PMD0 equ 0F3Fh");


typedef union {
struct {
unsigned TMR1MD :1;
unsigned TMR2MD :1;
unsigned TMR3MD :1;
unsigned TMR4MD :1;
unsigned TMR5MD :1;
unsigned TMR6MD :1;
unsigned UART1MD :1;
unsigned UART2MD :1;
};
struct {
unsigned :1;
unsigned SPI1MD :1;
unsigned SPI2MD :1;
};
} PMD0bits_t;
extern volatile PMD0bits_t PMD0bits @ 0xF3F;

# 470
extern volatile unsigned char VREFCON2 @ 0xF40;

asm("VREFCON2 equ 0F40h");


extern volatile unsigned char DACCON1 @ 0xF40;

asm("DACCON1 equ 0F40h");


typedef union {
struct {
unsigned DACR :5;
};
struct {
unsigned DACR0 :1;
unsigned DACR1 :1;
unsigned DACR2 :1;
unsigned DACR3 :1;
unsigned DACR4 :1;
};
} VREFCON2bits_t;
extern volatile VREFCON2bits_t VREFCON2bits @ 0xF40;

# 525
typedef union {
struct {
unsigned DACR :5;
};
struct {
unsigned DACR0 :1;
unsigned DACR1 :1;
unsigned DACR2 :1;
unsigned DACR3 :1;
unsigned DACR4 :1;
};
} DACCON1bits_t;
extern volatile DACCON1bits_t DACCON1bits @ 0xF40;

# 572
extern volatile unsigned char VREFCON1 @ 0xF41;

asm("VREFCON1 equ 0F41h");


extern volatile unsigned char DACCON0 @ 0xF41;

asm("DACCON0 equ 0F41h");


typedef union {
struct {
unsigned DACNSS :1;
unsigned :1;
unsigned DACPSS :2;
unsigned :1;
unsigned DACOE :1;
unsigned DACLPS :1;
unsigned DACEN :1;
};
struct {
unsigned :2;
unsigned DACPSS0 :1;
unsigned DACPSS1 :1;
};
} VREFCON1bits_t;
extern volatile VREFCON1bits_t VREFCON1bits @ 0xF41;

# 636
typedef union {
struct {
unsigned DACNSS :1;
unsigned :1;
unsigned DACPSS :2;
unsigned :1;
unsigned DACOE :1;
unsigned DACLPS :1;
unsigned DACEN :1;
};
struct {
unsigned :2;
unsigned DACPSS0 :1;
unsigned DACPSS1 :1;
};
} DACCON0bits_t;
extern volatile DACCON0bits_t DACCON0bits @ 0xF41;

# 692
extern volatile unsigned char VREFCON0 @ 0xF42;

asm("VREFCON0 equ 0F42h");


extern volatile unsigned char FVRCON @ 0xF42;

asm("FVRCON equ 0F42h");


typedef union {
struct {
unsigned :4;
unsigned FVRS :2;
unsigned FVRST :1;
unsigned FVREN :1;
};
struct {
unsigned :4;
unsigned FVRS0 :1;
unsigned FVRS1 :1;
};
} VREFCON0bits_t;
extern volatile VREFCON0bits_t VREFCON0bits @ 0xF42;

# 743
typedef union {
struct {
unsigned :4;
unsigned FVRS :2;
unsigned FVRST :1;
unsigned FVREN :1;
};
struct {
unsigned :4;
unsigned FVRS0 :1;
unsigned FVRS1 :1;
};
} FVRCONbits_t;
extern volatile FVRCONbits_t FVRCONbits @ 0xF42;

# 786
extern volatile unsigned char CTMUICON @ 0xF43;

asm("CTMUICON equ 0F43h");


extern volatile unsigned char CTMUICONH @ 0xF43;

asm("CTMUICONH equ 0F43h");


typedef union {
struct {
unsigned IRNG :2;
unsigned ITRIM :6;
};
struct {
unsigned IRNG0 :1;
unsigned IRNG1 :1;
unsigned ITRIM0 :1;
unsigned ITRIM1 :1;
unsigned ITRIM2 :1;
unsigned ITRIM3 :1;
unsigned ITRIM4 :1;
unsigned ITRIM5 :1;
};
} CTMUICONbits_t;
extern volatile CTMUICONbits_t CTMUICONbits @ 0xF43;

# 865
typedef union {
struct {
unsigned IRNG :2;
unsigned ITRIM :6;
};
struct {
unsigned IRNG0 :1;
unsigned IRNG1 :1;
unsigned ITRIM0 :1;
unsigned ITRIM1 :1;
unsigned ITRIM2 :1;
unsigned ITRIM3 :1;
unsigned ITRIM4 :1;
unsigned ITRIM5 :1;
};
} CTMUICONHbits_t;
extern volatile CTMUICONHbits_t CTMUICONHbits @ 0xF43;

# 936
extern volatile unsigned char CTMUCONL @ 0xF44;

asm("CTMUCONL equ 0F44h");


extern volatile unsigned char CTMUCON1 @ 0xF44;

asm("CTMUCON1 equ 0F44h");


typedef union {
struct {
unsigned EDG1STAT :1;
unsigned EDG2STAT :1;
unsigned EDG1SEL :2;
unsigned EDG1POL :1;
unsigned EDG2SEL :2;
unsigned EDG2POL :1;
};
struct {
unsigned :2;
unsigned EDG1SEL0 :1;
unsigned EDG1SEL1 :1;
unsigned :1;
unsigned EDG2SEL0 :1;
unsigned EDG2SEL1 :1;
};
} CTMUCONLbits_t;
extern volatile CTMUCONLbits_t CTMUCONLbits @ 0xF44;

# 1017
typedef union {
struct {
unsigned EDG1STAT :1;
unsigned EDG2STAT 