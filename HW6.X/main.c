#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<math.h> //Math Functions
#include "ILI9163C.h"
#include<stdio.h>

// DEVCFG0
#pragma config DEBUG = OFF // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // free up secondary osc pins
#pragma config FPBDIV = DIV_1 // divide CPU freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1048576 // slowest wdt
#pragma config WINDIS = OFF // no wdt window
#pragma config FWDTEN = OFF // wdt off by default
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the CPU clock to 48MHz
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiply by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 00000000 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module


void LCD_drawChar(unsigned short x0, unsigned short y0, unsigned char letter, unsigned short color){
    unsigned char index;
    index = letter - 0x20;
    int i,j;
    for(i = 0; i<5;i++){
        if(x0+i<128){
            for(j=0;j<8;j++){
                if(y0+j<128){
                    if((ASCII[index][i]>>j)&0b1){
                        LCD_drawPixel(x0+i,y0+j,color);
                    }
                    else{
                        LCD_drawPixel(x0+i,y0+j,WHITE);
                    }
                    
                }
            }
        }
    }
}

void LCD_drawString(unsigned short x0, unsigned short y0, char* text, unsigned short color){
    int char_index = 0;
    while(text[char_index]){
        LCD_drawChar(x0,y0,text[char_index],color);
        x0 = x0+6;
        char_index++;
    }
}

void LCD_barUpdate(unsigned short x0, unsigned short y0, unsigned short currLength, unsigned short color){
    int i;
    for(i=0;i<5;i++){
        LCD_drawPixel(x0+currLength,y0+i,color);
    }
}

int main() {

    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here
    TRISAbits.TRISA4 = 0;
    
    SPI1_init();
    LCD_init();
    LCD_clearScreen(WHITE);
    
 
    __builtin_enable_interrupts();
    char message[17];
    while(1)
    {
        int i;
        for(i=0;i<101;i++){
            _CP0_SET_COUNT(0);
            sprintf(message,"Hello World %d!  ",i);
            LCD_drawString(28,32,message,BLACK);
            LCD_barUpdate(13,50,i,RED);
            while(_CP0_GET_COUNT()<4799999){
                ;
            }
        }
        LCD_clearScreen(WHITE);

    }
}