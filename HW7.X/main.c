#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<math.h> //Math Functions
#include "i2c_master_noint.h"
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

#define SLAVE_ADDR 0b01101011

static volatile int CurrentLength;
static volatile int CurrentLengthY;

void LCD_drawChar(unsigned short x0, unsigned short y0, unsigned char letter, unsigned short color) {
    unsigned char index;
    index = letter - 0x20;
    int i, j;
    for (i = 0; i < 5; i++) {
        if (x0 + i < 128) {
            for (j = 0; j < 8; j++) {
                if (y0 + j < 128) {
                    if ((ASCII[index][i] >> j)&0b1) {
                        LCD_drawPixel(x0 + i, y0 + j, color);
                    } else {
                        LCD_drawPixel(x0 + i, y0 + j, WHITE);
                    }

                }
            }
        }
    }
}

void LCD_drawString(unsigned short x0, unsigned short y0, char* text, unsigned short color) {
    int char_index = 0;
    while (text[char_index]) {
        LCD_drawChar(x0, y0, text[char_index], color);
        x0 = x0 + 6;
        char_index++;
    }
}

void IMU_Init() {
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1);
    i2c_master_send(0x10);
    i2c_master_send(0x82);
    i2c_master_stop();

    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1);
    i2c_master_send(0x11);
    i2c_master_send(0x88);
    i2c_master_stop();
}

void I2C_read_multiple(unsigned char reg, unsigned char * data, int length) {
    i2c_master_start();
    i2c_master_send(SLAVE_ADDR << 1);
    i2c_master_send(reg);
    i2c_master_restart();
    i2c_master_send((SLAVE_ADDR << 1) | 1);
    int i;
    for (i = 0; i < (length - 1); i++) {
        *(data + i) = i2c_master_recv();
        i2c_master_ack(0);
    }
    *(data + (length - 1)) = i2c_master_recv();
    i2c_master_ack(1);
    i2c_master_stop();
}

void chars_2_ints(unsigned char* data, signed short* values) {
    int i;
    for (i = 0; i < 7; i++) {
        *(values + i) = ((data[2 * i + 1] << 8) | data[2 * i]);
    }
}

void drawBarX(signed short XL_X){
    int BarLength;
    BarLength = 50*(((float)XL_X)/16384);
    int i,j;
    if(BarLength>=0){
        for(j=3;j<52;j++){
            for(i=-2;i<3;i++){
                LCD_drawPixel(64-j,64+i,WHITE);
            }
        }
        
        for(j=CurrentLength;j>=BarLength;j--){
            for(i=-2;i<3;i++){
                LCD_drawPixel(64+j,64+i,WHITE);
            }
        }
        for(j=CurrentLength;j<=BarLength;j++){
            for(i=-2;i<3;i++){
                LCD_drawPixel(64+j,64+i,MAGENTA);
            }
        }
    }
    else{
        for(j=3;j<52;j++){
            for(i=-2;i<3;i++){
                LCD_drawPixel(64+j,64+i,WHITE);
            }
        }
        for(j=CurrentLength;j<=BarLength;j++){
            for(i=-2;i<3;i++){
                LCD_drawPixel(64+j,64+i,WHITE);
            }
        }
        for(j=CurrentLength;j>=BarLength;j--){
            for(i=-2;i<3;i++){
                LCD_drawPixel(64+j,64+i,MAGENTA);
            }
        }
    }
 
    CurrentLength = BarLength;
}

void drawBarY(signed short XL_Y){
    int BarLength;
    BarLength = 50*(((float)XL_Y)/16384);
    int i,j;
    if(BarLength>=0){
        for(j=3;j<52;j++){
            for(i=-2;i<3;i++){
                LCD_drawPixel(64+i,64-j,WHITE);
            }
        }
        
        for(j=CurrentLengthY;j>=BarLength;j--){
            for(i=-2;i<3;i++){
                LCD_drawPixel(64+i,64+j,WHITE);
            }
        }
        for(j=CurrentLengthY;j<=BarLength;j++){
            for(i=-2;i<3;i++){
                LCD_drawPixel(64+i,64+j,BLACK);
            }
        }
    }
    else{
        for(j=3;j<52;j++){
            for(i=-2;i<3;i++){
                LCD_drawPixel(64+i,64+j,WHITE);
            }
        }
        for(j=CurrentLengthY;j<=BarLength;j++){
            for(i=-2;i<3;i++){
                LCD_drawPixel(64+i,64+j,WHITE);
            }
        }
        for(j=CurrentLengthY;j>=BarLength;j--){
            for(i=-2;i<3;i++){
                LCD_drawPixel(64+i,64+j,BLACK);
            }
        }
    }
 
    CurrentLengthY = BarLength;
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
    i2c_master_setup();
    IMU_Init();
    SPI1_init();
    LCD_init();
    LCD_clearScreen(WHITE);


    __builtin_enable_interrupts();
    signed short IMU_values[7];
    unsigned char IMU_data[14];

    CurrentLength = 0;
    CurrentLengthY = 0;
    while (1) {
        _CP0_SET_COUNT(0);
        I2C_read_multiple(0x20, IMU_data, 14);
        chars_2_ints(IMU_data, IMU_values);
        drawBarX(IMU_values[4]);
        drawBarY(IMU_values[5]);
        
        while(_CP0_GET_COUNT()<4790000){
            ;
        }

    }
}