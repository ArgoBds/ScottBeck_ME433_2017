#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
#include<math.h> //Math Functions

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

#define CS LATAbits.LATA4

static volatile float SinWaveform[100];
static volatile char TriWaveform[200];

void makeWaveforms(){
    int i;
    for(i = 0; i<100; i++){
        SinWaveform[i] = (1.65*sin(2*M_PI*i/100)+1.65)*255/3.3;
    }
    for(i = 0; i<200; i++){
        TriWaveform[i] = i*256/200;
    }
}
void spiInit(){
    
    SPI1CON = 0;
    SPI1BUF;
    SPI1BRG = 0x1; 
    SPI1STATbits.SPIROV = 0;
    SPI1CONbits.MSTEN = 1;
    SPI1CONbits.CKE = 1;
    SPI1CONbits.ON = 1;
    
    TRISAbits.TRISA4 = 0;
    CS = 1;
    RPB8Rbits.RPB8R = 0b0011;
    SDI1Rbits.SDI1R = 0b0000;
    
}
unsigned char spi_io(unsigned char o){
    SPI1BUF = o;
    while(!SPI1STATbits.SPIRBF){
    
    }
    return SPI1BUF;
}

void sendVoltage(unsigned char channel, unsigned char data){
    CS = 0;
    unsigned char in1 = ((channel << 7) | (data >> 4))|(0b00110000);
    unsigned char in2 = (data << 4);
    spi_io(in1);
    spi_io(in2);
    CS = 1;
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
    
    spiInit();
    makeWaveforms();
    TRISBbits.TRISB4 = 1;
   _CP0_SET_COUNT(0);
    __builtin_enable_interrupts();
    
    int i = 0;
    int j = 0;
    while(1) {
        if(_CP0_GET_COUNT() >= 23999){
            unsigned char sendData = (unsigned char)SinWaveform[i];
            sendVoltage(0,sendData);
            sendVoltage(1,TriWaveform[j]);
            i++;
            j++;
            if(i == 100){
                i = 0;
            }
            if(j == 200){
                j = 0;
            }
            _CP0_SET_COUNT(0);
        }
    
    }
}