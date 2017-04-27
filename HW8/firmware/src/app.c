/*******************************************************************************
  MPLAB Harmony Application Source File
  
  Company:
    Microchip Technology Inc.
  
  File Name:
    app.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It 
    implements the logic of the application's state machine and it may call 
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files 
// *****************************************************************************
// *****************************************************************************

#include "app.h"
#include "ILI9163C.h"
#include "i2c_master_noint.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    This structure should be initialized by the APP_Initialize function.
    
    Application strings and buffers are be defined outside this structure.
 */

APP_DATA appData;

// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Functions
// *****************************************************************************
// *****************************************************************************

/* TODO:  Add any necessary callback functions.
 */

// *****************************************************************************
// *****************************************************************************
// Section: Application Local Functions
// *****************************************************************************
// *****************************************************************************


/* TODO:  Add any necessary local functions.
 */

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

void drawBarX(signed short XL_X) {
    int BarLength;
    BarLength = 50 * (((float) XL_X) / 16384);
    int i, j;
    if (BarLength >= 0) {
        for (j = 3; j < 52; j++) {
            for (i = -2; i < 3; i++) {
                LCD_drawPixel(64 - j, 64 + i, WHITE);
            }
        }

        for (j = CurrentLength; j >= BarLength; j--) {
            for (i = -2; i < 3; i++) {
                LCD_drawPixel(64 + j, 64 + i, WHITE);
            }
        }
        for (j = CurrentLength; j <= BarLength; j++) {
            for (i = -2; i < 3; i++) {
                LCD_drawPixel(64 + j, 64 + i, MAGENTA);
            }
        }
    } else {
        for (j = 3; j < 52; j++) {
            for (i = -2; i < 3; i++) {
                LCD_drawPixel(64 + j, 64 + i, WHITE);
            }
        }
        for (j = CurrentLength; j <= BarLength; j++) {
            for (i = -2; i < 3; i++) {
                LCD_drawPixel(64 + j, 64 + i, WHITE);
            }
        }
        for (j = CurrentLength; j >= BarLength; j--) {
            for (i = -2; i < 3; i++) {
                LCD_drawPixel(64 + j, 64 + i, MAGENTA);
            }
        }
    }

    CurrentLength = BarLength;
}

void drawBarY(signed short XL_Y) {
    int BarLength;
    BarLength = 50 * (((float) XL_Y) / 16384);
    int i, j;
    if (BarLength >= 0) {
        for (j = 3; j < 52; j++) {
            for (i = -2; i < 3; i++) {
                LCD_drawPixel(64 + i, 64 - j, WHITE);
            }
        }

        for (j = CurrentLengthY; j >= BarLength; j--) {
            for (i = -2; i < 3; i++) {
                LCD_drawPixel(64 + i, 64 + j, WHITE);
            }
        }
        for (j = CurrentLengthY; j <= BarLength; j++) {
            for (i = -2; i < 3; i++) {
                LCD_drawPixel(64 + i, 64 + j, BLACK);
            }
        }
    } else {
        for (j = 3; j < 52; j++) {
            for (i = -2; i < 3; i++) {
                LCD_drawPixel(64 + i, 64 + j, WHITE);
            }
        }
        for (j = CurrentLengthY; j <= BarLength; j++) {
            for (i = -2; i < 3; i++) {
                LCD_drawPixel(64 + i, 64 + j, WHITE);
            }
        }
        for (j = CurrentLengthY; j >= BarLength; j--) {
            for (i = -2; i < 3; i++) {
                LCD_drawPixel(64 + i, 64 + j, BLACK);
            }
        }
    }

    CurrentLengthY = BarLength;
}


// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Initialize(void) {
    /* Place the App state machine in its initial state. */
    appData.state = APP_STATE_INIT;
    i2c_master_setup();
    IMU_Init();
    SPI1_init();
    LCD_init();
    LCD_clearScreen(WHITE);


    /* TODO: Initialize your application's state machine and other
     * parameters.
     */
}

/******************************************************************************
  Function:
    void APP_Tasks ( void )

  Remarks:
    See prototype in app.h.
 */

void APP_Tasks(void) {

    /* Check the application's current state. */
    switch (appData.state) {
            /* Application's initial state. */
        case APP_STATE_INIT:
        {
            bool appInitialized = true;


            if (appInitialized) {

                appData.state = APP_STATE_SERVICE_TASKS;
            }
            break;
        }

        case APP_STATE_SERVICE_TASKS:
        {
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

                while (_CP0_GET_COUNT() < 4790000) {
                    ;
                }

            }
            break;
        }

            /* TODO: implement your application state machine.*/


            /* The default state should never be executed. */
        default:
        {
            /* TODO: Handle error in application's state machine. */
            break;
        }
    }
}



/*******************************************************************************
 End of File
 */
