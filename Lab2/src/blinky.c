//*****************************************************************************
//
// project0.c - Example to demonstrate minimal TivaWare setup
//
// Copyright (c) 2012-2020 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 2.2.0.295 of the EK-TM4C1294XL Firmware Package.
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "drivers/buttons.h"
#include "drivers/pinout.h"
#include "stdio.h"

//*****************************************************************************
//
// Define pin to LED mapping.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Project Zero (project0)</h1>
//!
//! This example demonstrates the use of TivaWare to setup the clocks and
//! toggle GPIO pins to make the LED blink. This is a good place to start
//! understanding your launchpad and the tools that can be used to program it.
//
//*****************************************************************************

#define USER_LED1  GPIO_PIN_0
#define USER_LED2  GPIO_PIN_1
#define USER_BUTTON_SW1 GPIO_PIN_0

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// The value is true when the INT_GPIOJ interrupt was processed. (Maybe replace to bool type)
//
//*****************************************************************************
volatile uint32_t g_ui32GPIOj;

//*****************************************************************************
//
// This is the handler for INT_GPIOJ.
//
//*****************************************************************************
void
IntGPIOj(void)
{
  //O que estou pensando, é de colocar a verificação do botao pressionado aqui dentro, 
  //se foi pressionado, ai sim essa flag vai para um (podemos renomear o nome dessa var tbm)
  g_ui32GPIOj = 1; //true
  
}

//*******************************************************************
//
//Function for init the interrupt for PORT J
//
//*******************************************************************
void initInterrupt()
{
    //
    // Configure the device pins.
    //
    PinoutSet(false, false);  //não sei o que essa função faz (vem do arquivo pinout.c/pinout/h na pasta drivers)
    IntMasterEnable(); //Enable the Master key of interrupt.
    IntEnable(INT_GPIOJ);
    IntPrioritySet(INT_GPIOJ, 0x00);
}

//*****************************************************************************
//
// Main 'C' Language entry point.  Toggle an LED using TivaWare.
//
//*****************************************************************************
int
main(void)
{
    uint32_t ui32SysClock;

    //
    // Run from the PLL at 120 MHz.
    // Note: SYSCTL_CFG_VCO_240 is a new setting provided in TivaWare 2.2.x and
    // later to better reflect the actual VCO speed due to SYSCTL#22.
    //
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                       SYSCTL_OSC_MAIN |
                                       SYSCTL_USE_PLL |
                                       SYSCTL_CFG_VCO_240), 120000000);

    //
    // Enable and wait for the port to be ready for access
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION | SYSCTL_PERIPH_GPIOJ);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION| SYSCTL_PERIPH_GPIOJ))
    {
    }
    
    //
    // Configure the GPIO port for the LED operation.
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, USER_LED2);
    
    
    //
    // Initialize the button driver.
    //
    ButtonsInit();
    
    initInterrupt();
    
    //
    // Loop Forever
    //
    while(1)
    {
        //
        // Turn on the LED
        //
        GPIOPinWrite(GPIO_PORTN_BASE, (USER_LED2), USER_LED2);

        //
        // Delay for a bit
        //
        SysCtlDelay(ui32SysClock/6);

        //
        // Turn on the LED
        //
        GPIOPinWrite(GPIO_PORTN_BASE, (USER_LED2), 0);
 
        //
        // Delay for a bit
        //
        SysCtlDelay(ui32SysClock/6);
        
        //int32_t buttonPinValue = GPIOPinRead(GPIO_PORTJ_AHB_BASE, USER_BUTTON_SW1);
        //if(buttonPinValue == 0) {
        //  printf("apertou o botao bobao\n");
        //}
        
        uint8_t ui8Buttons;
        uint8_t ui8ButtonsChanged;

        //
        // Grab the current, debounced state of the buttons.
        //
        ui8Buttons = ButtonsPoll(&ui8ButtonsChanged, 0);

        //
        // If the USR_SW1 button has been pressed, and was previously not pressed,
        // start the process of changing the behavior of the JTAG pins.
        //
        if(BUTTON_PRESSED(USR_SW1, ui8Buttons, ui8ButtonsChanged))
        {
          printf("ALOU\n");
        }
        
        if(g_ui32GPIOj == 1) {
          g_ui32GPIOj = 0;
          printf("oi interrupção\n");
        }
        
    }
}
