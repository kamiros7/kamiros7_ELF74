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
//new libs for systick
#include "driverlib/systick.h"
//new libs for timer
#include "driverlib/timer.h"

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
volatile uint32_t flagInterruptPortJ;

//*****************************************************************************
//
// The value is used for get the curret value of timer counter.
//
//*****************************************************************************
volatile uint32_t currentTimerValueCounter, flagTimerValueCounter;

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
  currentTimerValueCounter = TimerValueGet(TIMER0_BASE, TIMER_A);
  flagInterruptPortJ = 1; //true 
}

//*******************************************************************
//
//Function for init the interrupt for PORT J
//
//*******************************************************************
void initInterrupt()
{
    //
    //Enables the specified GPIO interrupts
    //
    GPIOIntEnable(GPIO_PORTJ_BASE, GPIO_INT_PIN_0);
  
    //
    //Set the interrupt type in the specfied pin (high_level/low_level), (falling_edge, rising_edge...)
    //
    GPIOIntTypeSet(GPIO_PORTJ_BASE, GPIO_PIN_0 , GPIO_LOW_LEVEL);
  
    IntEnable(INT_GPIOJ);
    IntPrioritySet(INT_GPIOJ, 0x00);

    IntMasterEnable(); //Enable the Master key of interrupt.
}

//*******************************************************************
//
//Function that will called for the interrupt with Timer
//
//*******************************************************************
void timerInterrupt()
{
  TimerIntClear(TIMER0_BASE, (TIMER_TIMA_TIMEOUT|TIMER_TIMB_TIMEOUT));
  flagTimerValueCounter = 1;
}

//*******************************************************************
//
//Function for init the Timer API
//
//*******************************************************************
void initTimer(void) 
{
  //
  // Enable the Timer0 peripheral
  //
  SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  //
  // Wait for the Timer0 module to be ready.
  //
  while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0))
  {
  }
  //
  // Configure TimerA as a full-width one-shot timer.
  //
  TimerConfigure(TIMER0_BASE, TIMER_CFG_ONE_SHOT);
  
  //
  // Set the count time for the the one-shot timer (TimerA).
  //
  uint32_t counter = 120000000*3;
  TimerLoadSet(TIMER0_BASE, TIMER_A, counter);
  
  //
  //Configure interrupt handler for timer
  //
  TimerIntRegister(TIMER0_BASE, TIMER_BOTH, &timerInterrupt);
  
  TimerIntEnable(TIMER0_BASE, (TIMER_TIMA_TIMEOUT|TIMER_TIMB_TIMEOUT));
  
}

//*****************************************************************************
//
// Function for convert counter.
//
//*****************************************************************************
uint32_t convertToMiliSeconds(uint32_t counter)
{
  uint32_t time;
  time = ((float)1/120000) * counter;
  return time;
}
  

//*****************************************************************************
//
// Main 'C' Language entry point.
//
//*****************************************************************************
int
main(void)
{
    currentTimerValueCounter = 0;
    flagInterruptPortJ = 0;
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
    //
    // Initialize the interrupts for GPIO PORT J.
    //
    initInterrupt();
    //
    // Initialize timer.
    //  
    initTimer();  
    
    //
    // Delay for 0.5sec
    //
    SysCtlDelay(ui32SysClock/6);

    //
    // Turn on the LED (start the game)
    //
    GPIOPinWrite(GPIO_PORTN_BASE, (USER_LED2), USER_LED2);
    
    //
    // Enable the timers.
    //
    TimerEnable(TIMER0_BASE, TIMER_BOTH);

    while(1)
    {           
        if(flagInterruptPortJ == 1) {
          flagInterruptPortJ = 0;
          //
          // Turn off the LED
          //
          GPIOPinWrite(GPIO_PORTN_BASE, (USER_LED2), 0);
          uint32_t currentTimerValueSec = convertToMiliSeconds(currentTimerValueCounter);
          printf("contagem: %d e tempo: %d ms\n", currentTimerValueCounter, currentTimerValueSec);
          break;
        }
        
        if(flagTimerValueCounter == 1){
          flagTimerValueCounter = 0;
          //
          // Turn off the LED
          //
          GPIOPinWrite(GPIO_PORTN_BASE, (USER_LED2), 0);
          printf("Acabou o tempo\n");
          break;
        }
    }
    printf("FIM DE JOGO\n");
}

