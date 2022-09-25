/*__________________________________________________________________________________
|       Disciplina de Sistemas Embarcados - 2022-1
|       Prof. Douglas Renaux
| __________________________________________________________________________________
|
|		Lab 4
| __________________________________________________________________________________
*/

/**
 * @file     main.c
 * @author   Luis Camilo Jussiani Moreira e João Victor Laskoski
 * @version  V2 -for 2022-2 semester
 * @date     Sep, 2022
 ******************************************************************************/

/*------------------------------------------------------------------------------
 *
 *      Use Doxygen to report lab results
 *
 *------------------------------------------------------------------------------*/
/** @mainpage Results from Lab1
 *
 * @section Ouput Values
 *
 * The values of ...
 *
 * @section Terminal
 *
 * @subsection Output
 *
 * etc...
 */

/*------------------------------------------------------------------------------
 *
 *      File includes
 *
 *------------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"

/*------------------------------------------------------------------------------
 *
 *      Typedefs and constants
 *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 *
 *      Global vars
 *
 *------------------------------------------------------------------------------*/
  int adc0SS2IntHandlerCalled;
  uint32_t vectorJoystick[2];
  uint32_t ui32VerticalValue;
  uint32_t ui32HorizontalValue;
  uint32_t ui32JoystickButtonValue;
  uint32_t pwm0Counter;
  uint32_t ui32SysClock;
/*------------------------------------------------------------------------------
 *
 *      File scope vars
 *
 *------------------------------------------------------------------------------*/

/*------------------------------------------------------------------------------
 *
 *      Functions and Methods
 *
 *------------------------------------------------------------------------------*/
  //@pwmCounter is the N of PWMGenPeriodSet function
uint32_t convertADOutputToPwmPulseWidth(uint32_t adOutputValue, uint32_t pwmCounter) {
    //3730          -> pwmCounter
    //adOutputValue -> x (return value)
    //3730 is the max value obtained on the conversor AD and 230 the min value.
    return (((adOutputValue-230)*pwmCounter)/3730);
}
void ADCAndTimerInit(){
    //tomando o link: https://e2e.ti.com/support/microcontrollers/arm-based-microcontrollers-group/arm-based-microcontrollers/f/arm-based-microcontrollers-forum/374217/adc-interrupt-code-for-tiva
    
    adc0SS2IntHandlerCalled = 0;                                                        //Inicia flag para interrupção do SS2
    //Peripheral section
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);                                         //Enable the peripheral (ADC0)          
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);                                        //Enable the peripheral (GPIOE)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);                                       //Enable the peripheral (TIMER0)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);                                        //Enable the peripheral (GPIOC)
    
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
    {
    }
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))
    {
    }
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0))
    {
    }
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOC))
    {
    }
    
    //TIMER section
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    uint32_t counter = 1200000000/5;                                                    //Counter until 200ms
    TimerLoadSet(TIMER0_BASE, TIMER_A, counter);
    TimerControlTrigger(TIMER0_BASE, TIMER_A, true);                                    //Configura o timer como trigger
    
    //GPIO section
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);                                        //Configurando o pino PE3 para função alternativa de conversor AD (vertical)
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_4);                                        //Configurando o pino PE4 para função alternativa de conversor AD (horizontal)
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_6);                                 //Configure the pin PC4 like a gpio pin (joystick button like a R3 of dualsense controller)
    
    //ADC section
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_TIMER, 0);                           //Configura o sequenciador, prioridade do sequenciador e o tipo de gatilho usado
    
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH0);                              //A pos. da vertical será a primeira amostra
    ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_CH9 | ADC_CTL_IE | ADC_CTL_END);   //A pos. da horizontal será a segunda amostra (sendo a última do seq. e gerará uma interrupção quando completada)
    ADCSequenceEnable(ADC0_BASE, 2);
}

void interruptInit(){
    
    //Interrupts for Timer
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  
    //Interrupts for ADC
    ADCIntEnable(ADC0_BASE, 2);                                                         //Habilita a interrupção para o seq. SS2
    IntEnable(INT_ADC0SS2);
        
    IntMasterEnable();                                                                  //Habilita a chave geral de interrupções                                     
    IntPrioritySet(INT_ADC0SS2,0);                                                      //Seta o nível de prioridade para interrupção
    
    TimerEnable(TIMER0_BASE,TIMER_A);
    ADCIntClear(ADC0_BASE, 2);                                                          //ACK interrupcao para o ADC0 SS2
}

void LEDAndPWMInit(){
    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    //
    // Enable the PWM peripheral
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOG))
    {
    }
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0))
    {
    }
    
    //
    //GPIO Section
    //
    GPIOPinConfigure(GPIO_PF2_M0PWM2);                      //Sets the alternative function (PWM) for pin PF2
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
    
    GPIOPinConfigure(GPIO_PF3_M0PWM3);                     //Sets the alternative function (PWM) for pin PF3
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_3);

    GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_0);   //Configure the pin PG0 like a gpio pin
    GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, 0);         //Init the pin with 0

    //
    //PWM Section
    //
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_4);           //Set the PWM clock to be 30Mhz
    uint32_t ui32PWMClockRate = ui32SysClock / 4; 

    PWMGenConfigure(PWM0_BASE, PWM_GEN_1,
                        PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_DB_NO_SYNC);
    
    //
    // Set the PWM period to 30kHz.  To calculate the appropriate parameter
    // use the following equation: N = (1 / f) * PWMClk.  Where N is the
    // function parameter, f is the desired frequency, and PWMClk is the
    // PWM clock frequency based on the system clock. So the N = 1000
    //
    pwm0Counter = (ui32PWMClockRate/30000);                           //this var is the N for equation (antes era 30000)
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, pwm0Counter);                 
    
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 10);                        //Intial pulse width
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, 10);                        //Intial pulse width

    
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);             //Enable the PWM generator block
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true); //Enable the PWM Out for PF2 (output signal)
    PWMOutputState(PWM0_BASE, PWM_OUT_3_BIT, true); //Enable the PWM Out for PF3 (output signal)

}

void UARTInit() {
    //
    //Peripheral section
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_UART0))
    {
    }
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOA))
    {
    }
    
    //
    //GPIO Section
    //
    
    //Set the alternative functions for gpio pins, for the uart communication
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    
    /*Configuracao para (8N1):
            8 bits de dados	(UART_CONFIG_WLEN_8)
            1 stop bit (UART_CONFIG_STOP_ONE )
            sem bit de paridade (UART_CONFIG_PAR_NONE)
            
    */
    
    UARTConfigSetExpClk(UART0_BASE, ui32SysClock, 115200,
                       (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
                        
    UARTEnable(UART0_BASE);
}

void systemInit() {
    //
    // Run from the PLL at 120 MHz.
    // Note: SYSCTL_CFG_VCO_240 is a new setting provided in TivaWare 2.2.x and
    // later to better reflect the actual VCO speed due to SYSCTL#22.
    //
    ui32SysClock = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                       SYSCTL_OSC_MAIN |
                                       SYSCTL_USE_PLL |
                                       SYSCTL_CFG_VCO_240), 120000000);
    
    ADCAndTimerInit();
    LEDAndPWMInit();
    interruptInit();
    UARTInit();
}
  /*------------------------------------------------------------------------------
 *
 *      Handler Functions and Methods
 *
 *------------------------------------------------------------------------------*/
void 
Adc0SS2IntHandler(void) {
    ADCIntClear(ADC0_BASE, 2);
    adc0SS2IntHandlerCalled = 1;
    ADCSequenceDataGet(ADC0_BASE, 2, vectorJoystick);
    ui32VerticalValue = vectorJoystick[0];
    ui32HorizontalValue = vectorJoystick[1];
    ui32JoystickButtonValue = GPIOPinRead(GPIO_PORTC_BASE,GPIO_PIN_6);
}

void
Timer0IntHandler(void) {
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

/**
 * Main function.
 *
 * @param[in] argc - not used, declared for compatibility
 * @param[in] argv - not used, declared for compatibility
 * @returns int    - not used, declared for compatibility
 */

int main(int argc, char ** argv)
{   
    systemInit();
    UARTCharPutNonBlocking(UART0_BASE, 'c');
    UARTDisable(UART0_BASE);
    while(1){
      if(adc0SS2IntHandlerCalled == 1){
          adc0SS2IntHandlerCalled = 0;
          uint32_t newPulseWidthRED = convertADOutputToPwmPulseWidth(ui32VerticalValue, pwm0Counter);
          PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, newPulseWidthRED); //modify pulse width for analogic vertical position
          
          uint32_t newPulseWidthGreen = convertADOutputToPwmPulseWidth(ui32HorizontalValue, pwm0Counter);
          PWMPulseWidthSet(PWM0_BASE, PWM_OUT_3, newPulseWidthGreen); //modify pulse width for analogic vertical position
                    
          if(ui32JoystickButtonValue == 64) { //value when the button ins't pressed
              GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, 0); //modify the blue pin of rgb led with the value of joystick button
          } else if (ui32JoystickButtonValue == 0) {                    //value when the button is pressed
              GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, GPIO_PIN_0);    //modify the blue pin of rgb led with the value of joystick button
          }        
      }
    }
}
