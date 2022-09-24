/*__________________________________________________________________________________
|       Disciplina de Sistemas Embarcados - 2022-1
|       Prof. Douglas Renaux
| __________________________________________________________________________________
|
|		Lab 1
| __________________________________________________________________________________
*/

/**
 * @file     main.c
 * @author   Douglas P. B. Renaux
 * @brief    Solution to Lab1 of ELF74/CSW41 - UTFPR. \n 
 *           Tools instalation and validation procedure.\n 
 *           Show messages on terminal using std::cout. \n 
 *           Show current value of some predefined macros (preprocessor symbols).\n 
 *           Read float value from terminal using std::cin.
 * @version  V2 -for 2022-1 semester
 * @date     Feb, 2022
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
  int adc0IntHandlerCalled;
  uint32_t ui32VerticalValue;
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
    //adOutputValue -> x
    //3730 is the max value obtained on the conversor AD and 230 the min value.
    return ((adOutputValue*pwmCounter)/3730)+230;
}
void ADCAndTimerInit(){
    //tomando o link: https://e2e.ti.com/support/microcontrollers/arm-based-microcontrollers-group/arm-based-microcontrollers/f/arm-based-microcontrollers-forum/374217/adc-interrupt-code-for-tiva
    
    adc0IntHandlerCalled = 0;                                                           //Inicia flag para interrupção
    
    //Peripheral section
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);                                         //Ativa um periferico (ADC0)          
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);                                        //Ativa um periferico (GPIOE)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);                                       //Ativa um periferico (TIMER0)
    
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
    {
    }
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))
    {
    }
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0))
    {
    }
    
    
    //TIMER section
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    uint32_t counter = 1200000000/5;                                                    //Counter until 200ms
    TimerLoadSet(TIMER0_BASE, TIMER_A, counter);
    TimerControlTrigger(TIMER0_BASE, TIMER_A, true);                                    //Configura o timer como trigger
    
    //GPIO section
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3);                                        //Configurando o pino PE3 para função alternativa de conversor AD
    //GPIOADCTriggerEnable(GPIO_PORTE_BASE, GPIO_PIN_3);                                  //Indica o pino de GPIO que realizar? o trigger da convers?o

    //ADC section
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_TIMER, 0);                           //Configura o sequenciador, prioridade do sequenciador e o tipo de gatilho usado
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, 
                             ADC_CTL_CH0|ADC_CTL_IE|ADC_CTL_END);                       //Configura o passo (relacionado a ordem se sequenciadores) e algumas configuracoes (canal 0 | interrupcoes ativas | ultima amostra lida)
    ADCSequenceEnable(ADC0_BASE, 3);                                                    //ativa o sequenciador 3 para o ADC0
}

void interruptInit(){
    
    //Interrupts for Timer
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
  
    //Interrupts for ADC
    ADCIntEnable(ADC0_BASE, 3);                                                         //Habilita a interrupção para o seq. SS3
    IntEnable(INT_ADC0SS3);
    
    IntMasterEnable();                                                                  //Habilita a chave geral de interrupções                                     
    IntPrioritySet(INT_ADC0SS3,0);                                                      //Seta o nível de prioridade para interrupção
    
    TimerEnable(TIMER0_BASE,TIMER_A);
    ADCIntClear(ADC0_BASE, 3);                                                          //ACK interrupcao para o ADC0
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
    
    GPIOPinConfigure(GPIO_PF2_M0PWM2);                  //Sets the alternative function (PWM) for pin
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2);
    
    //PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_8);           //Set the PWM clock to be 15Mhz
    //uint32_t ui32PWMClockRate = ui32SysClock / 8; 
    
    PWMClockSet(PWM0_BASE, PWM_SYSCLK_DIV_4);           //Set the PWM clock to be 30Mhz
    uint32_t ui32PWMClockRate = ui32SysClock / 4; 

    PWMGenConfigure(PWM0_BASE, PWM_GEN_1,
                        PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
        
     //
    // Set the PWM period to 30kHz.  To calculate the appropriate parameter
    // use the following equation: N = (1 / f) * PWMClk.  Where N is the
    // function parameter, f is the desired frequency, and PWMClk is the
    // PWM clock frequency based on the system clock. So the N = 1000
    //
    pwm0Counter = (ui32PWMClockRate / 30000);                           //this var is the N for equation
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, pwm0Counter);                 
    
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 250);                        //25% pulse width
    
    PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true); //Enable the PWM Out for PF2 (output signal)
    PWMGenEnable(PWM0_BASE, PWM_GEN_1);            //Enable the PWM generator block


    //AVISO: funcção para usar para modificar o pwm quando pegar o valor do conversor
    //PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, g_ui32PWMIncrement);
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
}
  /*------------------------------------------------------------------------------
 *
 *      Handler Functions and Methods
 *
 *------------------------------------------------------------------------------*/
void 
Adc0IntHandler(void) {
    ADCIntClear(ADC0_BASE, 3);
    adc0IntHandlerCalled = 1;
    ADCSequenceDataGet(ADC0_BASE, 3, &ui32VerticalValue);
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
    while(1){
      if(adc0IntHandlerCalled == 1){
          adc0IntHandlerCalled = 0;
          uint32_t newPulseWidthRED = convertADOutputToPwmPulseWidth(ui32VerticalValue, pwm0Counter);
          PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, newPulseWidthRED); //modify pulse width for analogic vertical position
          printf("Y axis: %d \n", ui32VerticalValue);
          
      }
    }
}
