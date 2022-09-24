/*__________________________________________________________________________________
|       Disciplina de Sistemas Embarcados - 2022-1
|       Prof. Douglas Renaux
| __________________________________________________________________________________
|
|		Lab 1
| __________________________________________________________________________________
*/

/**
 * @file     main.cpp
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

#include <iostream>
using std::cout;
//#include "template.h"
#include "driverlib/adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"

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
  bool adc0IntHandlerCalled;
  uint32_t ui32VerticalValue;
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

/**
 * Main function.
 *
 * @param[in] argc - not used, declared for compatibility
 * @param[in] argv - not used, declared for compatibility
 * @returns int    - not used, declared for compatibility
 */


void ADCInit(){
    adc0IntHandlerCalled = false;
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);                                         //Ativa um periferico (ADC0)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);                                        //Ativa um periferico (GPIOE)   
    //AQUI NAO DEVEMOS TER UM LOOP PARA ESPERAR O PERIFERICO ESTAR PRONTO
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_EXTERNAL, 0);                        //Configura o sequenciador, prioridade do sequenciador e o tipo de gatilho usado
    GPIOADCTriggerEnable(GPIO_PORTE_BASE, GPIO_PIN_3);                                  //Indica o pino de GPIO que realizar? o trigger da convers?o
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0|ADC_CTL_IE|ADC_CTL_END);      //Configura o passo (relacionado a ordem se sequenciadores) e algumas configuracoes (canal 0 | interrupcoes ativas | ultima amostra lida)
    ADCSequenceEnable(ADC0_BASE, 3);                                                    //ativa o sequenciador 3 para o ADC0
    ADCIntClear(ADC0_BASE, 3);                                                          //ACK interrupcao
}

void ADC0IntHandler(void){
    ADCIntClear(ADC0_BASE, 3);
    adc0IntHandlerCalled = true;
    ADCSequenceDataGet(ADC0_BASE, 3, &ui32VerticalValue);
}

void LEDInit(){
    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Check if the peripheral access is enabled.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }

    //
    // Enable the GPIO pin for the LED (PF2 and PF3).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    //
    // 
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
    
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);

    //
    // Check if the peripheral access is enabled.
    //
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOG))
    {
    }

    //
    // Enable the GPIO pin for the LED (PG0).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTG_BASE, GPIO_PIN_0);
    
    //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
    //GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTG_BASE, GPIO_PIN_0, GPIO_PIN_0);

}

int main(int argc, char ** argv)
{   
    ADCInit();
    LEDInit();
    
    while(1){
      if(adc0IntHandlerCalled == true){
          adc0IntHandlerCalled = false;
          cout << "Y axis: " << ui32VerticalValue << std::endl;
      }
    }
}
