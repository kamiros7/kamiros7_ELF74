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

//#include "inc/hw_ints.h"
//#include "inc/hw_types.h"
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


void ADCinit(){
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);                                         //Ativa um periferico (ADC0)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);                                        //Ativa um periferico (GPIOE)   
    ADCSequenceConfigure(ADC0_BASE, 3, ADC_TRIGGER_EXTERNAL, 0);                        //Configura o sequenciador, prioridade do sequenciador e o tipo de gatilho usado
    GPIOADCTriggerEnable(GPIO_PORTE_BASE, GPIO_PIN_3);                                  //Indica o pino de GPIO que realizará o trigger da conversão
    ADCSequenceStepConfigure(ADC0_BASE, 3, 0, ADC_CTL_CH0|ADC_CTL_IE|ADC_CTL_END);      //Configura o passo (relacionado a ordem se sequenciadores) e algumas configuracoes (canal 0 | interrupcoes ativas | ultima amostra lida)
    ADCSequenceEnable(ADC0_BASE, 3);                                                    //ativa o sequenciador 3 para o ADC0
    ADCIntClear(ADC0_BASE, 3);                                                          //ACK interrupcao
}

void ADCInterrupt(){
    
}

int main(int argc, char ** argv)
{   
    return 0;
}
