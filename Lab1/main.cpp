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

#include "template.h"

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
int main(int argc, char ** argv)
{
    float f_var;
    std::cout << __cplusplus << std::endl;
    std::cout << __DATE__ << std::endl;
    std::cout << __TIME__ << std::endl;
    std::cout << __FILE__ << std::endl;
    std::cout << __LINE__ << std::endl;
    std::cout << __STDC__ << std::endl;   
    std::cout << __STDC_VERSION__ << std::endl;
    std::cout << __ARM_ARCH << std::endl;
    std::cout << __ARM_ARCH_ISA_THUMB << std::endl;
    std::cout << __ARM_SIZEOF_MINIMAL_ENUM << std::endl;
    std::cout << __ARM_SIZEOF_WCHAR_T << std::endl;
    std::cout << __ARMVFP__ << std::endl;
    std::cout << __CORE__ << std::endl;
    
    std::cout << "escreva um valor" << std::endl ;
    std::cin >> f_var;
    
    f_var = f_var + 5.5f;
    std::cout << f_var << std::endl;
    return 0;
}
