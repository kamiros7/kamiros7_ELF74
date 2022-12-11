/*------------------------------------------------------------------------------
 *
 *     Projeto Final - Sistemas Embarcados (CSW41)
 *     Luis Camilo Jussiani Moreira - 2063166
 *     Jo�o Victor Laskoski - 1906470
 *
 *------------------------------------------------------------------------------*/



/* This is a small demo of the high-performance ThreadX kernel.  It includes examples of eight
   threads of different priorities, using a message queue, semaphore, mutex, event flags group, 
   byte pool, and block pool.  */

#include "tx_api.h"

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "inc/hw_memmap.h"

#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"
#include "driverlib/pin_map.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"

#define DEMO_STACK_SIZE         1024
#define DEMO_BYTE_POOL_SIZE     9120
#define DEMO_BLOCK_POOL_SIZE    100
#define DEMO_QUEUE_SIZE         100

enum OutputSelected {NONE, WINDOW, PAINEL, FEET, ALL};
enum controlMode { UNDEFINED, IDLE, HEAT, COOL};

/* Define the ThreadX objects...  */

TX_THREAD               thread_update_input;
TX_THREAD 		thread_control_temperature;
TX_THREAD		thread_register_output;
TX_BYTE_POOL            byte_pool_0;
TX_EVENT_FLAGS_GROUP    event_flags_0; //For active/inactive state flags
TX_EVENT_FLAGS_GROUP    event_flags_1; //For ISRs flags

/* Define byte pool memory.  */

UCHAR                   byte_pool_memory[DEMO_BYTE_POOL_SIZE];

/*------------------------------------------------------------------------------
 *
 *      Global vars
 *
 *------------------------------------------------------------------------------*/
uint32_t ui32SysClock;

//Values get in ISRs of ADC
uint32_t vectorJoystick[2];
uint32_t ui32HorizontalValueADC = 0;
uint32_t ui32VerticalValueADC = 0;
uint32_t ui32TemperatureValueADC = 0;

uint32_t realTemperature = 22; // The diff with ui32HorizontalValueADC is that the realTemperature 
				// is Celsius and the other is the ADC value
uint32_t desireTemperature = 25;
uint32_t mixTemperature = 27;
uint32_t desireFanSpeed = 1;
uint32_t openningHot = 0;
uint32_t openningCold = 0;
enum OutputSelected outputSelectedMode = NONE;

/*------------------------------------------------------------------------------
 *
 *      Functions and Methods
 *
 *------------------------------------------------------------------------------*/
/* Define thread prototypes.  */
void    thread_update_input_entry(ULONG thread_input);
void    thread_control_temp_entry(ULONG thread_input);
void    thread_regis_output_entry(ULONG thread_input);

void validateDesireTemperature(uint32_t temp);
void validateDesireFanSpeed(uint32_t speed);

bool openningIsValidate(uint32_t openning);
enum controlMode setControlMode(uint32_t tempDesejada, uint32_t tempReal);

void decimalToBinary(int num, int* binary);
uint32_t converterADCTempToRealTemp(uint32_t ui32TemperatureValueADC);
void converterADCJoystickToOutputMode(uint32_t joyHorADC, uint32_t joyVerADC);
void sendInfoToFPGA(int* binary, int controlMode[3]);
//Verify any updated for stop the thread for 5 secs.
short verifyUpdateVariables(uint32_t currentDesireTemp,
                            uint32_t currentRealTemp,
                            UINT currentFanSpeed,
                            UINT currentOpenningHot,
                            UINT currentOpenningCold,
                            enum OutputSelected currentOutputSelectedMode);
void findValidColdOpening(enum controlMode currentControlMode, 
                           enum controlMode* oldControlMode, 
                           uint32_t tempDesejada,
                           uint32_t* aberturaHot,
                           uint32_t* aberturaCold);
void initValuesToFPGA();

void GPIOInit();
/* Define what the initial system looks like.  */

void    tx_application_define(void *first_unused_memory)
{
CHAR    *pointer = TX_NULL;


#ifdef TX_ENABLE_EVENT_TRACE
    tx_trace_enable(trace_buffer, sizeof(trace_buffer), 32);
#endif

    /* Create a byte memory pool from which to allocate the thread stacks.  */
    tx_byte_pool_create(&byte_pool_0, "byte pool 0", byte_pool_memory, DEMO_BYTE_POOL_SIZE);

    /* Put system definition stuff in here, e.g. thread creates and other assorted
       create information.  */

    /* Allocate the stack for thread_update_input.  */
    tx_byte_allocate(&byte_pool_0, (VOID **) &pointer, DEMO_STACK_SIZE, TX_NO_WAIT);

    /* Create the main thread.  */
    tx_thread_create(&thread_update_input, "thread upd input", thread_update_input_entry, 0,  
            pointer, DEMO_STACK_SIZE, 
            0, 0, 3, TX_AUTO_START);
		
    /* Allocate the stack for thread_control_temperature.  */
    tx_byte_allocate(&byte_pool_0, (VOID **) &pointer, DEMO_STACK_SIZE, TX_NO_WAIT);

    tx_thread_create(&thread_control_temperature, "thread control temp", thread_control_temp_entry, 0,  
            pointer, DEMO_STACK_SIZE, 
            0, 0, 2, TX_AUTO_START);
    
    /* Allocate the stack for thread_register_output.  */
    tx_byte_allocate(&byte_pool_0, (VOID **) &pointer, DEMO_STACK_SIZE, TX_NO_WAIT);
		
    tx_thread_create(&thread_register_output, "thread reg output", thread_regis_output_entry, 0,  
            pointer, DEMO_STACK_SIZE, 
            0, 0, 4, TX_AUTO_START);
    
    /* Create event flag group for active/inactive state of system */
    tx_event_flags_create(&event_flags_0, "event flag state system");

    /* Create event flag group for ISR */
    tx_event_flags_create(&event_flags_1, "event flag ISR");

}

void thread_update_input_entry(ULONG thread_input) {
    ULONG actual_flags, actual_flags_isr;
    UINT status, status_isr;
    
    GPIOInit();
    initValuesToFPGA();
    while(1) {
        // Stop the thread until some output mode to be selected and is not NONE.
        status = tx_event_flags_get(&event_flags_0, 0x1, TX_OR,
            &actual_flags, TX_WAIT_FOREVER);
        
        //Check status
        if ((status != TX_SUCCESS) || (actual_flags != 0x1))
            continue;

        //Order of Flags ADC1 SS3, ADC0 SS2,PORT C6, PORT L2, PORT L1, PORT J1, PORT J0
        status_isr = tx_event_flags_get(&event_flags_1, 0x7F, TX_OR_CLEAR,
            &actual_flags_isr, TX_NO_WAIT);
        
	if(status_isr != TX_SUCCESS)
          continue;
        
        if(actual_flags_isr & 0x1){ //ISR Port J Pin 0 called
          desireTemperature--;
          validateDesireTemperature(desireTemperature);
        }
        if(actual_flags_isr & 0x2){ //ISR Port J Pin 1 called
          desireTemperature++;
          validateDesireTemperature(desireTemperature);
        }
        if(actual_flags_isr & 0x4){ //ISR Port L Pin 1 called
          desireFanSpeed++;
          validateDesireFanSpeed(desireFanSpeed);
        }
        if(actual_flags_isr & 0x8){ //ISR Port L Pin 2 called
          desireFanSpeed--;
          validateDesireFanSpeed(desireFanSpeed);
        }
        if(actual_flags_isr & 0x10){ //ISR Port C Pin 6 called
          outputSelectedMode = ALL;
        }
        if(actual_flags_isr & 0x20){ //ISR ADC0 SS2 called
          //nothing to do
        }
        if(actual_flags_isr & 0x40){ //ISR ADC1 SS3 called
          realTemperature = converterADCTempToRealTemp(ui32TemperatureValueADC);
        }
		
    }
}

void thread_control_temp_entry(ULONG thread_input) {
    ULONG actual_flags;
    UINT status;
    enum controlMode oldControlMode = UNDEFINED, currentControlMode = UNDEFINED;
    oldControlMode = setControlMode((int) desireTemperature, (int) realTemperature);
    while(1) {
        // Stop the thread until some output mode to be selected and is not NONE.
        status = tx_event_flags_get(&event_flags_0, 0x1, TX_OR,
            &actual_flags, TX_WAIT_FOREVER);   
        // Check status
        if ((status != TX_SUCCESS) || (actual_flags != 0x1))
            continue;
        
        currentControlMode = setControlMode((int) desireTemperature, (int) realTemperature);
        
        switch(currentControlMode){
            case IDLE:
                while(desireTemperature == realTemperature){ //pensar na questao do range (igual aproximadamente)
                    findValidColdOpening(currentControlMode, &oldControlMode, (uint32_t) desireTemperature, &openningHot, &openningCold);
                }
                oldControlMode = IDLE;
                break;
            case HEAT:
                while (desireTemperature > realTemperature){
                    findValidColdOpening(currentControlMode, &oldControlMode, (uint32_t) desireTemperature, &openningHot, &openningCold);
                }
                oldControlMode = HEAT;
                break;
            case COOL:
                while (desireTemperature < realTemperature){
                    findValidColdOpening(currentControlMode, &oldControlMode, (uint32_t) desireTemperature, &openningHot, &openningCold);
                }
                oldControlMode = COOL;
                break;
            case UNDEFINED:
                oldControlMode = UNDEFINED;
                break;
        }
    }
}

void thread_regis_output_entry(ULONG thread_input) {
    ULONG actual_flags;
    UINT status;
    
    uint32_t currentDesireTemp = 25;
    uint32_t currentRealTemp = 25;
    uint32_t currentFanSpeed = 2;
    uint32_t currentOpenningHot = 0;
    uint32_t currentOpenningCold = 0;
    enum OutputSelected currentOutputSelectedMode = NONE;
    SHORT currentVarsUpdated = 0;
    
    //Vars for transfer data to FPGA
    int binaryValue[32];
    int controlMode[3] = {0,0,0};
    while(1) {

      // Stop the thread until some output mode to be selected and is not NONE.	
      status = tx_event_flags_get(&event_flags_0, 0x1, TX_OR,
            &actual_flags, TX_WAIT_FOREVER);
      // Check status
      if ((status != TX_SUCCESS) || (actual_flags != 0x1))
          continue;
              
      currentVarsUpdated =  verifyUpdateVariables(currentDesireTemp,
                                              currentRealTemp,
                                              currentFanSpeed,
                                              currentOpenningHot,
                                              currentOpenningCold,                                                
                                              currentOutputSelectedMode); 
      //registrar temperatura real (será sempre enviada)
      controlMode[0] = 0;
      controlMode[1] = 0;
      controlMode[2] = 0;
      decimalToBinary( (int) realTemperature, binaryValue);
      sendInfoToFPGA( binaryValue, controlMode);
      
      if(currentVarsUpdated == 1) {
        //stop thread for 5 seconds.
        tx_thread_sleep(10); //bom verificar se fica diferente o ui32SysClock
        
        //ticks será uma variável global, o qual será definida
        // antes do tx_kernel, pegando a quantiadde de ticks por 1 segundo.
            
        //registrar temperatura desejada
        controlMode[0] = 0;
        controlMode[1] = 0;
        controlMode[2] = 1;
        decimalToBinary((int) desireTemperature, binaryValue);
        sendInfoToFPGA( binaryValue, controlMode);
        
        //registrar temperatura mistura
        controlMode[0] = 0;
        controlMode[1] = 1;
        controlMode[2] = 0;
        decimalToBinary((int) mixTemperature, binaryValue);
        sendInfoToFPGA( binaryValue, controlMode);
        
        //registrar saida selecionada
        controlMode[0] = 0;
        controlMode[1] = 1;
        controlMode[2] = 1;
        decimalToBinary(outputSelectedMode, binaryValue);
        sendInfoToFPGA( binaryValue, controlMode);
        
        //registrar velocidade do ventilador
        controlMode[0] = 1;
        controlMode[1] = 0;
        controlMode[2] = 0;
        decimalToBinary(desireFanSpeed, binaryValue);
        sendInfoToFPGA( binaryValue, controlMode);
              
        //registrar abertura de ar quente
        controlMode[0] = 1;
        controlMode[1] = 0;
        controlMode[2] = 1;
        decimalToBinary(openningHot, binaryValue);
        sendInfoToFPGA( binaryValue, controlMode);
        
        //registrar abertura de ar frio
        controlMode[0] = 1;
        controlMode[1] = 1;
        controlMode[2] = 0;
        decimalToBinary(openningCold, binaryValue);
        sendInfoToFPGA( binaryValue, controlMode);       
      }
      
    }
}

/*------------------------------------------------------------------------------
 *
 *    functions
 *
 *------------------------------------------------------------------------------*/

void validateDesireTemperature(uint32_t temp){
  if(temp > 30){
      desireTemperature = 30;
  } else if (temp < 16) {
      desireTemperature = 16;
  }
}

void validateDesireFanSpeed(uint32_t speed){
  if(speed > 5){
      desireFanSpeed = 5;
  } else if (speed < 1) {
      desireFanSpeed = 1;
  }
}

bool openningIsValidate(uint32_t openning) {
    bool isValid = true;
    if(!(openning >= 0 && openning <= 100)){
       isValid = false;
       openningCold = 0;
    }
    return isValid;
}

enum controlMode setControlMode(uint32_t tempDesejada, uint32_t tempReal){
    if(tempDesejada == tempReal) {
        return IDLE;
    } else if(tempDesejada > tempReal){
        return HEAT;
    } else {
        return COOL;
    }
	
}

uint32_t converterADCTempToRealTemp(uint32_t ui32TemperatureValueADC) {
   //2660          -> 40.0º
  // 0             -> 10ºC
  //ui32TemperatureValueADC -> x (return value)
  float temperature = ((float) 40*ui32TemperatureValueADC)/(2660) + 10;
  return temperature;
}

void converterADCJoystickToOutputMode(uint32_t joyHorADC, uint32_t joyVerADC) {
    //Not is used a specific local, but a region for to realize the choic
    if (joyHorADC > 3100 && (joyVerADC > 1600 && joyVerADC < 2400)) { //NONE
      outputSelectedMode = NONE;
    } else if ((joyHorADC > 1600 && joyHorADC < 2400 ) && (joyVerADC > 3100)) { //FEET
      outputSelectedMode = FEET;
    } else if ((joyHorADC > 1600 && joyHorADC < 2400 ) && joyVerADC < 900 ) { //PAINEL
      outputSelectedMode = PAINEL;
    } else if (joyHorADC < 900 && (joyVerADC > 1600 && joyVerADC < 2400)) { //WINDOW
       outputSelectedMode = WINDOW;
    }
}

void decimalToBinary(int num, int* binaryNum) {   

  // Stores binary representation of number.
  //int binaryNum[32]; // Assuming 32 bit integer.
  int i=0;
   
  if (num == 0) {
    for(i = 0; i < 7; i++){
      binaryNum[i] = 0;
    }
  }

  for ( i=0; i < 7; i++){
    binaryNum[6-i] = num % 2;
    num /= 2;
  }
}

void sendInfoToFPGA(int* binary, int controlMode[3]) {
    //disable write/enable pin (the pin is active for transfer data only.)
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0);
    //for control
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, controlMode[0]*GPIO_PIN_4);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_5, controlMode[1]*GPIO_PIN_5);
    GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_4, controlMode[2]*GPIO_PIN_4);

    //enable write/enable pin for transfer data
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_PIN_5);
    //for data
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, binary[0]*GPIO_PIN_4); //Aqui multiplicar pelo valor do pino, por exemplo binary[0]*GPIO_PIN_4
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, binary[1]*GPIO_PIN_5);
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0, binary[2]*GPIO_PIN_0);
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_1, binary[3]*GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_2, binary[4]*GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_3, binary[5]*GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, binary[6]*GPIO_PIN_4);
    //disable again the write/enable
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0);	
}

short verifyUpdateVariables(uint32_t currentDesireTemp,
                            uint32_t currentRealTemp,
                            UINT currentFanSpeed,
                            UINT currentOpenningHot,
                            UINT currentOpenningCold,
                            enum OutputSelected currentOutputSelectedMode) {
    short varIsUpdated = 0;
    
    if(currentDesireTemp != desireTemperature) {
      varIsUpdated = 1;
      currentDesireTemp = desireTemperature;
    } 
    if (currentFanSpeed != desireFanSpeed) {
      varIsUpdated = 1;
      currentFanSpeed = desireFanSpeed;
    }
    if (currentOpenningCold != openningCold) {
      varIsUpdated = 1;
      currentOpenningCold = openningCold;
    }
    if (currentOpenningHot != openningHot ) {
      varIsUpdated = 1;
      currentOpenningHot = openningHot;
    }
    if (currentOutputSelectedMode != outputSelectedMode ) {
      varIsUpdated = 1;
      currentOutputSelectedMode = outputSelectedMode;
    }
    return varIsUpdated;
	
}

void findValidColdOpening(enum controlMode currentControlMode, 
                          enum controlMode* oldControlMode, 
                          uint32_t tempDesejada,
                          uint32_t* aberturaHot,
                          uint32_t* aberturaCold) {
    bool openningIsValid = false;
    int32_t constanTempValue = 0; //Is the const that if the system is heat or cool, the const will be +2 or -2.
    uint32_t openningHotModifier; //variavel que irá modificar o valor da abertura, sendo um iterador positivo ou negativo
    
    uint32_t localOpenningHot = 0;
    uint32_t localOpenningCold = 0;
    
     switch(currentControlMode){
        case IDLE:
            if (*oldControlMode == COOL){
                localOpenningHot = 1;
                openningHotModifier = 1;
            } else if(*oldControlMode == HEAT){
                localOpenningHot = 99;
                openningHotModifier = -1;
            } else {
                openningHotModifier = 0;
            }
            constanTempValue = 0;
            break;
        case COOL:
            localOpenningHot = 1;
            constanTempValue = (tempDesejada >= 18) ? -2 : 0;
            openningHotModifier = 1;
            break;
        case HEAT:
            localOpenningHot = 99;
            constanTempValue = 2;
            openningHotModifier = -1;
            break;
        case UNDEFINED:
          constanTempValue =0;
          openningHotModifier = 0;
            break;		
    }
    mixTemperature = desireTemperature + constanTempValue;
    while((!openningIsValid && *oldControlMode != IDLE)){ //or to use ondControlMode != currentControlMode 
        //*aberturaCold = (-1)*(*aberturaHot)*(tempDesejada -40 + constanTempValue)/ (tempDesejada + constanTempValue - 15);
        localOpenningCold = (-1)*(localOpenningHot)*(tempDesejada -40 + constanTempValue)/ (tempDesejada + constanTempValue - 15);
        openningIsValid = openningIsValidate(localOpenningCold);
        if(!openningIsValid){
            (localOpenningHot) = (localOpenningHot) + openningHotModifier;
        }
        if((localOpenningHot) < 0 || (localOpenningHot) > 100) { //The iterator arrived in the limits, never will to find a valid openning
            //do nothing,but to consider an error case
            localOpenningHot = 0;
            localOpenningCold = 0;
            
        }
    }
    if(openningIsValid) {
      *aberturaHot = localOpenningHot;
      *aberturaCold = localOpenningCold;
    }
    
    //A regi�o abaixo, � para atualizar o oldControlMode, principalmente para n�o cair no bug de estava no estado IDLE, e ao partir para o estado
    //esquetando ou esfriando, enquanto estiver nesse estado e o estado anterior era o IDLE, nunca calcular as aberturas, at� que o oldControlMode seja
    //esquentando ou resfriando.
    
    switch (currentControlMode) {
      case IDLE:
        *oldControlMode = IDLE;
        break;
      case COOL:
        *oldControlMode = COOL;
        break;
      case HEAT:
        *oldControlMode = HEAT;
        break;
      case UNDEFINED:
        *oldControlMode = UNDEFINED;
        break;
    }
}

void initValuesToFPGA() {
     //Vars for transfer data to FPGA
    int binaryValue[32];
    int controlMode[3] = {0,0,0};

    //registrar temperatura real (será sempre enviada)
    decimalToBinary( (int) realTemperature, binaryValue);
    sendInfoToFPGA( binaryValue, controlMode);
      
    //ticks será uma variável global, o qual será definida
    // antes do tx_kernel, pegando a quantiadde de ticks por 1 segundo.
        
    //registrar temperatura desejada
    controlMode[0] = 0;
    controlMode[1] = 0;
    controlMode[2] = 1;
    decimalToBinary((int) desireTemperature, binaryValue);
    sendInfoToFPGA( binaryValue, controlMode);
    
    //registrar temperatura mistura
    controlMode[0] = 0;
    controlMode[1] = 1;
    controlMode[2] = 0;
    decimalToBinary((int) mixTemperature, binaryValue);
    sendInfoToFPGA( binaryValue, controlMode);
    
    //registrar saida selecionada
    controlMode[0] = 0;
    controlMode[1] = 1;
    controlMode[2] = 1;
    decimalToBinary(outputSelectedMode, binaryValue);
    sendInfoToFPGA( binaryValue, controlMode);
    
    //registrar velocidade do ventilador
    controlMode[0] = 1;
    controlMode[1] = 0;
    controlMode[2] = 0;
    decimalToBinary(desireFanSpeed, binaryValue);
    sendInfoToFPGA( binaryValue, controlMode);
          
    //registrar abertura de ar quente
    controlMode[0] = 1;
    controlMode[1] = 0;
    controlMode[2] = 1;
    decimalToBinary(openningHot, binaryValue);
    sendInfoToFPGA( binaryValue, controlMode);
    
    //registrar abertura de ar frio
    controlMode[0] = 1;
    controlMode[1] = 1;
    controlMode[2] = 0;
    decimalToBinary(openningCold, binaryValue);
    sendInfoToFPGA( binaryValue, controlMode);       
    
}

/*------------------------------------------------------------------------------
 *
 *      Interrupt functions (ISR)
 *
 *------------------------------------------------------------------------------*/
//Order of Flags ADC1 SS3, ADC0 SS2,PORT C6, PORT L2, PORT L1, PORT J1, PORT J0
void Adc0SS2IntHandler(void) {
    ADCIntClear(ADC0_BASE, 2);
    ADCSequenceDataGet(ADC0_BASE, 2, vectorJoystick);
    ui32VerticalValueADC = vectorJoystick[0];
    ui32HorizontalValueADC = vectorJoystick[1];
    converterADCJoystickToOutputMode(ui32HorizontalValueADC, ui32VerticalValueADC);
    
    tx_event_flags_set(&event_flags_1, 0x20 ,TX_OR);
    if(outputSelectedMode != NONE) {
      /* Set event flag 0 to wakeup the threads. */
      tx_event_flags_set(&event_flags_0, 0x1, TX_OR);
    } else {
      /* Set event flag 0 to sleep the threads. */
      openningCold = 0;
      openningHot = 0;
      tx_event_flags_set(&event_flags_0, 0x0, TX_AND);
    }
}

//Order of Flags ADC1 SS3, ADC0 SS2,PORT C6, PORT L2, PORT L1, PORT J1, PORT J0
void Adc1SS3IntHandler(void) {
    ADCIntClear(ADC1_BASE, 3);
    ADCSequenceDataGet(ADC1_BASE, 3, &ui32TemperatureValueADC);
    tx_event_flags_set(&event_flags_1, 0x40 ,TX_OR);
}

//Order of Flags ADC1 SS3, ADC0 SS2,PORT C6, PORT L2, PORT L1, PORT J1, PORT J0
void GpioPortJHandler (){
    uint32_t interrputs = GPIOIntStatus(GPIO_PORTJ_BASE, false);
    GPIOIntClear(GPIO_PORTJ_BASE, GPIO_INT_PIN_0 | GPIO_INT_PIN_1);	
    if ((interrputs & GPIO_INT_PIN_0) != 0) {
        tx_event_flags_set(&event_flags_1, 0x1 ,TX_OR); 
    }
    if ((interrputs & GPIO_INT_PIN_1) != 0) {
        tx_event_flags_set(&event_flags_1, 0x2 ,TX_OR); 
    }
}

//Order of Flags ADC1 SS3, ADC0 SS2,PORT C6, PORT L2, PORT L1, PORT J1, PORT J0
void GpioPortLHandler (){
    uint32_t interrputs = GPIOIntStatus(GPIO_PORTL_BASE, false);
    GPIOIntClear(GPIO_PORTL_BASE, GPIO_INT_PIN_1 | GPIO_INT_PIN_2);
    if ((interrputs & GPIO_INT_PIN_1) != 0) {
        tx_event_flags_set(&event_flags_1, 0x4 ,TX_OR);
    }
    if ((interrputs & GPIO_INT_PIN_2) != 0) {
        tx_event_flags_set(&event_flags_1, 0x8 ,TX_OR);
    }
}

//Order of Flags ADC1 SS3, ADC0 SS2,PORT C6, PORT L2, PORT L1, PORT J1, PORT J0
void GpioPortCHandler (){
    GPIOIntClear(GPIO_PORTC_BASE, GPIO_INT_PIN_6);
    tx_event_flags_set(&event_flags_0, 0x1, TX_OR);
    tx_event_flags_set(&event_flags_1, 0x10 ,TX_OR);
}

void Timer0IntHandler(void) {
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}

/*------------------------------------------------------------------------------
 *
 *      Init and configuration pins
 *
 *------------------------------------------------------------------------------*/

void initButtonsAndCommunicationPeripherals() {
    uint32_t peripherals[8] = {SYSCTL_PERIPH_GPIOJ, SYSCTL_PERIPH_GPIOL, SYSCTL_PERIPH_GPIOC,
    SYSCTL_PERIPH_GPIOB, SYSCTL_PERIPH_GPIOK, SYSCTL_PERIPH_GPIOA, SYSCTL_PERIPH_GPION, SYSCTL_PERIPH_GPIOP};
    
    int counter;
    for (counter = 0; counter < 8; counter++) {		
      //
      // Enable the GPIO port.
      //
      SysCtlPeripheralEnable(peripherals[counter]);
      while(!SysCtlPeripheralReady(peripherals[counter]))
      {
      }
    }
    
    //
    //INPUT BUTTONS SECTION
    //
    
    //Configure the GPIO port for the user button of the Tiva board.
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_0);
    GPIOPinTypeGPIOInput(GPIO_PORTJ_BASE, GPIO_PIN_1);
    GPIOPadConfigSet(GPIO_PORTJ_BASE, GPIO_PIN_0 | GPIO_PIN_1, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    //Configure the GPIO port for the user button of the Tiva board.
    GPIOPinTypeGPIOInput(GPIO_PORTL_BASE, GPIO_PIN_1);
    GPIOPinTypeGPIOInput(GPIO_PORTL_BASE, GPIO_PIN_2);
    
    //Configure the GPIO port for the button of the Joystick.
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_6);

    //
    //OUTPUT COMMUNICATION SECTION
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_4);
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_4, 0);
    
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_5);
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_5, 0);
    
    GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_0);
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_0, 0);
	
    GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_1);
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_1, 0);
	
    GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_2);
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_2, 0);
	
    GPIOPinTypeGPIOOutput(GPIO_PORTK_BASE, GPIO_PIN_3);
    GPIOPinWrite(GPIO_PORTK_BASE, GPIO_PIN_3, 0);

    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_4);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_4, 0);

    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_4);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_4, 0);
	
    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_5);
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_5, 0);
	
    GPIOPinTypeGPIOOutput(GPIO_PORTP_BASE, GPIO_PIN_4);
    GPIOPinWrite(GPIO_PORTP_BASE, GPIO_PIN_4, 0);
    
    GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5);
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0);
}

void initADConversorPeripherals() {
    //
    // Enable the GPIO port.
    //
     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE))
    {
    }
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC0))
    {
    } 
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_ADC1))
    {
    }
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_TIMER0))
    {
    } 
    
    //TIMER section
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    uint32_t counter = 1200000000/50;       //Counter until 20ms
    TimerLoadSet(TIMER0_BASE, TIMER_A, counter);
    TimerControlTrigger(TIMER0_BASE, TIMER_A, true); 
    
    //
    //ADC SECTION
    //
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_2); //temperature sensor
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3); //joystick (horizontal)
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_4); //joystick (vertical)
    
    GPIOADCTriggerEnable(GPIO_PORTE_BASE, GPIO_PIN_2);
    GPIOADCTriggerEnable(GPIO_PORTE_BASE, GPIO_PIN_3);
    GPIOADCTriggerEnable(GPIO_PORTE_BASE, GPIO_PIN_4);

    //ADC configuration (joystick)
    ADCSequenceConfigure(ADC0_BASE, 2, ADC_TRIGGER_TIMER, 0); 
    ADCSequenceStepConfigure(ADC0_BASE, 2, 0, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE, 2, 1, ADC_CTL_CH9 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE, 2); 
    
    //ADC configuration (temperature sensor)
    ADCSequenceConfigure(ADC1_BASE, 3, ADC_TRIGGER_TIMER, 1); 
    ADCSequenceStepConfigure(ADC1_BASE, 3, 0, ADC_CTL_CH1 | ADC_CTL_IE | ADC_CTL_END);
    ADCSequenceEnable(ADC1_BASE, 3);
}

void interruptInit() {
  
    IntMasterDisable();

    //Interrupts for componentes that uses the ADC 
    //ADC0 - SS2: Joystick Horizontal and Joystick Vertical
    //ADC1 - SS3: Temperature sensor
	
    ADCIntEnable(ADC0_BASE, 2);      //Habilita a interrupção para o seq. SS2
    ADCIntEnable(ADC1_BASE, 3);      //Habilita a interrupção para o seq. SS3

    IntEnable(INT_ADC0SS2);
    IntEnable(INT_ADC1SS3);
        
    IntPrioritySet(INT_ADC0SS2, 0x02);   //Seta o nível de prioridade para interrupção
    IntPrioritySet(INT_ADC1SS3, 0x02);   //Seta o nível de prioridade para interrupção
    
    //Interrupts for componentes that uses the DigitalIO 
    GPIOIntEnable(GPIO_PORTJ_BASE, GPIO_INT_PIN_0);
    GPIOIntEnable(GPIO_PORTJ_BASE, GPIO_INT_PIN_1);
    
    GPIOIntTypeSet(GPIO_PORTJ_BASE, GPIO_PIN_0 , GPIO_LOW_LEVEL);
    GPIOIntTypeSet(GPIO_PORTJ_BASE, GPIO_PIN_1 , GPIO_LOW_LEVEL);
  
    IntEnable(INT_GPIOJ);
    IntPrioritySet(INT_GPIOJ, 0x02);
	
    //Speed section
    GPIOIntEnable(GPIO_PORTL_BASE, GPIO_INT_PIN_1);
    GPIOIntEnable(GPIO_PORTL_BASE, GPIO_INT_PIN_2);
    
    GPIOIntTypeSet(GPIO_PORTL_BASE, GPIO_PIN_1 , GPIO_LOW_LEVEL);
    GPIOIntTypeSet(GPIO_PORTL_BASE, GPIO_PIN_2 , GPIO_LOW_LEVEL);

    IntEnable(INT_GPIOL);
    IntPrioritySet(INT_GPIOL, 0x02);

    //Selecton exit section
    GPIOIntEnable(GPIO_PORTC_BASE, GPIO_INT_PIN_6);

    GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_6 , GPIO_LOW_LEVEL);
	
    IntEnable(INT_GPIOC);
    IntPrioritySet(INT_GPIOC, 0x02);
    
    //Timer section
     //Interrupts for Timer
    IntEnable(INT_TIMER0A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntPrioritySet(INT_TIMER0A, 0x02);
	
    //Enable the general interrupts
    IntMasterEnable();             
    
    ADCIntClear(ADC0_BASE, 2);
    ADCIntClear(ADC1_BASE, 3); 
    TimerEnable(TIMER0_BASE,TIMER_A);

}

void GPIOInit() {
    initButtonsAndCommunicationPeripherals();
    initADConversorPeripherals();
    interruptInit(); 
}


/* Define main entry point.  */

int main()
{
    /* Enter the ThreadX kernel.  */
    tx_kernel_enter();
}

