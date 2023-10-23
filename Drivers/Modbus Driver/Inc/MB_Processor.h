/*
 * MB_Processor.h
 *
 *  Created on: Mar 9, 2022
 *      Author: Alper
 *
 *  This driver is copied and modified.
 *  Original source: https://github.com/srikanth977/ModbusRTUSlave_RS232
 *
 *  UPDATE LOG:
 *
 *     Check_Holding_Register_Value function added.
 *     Several comment lines and TODO requests are added.
 *     Flash read and write library added
 *     MODBUS Slave ID flash save and ID check/update functions implemented
 *     Init
 *
 *
 *  Special Holding register values:
 *     1st element (0th in the array): 100 ms pulse counter data
 *     3rd element: New slave ID (Note that this number will be reset in case of shutdown.)
 *
 *
 *  Slave ID configuration:
 *     The slave ID number is automatically updated from the flash address 0x08012000, if the value that the given address hold is in between 1-99.
 *     Otherwise the default value for the slave id is 1.
 *     To update the slave id and 0x08012000 address data, one should write the desired id to the 3rd holding register of the modbus driver with the function codes of 0x06 or 0x10.
 *     In case of power loss, the slave ID will be retrieved from the flash.
 *
 *  Driver setup:
 *     There should be some additional lines of code in the interrupt and main init functions of USART3, TIM6 and TIM7
 *     in the stm32fxx_it.c, main.c and main.h files.
 *     NOTE: They are needed in order to use this Driver. Related lines have this comment:  //For Modbus implementation\newline
 *
 *     Lines:
 *  	in main.h, one should include this header file:
 *  		#include "MB_Processor.h"   //For Modbus implementation //Detailed information can be found in MB_Processor.h
 *  	in main.c, before the while loop, one should add this line for initialisation:
 *  		MB_Init(); //For Modbus implementation
 *  	in stm32fxx\_it.c, for interrupt handler functions of TIM6, TIM7 and USART3 user should add the following lines:\\
 *  	in TIM6\_DAC\_IRQHandler function:
 *  		MB_TIM6_DAC_IRQHandler(); //For Modbus implementation
 *  	in TIM7\_IRQHandler function:
 *  		MB_TIM7_IRQHandler(); //For Modbus implementation
 *  	in USART3\_IRQHandler function:
 *  		MB_USART3_IRQHandler(); //For Modbus implementation
 *
 *  Cube IDE configuration:
 *     USART3 is initilized in Async mode with the baud rate of 115200. USART3 global interrupt is enabled.
 *     There is one start and one stop bit and no parity, hence the data is 10bits long.
 *     Furthermore, Hardware Flow Control is enabled in the .ioc configuration file, as it is required by RS485 Network(optional).
 *     When there is a data, USART3 interrupt occurs and data is stored.
 *
 *     TIM6 and TIM7 are used with 1Mhz frequency in order to check 1.5 and 3.5 char timings.
 *     Both timer interrupts are enabled. 1MHz clock is achieved by selecting the prescalar as 71 as the
 *     clock speed of these timers are 72MHz. The prescaler is off by 1 because it’s 0-based: a PSC value
 *     of “0” means to use a prescaler (clock divider) of 1.
 *
 *     Counter periods are 750 and 1750 for TIM6 an TIM7. (1MHz means 1 microsecond tick therefore timers
 *     overflow at 750 and 1750 micro seconds) Auto reload of 1.5 char timer ,i.e. TIM6, is disabled as this timer
 *     only used in between data frames. Furthermore, Trigger Event selection is selected as Enable(CNT\_EN)
 *     Auto reload is enabled for TIM7(3.5 char counter). Trigger event selection is same, Enable(CNT\_EN).
 *
 *
 * */

#ifndef MODBUS_DRIVER_MB_PROCESSOR_H_
#define MODBUS_DRIVER_MB_PROCESSOR_H_

#include "stm32f3xx_hal.h"

void MB_Init(void);

unsigned short CRC16 (volatile unsigned char *puchMsg, unsigned short usDataLen );
unsigned int MBRegisterCount(void);
void AppendDatatoMBRegister(unsigned int StAddr,unsigned int count, unsigned int *inreg, volatile unsigned char *outreg);
unsigned int MBStartAddress(void);
void MBSendData(unsigned char count);
void AppendCRCtoMBRegister(unsigned char packtop);
void MBException(unsigned char exceptionCode);
void MBProcessRegisterRead(unsigned int *InArr, unsigned int InArrSize);
void AppendBitsToRegisters(unsigned int StAddr, unsigned int count, unsigned char *inreg, volatile unsigned char *outreg);
void MBProcessBitsRead(unsigned char *InArr, unsigned int InArrSize);
void CheckMBPDU(void);


void WriteMBRegistertoData(unsigned int StAddr,unsigned int count, volatile unsigned char *inreg, unsigned int *outreg);
void MBPresetSingleRegister(unsigned int *InArr, unsigned int InArrSize);
void MBPresetMultipleRegisters(unsigned int *InArr, unsigned int InArrSize);

void MBForceSingleCoil(unsigned char *InArr, unsigned int InArrSize);
void MBForceMultipleCoils(unsigned char *InArr, unsigned int InArrSize);
void WriteMBRegistersToBits(unsigned int StAddr, unsigned int count, volatile unsigned char *inreg,unsigned char *outreg);

unsigned int ReturnDiagCounter(unsigned int scode);
void ClearModbusCounters(void);
void MBProcessDiagnostics(void);

void MB_USART3_IRQHandler(void);
void MB_TIM6_DAC_IRQHandler(void);
void MB_TIM7_IRQHandler(void);

void Update_Holding_Register_value(unsigned int data, unsigned int position);
void Update_SLAVE_ID(uint32_t new_slave_id);
int Check_Holding_Register_Value(unsigned int position);
void Check_for_NEW_SLAVE_ID(void);

#endif /* MODBUS_DRIVER_MB_PROCESSOR_H_ */
