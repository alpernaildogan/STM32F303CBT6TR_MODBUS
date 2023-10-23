/*
 * MB_Processor.c
 *
 *  Created on: Mar 9, 2022
 *      Author: Alper
 *
 *  See MB_Processor.h to configure and use!
 *
 */
#include "MB_Processor.h"
#include "FLASH_PAGE_F1.h"

unsigned char MY_SLAVE_ID;
volatile unsigned char ResponseFrameSize;
volatile unsigned char data_in[256u]; //character array that will contain the received data
volatile unsigned int DataPos;
volatile unsigned int TotalCharsReceived;

unsigned int HoldingRegSize;
unsigned int InputRegSize;
unsigned int CoilsRegsize;
unsigned int DiscreteInputRegsize;

//Diagnostic counters
unsigned int BusMsgCount;		//Bus Message Count
unsigned int BusCommErrCount;	//Bus Communication Error Count
unsigned int SlaveExErrCount;	//Bus Exception Error Count
unsigned int SlaveMsgCount;		//Slave Message Count
unsigned int SlaveNoRspCount;	//Slave No Response Count
unsigned int SlaveNAKCount;		//Slave NAK Count
unsigned int SlaveBusyCount;	//Slave Busy Count
unsigned int BusChrOvrCount;	//Bus Character Overrun Count

unsigned int CharCount;

//Holding Register Array
static unsigned int HoldingRegisters[100] = { [0 ... 99] = 9999 };

//Input Register Array
static unsigned int InputRegisters[100] = { [0 ... 99] = 9999 };

//Coil Array
static unsigned char Coils[100] = { [0 ... 99] = 99 };

//Discrete Input Array
static unsigned char DiscreteInputs[100] = { [0 ... 99] = 99 };

// Global variable for modbus SLAVE ID
uint32_t SLAVE_ID = 1;
uint32_t *SLAVE_ID_PTR = &SLAVE_ID;

void MB_Init(void) {
	/*
	 * @brief Init function of modbus
	 *
	 * @note
	 *
	 * @param SLAVE ID is the slave ıd of the modbus slave
	 *
	 * @retval None
	 *
	 * */

	//create a pointer for flash read
	uint32_t number;
	uint32_t *ptr = &number;

	//Read the ID from flash and check
	Flash_Read_Data (0x08012000, ptr, 1);
	SLAVE_ID = number;
	if (SLAVE_ID > 0 && SLAVE_ID < 100) { // check if the slave id is in the correct range

	} else {
		SLAVE_ID = 1;
	}


	//Start MODBUS init
	MY_SLAVE_ID = SLAVE_ID; // Slave ID is the address of the slave
	ClearModbusCounters(); // Clear all error counters
	HoldingRegSize = (sizeof(HoldingRegisters) / sizeof(HoldingRegisters[0])); // Size of data arrays
	InputRegSize = (sizeof(InputRegisters) / sizeof(InputRegisters[0]));
	CoilsRegsize = (sizeof(Coils) / sizeof(Coils[0]));
	DiscreteInputRegsize = (sizeof(DiscreteInputs) / sizeof(DiscreteInputs[0]));

	/* USER CODE BEGIN TIM6_Init 2 */
	TIM6->DIER |= 1U;	// ENABLE COUNTER OVER FLOW INTERRUPT
	TIM6->SR = 0U;		// CLEAR INTERRUPT BIT
	TIM6->CR1 &= ~(1U);	// STOP TIMER
	/* USER CODE END TIM6_Init 2 */

	/* USER CODE BEGIN TIM7_Init 2 */
	TIM7->DIER |= 1U;	// ENABLE COUNTER OVER FLOW INTERRUPT
	TIM7->SR = 0U;		// CLEAR INTERRUPT BIT
	TIM7->CR1 &= ~(1U);	// STOP TIMER
	/* USER CODE END TIM7_Init 2 */

	/* USER CODE BEGIN USART3_Init 2 */
	USART3->CR1 |= USART_CR1_RXNEIE; //By default usart receive interrupt is not enabled
	/* USER CODE END USART3_Init 2 */

}


void CheckMBPDU() {
	/*
	 * @brief Starting Modbus function to check if the PDU(protocol data unit) is received
	 *
	 * @note
	 *
	 * @param None
	 *
	 * @retval None
	 *
	 * */
	CharCount = 0;
	CharCount = TotalCharsReceived;
	TotalCharsReceived = 0;
	if (CharCount >= 4u) {
		//Check inbound frame CRC
		unsigned int crcvalue = CRC16(data_in, (CharCount - 2));

		if ((data_in[CharCount - 2] == (unsigned char) (crcvalue)) & //lower byte at higher register
				(data_in[CharCount - 1] == (unsigned char) (crcvalue >> 8))) //higher byte at lower register
				{
			BusMsgCount += 1;	//increment bus message counter
			if ((data_in[0] == MY_SLAVE_ID) | (data_in[0] == 0u)) {
				SlaveMsgCount += 1; //Increment Slave message count
				//STEP 2: Check function code
				SlaveNoRspCount += data_in[0] == 0 ? 1 : 0; // this is a ternary operator, if the slave ID is zero then NoRspCount increases by one
				switch (data_in[1]) {
				case 0x01:
					MBProcessBitsRead(Coils, CoilsRegsize); //read coils
					break;

				case 0x02:
					MBProcessBitsRead(DiscreteInputs, DiscreteInputRegsize); //read discrete inputs
					break;

				case 0x03:
					MBProcessRegisterRead(HoldingRegisters, HoldingRegSize); //read holding register
					break;

				case 0x04:
					MBProcessRegisterRead(InputRegisters, InputRegSize); //read input register
					break;

				case 0x05:
					MBForceSingleCoil(Coils, CoilsRegsize);	//Write single coil
					break;

				case 0x06:
					MBPresetSingleRegister(HoldingRegisters, HoldingRegSize);//write single register
					break;

				case 0x08:
					MBProcessDiagnostics();
					break;

				case 0x0f:
					MBForceMultipleCoils(Coils, CoilsRegsize);//Write multiple coils
					break;

				case 0x10: // i.e. function 16
					MBPresetMultipleRegisters(HoldingRegisters, HoldingRegSize);//write multiple registers
					break;

				default: {
					MBException(0x01); //Illegal function code 01
					MBSendData(ResponseFrameSize); //send data if not broadcast command
				}
					break;
				}
			}
		} else
			BusCommErrCount += 1; //Increment bus communication error counter
	} else
		BusCommErrCount += 1; //Increment bus communication error counter
}


unsigned int MBRegisterCount(void) {
	/*
	 * @brief Count the number of registers
	 *
	 * @note
	 *
	 * @param None
	 *
	 * @retval TODO
	 *
	 * */
	return (data_in[5] | (data_in[4] << 8));
}

unsigned int MBStartAddress(void)	//Return requested start address
{
	/*
	 * @brief Get Starting Modbus Address
	 *
	 * @note
	 *
	 * @param None
	 *
	 * @retval TODO
	 *
	 * */
	return (data_in[3] | (data_in[2] << 8));
}


void MBSendData(unsigned char count)
{
	/*
	 * @brief Send data over USART
	 *
	 * @note
	 *
 	 * @param TODO
	 *
	 * @retval None
	 *
	 * */
	if (data_in[0] != 0) {
		for (unsigned char c = 0; c < count; c++) {
			while (!( USART3->ISR & (1 << 7u))) {
			};	//wait till transmit buffer is empty
			USART3->TDR = data_in[c];
		}
	}
}

/*************** Append CRC ***************/
//
void AppendCRCtoMBRegister(unsigned char packtop)//crc is calculated from slave id to last data byte
{
	/*
	 * @brief C function to append CRC to Slave Modbus response PDU(protocol data unit)
	 *
	 * @note
	 *
	 * @param TODO
	 *
	 * @retval None
	 *
	 * */
	unsigned short crcvalue = 0;
	crcvalue = CRC16(data_in, packtop);
	data_in[packtop] = (unsigned char) (crcvalue);//lower byte at higher register
	data_in[packtop + 1] = (unsigned char) (crcvalue >> 8);	//higher byte at lower register
	ResponseFrameSize = packtop + 2;
}
//************** Append CRC ***************

//**************  EXCEPTION  **************

void MBException(unsigned char exceptionCode)	//Exception code
{
	/*
	 * @brief Write Exception code
	 *
	 * @note
	 *
	 * @param TODO
	 *
	 * @retval None
	 *
	 * */
	SlaveExErrCount += 1;			//Increment Slave Exception Error count
	data_in[1] |= 0x80;	//setting MSB of the function code (the exception flag)
	data_in[2] = exceptionCode; //Exception code. Also the last byte containing dat
	ResponseFrameSize = 3;		// 3 bytes to send. No crc calculation.
}
//**************  EXCEPTION  **************

//*********Modbus Register Read Operations*************

void MBProcessRegisterRead(unsigned int *InArr, unsigned int InArrSize) {
	/*
	 * @brief C function related to Analog Read operations (Function code 03, 04)
	 *
	 * @note
	 *
	 * @param TODO
	 *
	 * @retval None
	 *
	 * */
	//First check what is the count of registers requested
	unsigned int RegCount = MBRegisterCount();

	//| SLAVE_ID | FUNCTION_CODE | RETURN BYTES COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE            | N*2 BYTES | 2 BYTES |
	//The maximum size of a MODBUS RTU frame is 256 bytes. Therefore the final data should fit in above 256 size, so data should be max 251 bytes

	if ((RegCount >= 1u) & (RegCount * 2 <= 251u) & (RegCount <= 125u)) {
		//to check if the requested start and end addresses are available in the microcontroller
		//Get to know the starting address of the requested data
		unsigned int StAddress = MBStartAddress();
		unsigned int EndAddress = StAddress + RegCount - 1u;

		//We will simply check if the end address is inside the size of our holding register
		if ((StAddress >= 0u) & (EndAddress <= (InArrSize - 1u))) {
			//Process the request
			data_in[2] = (unsigned char) (RegCount * 2);//fill the byte count in the data array, every data is 2bytes long that is why *2
			AppendDatatoMBRegister(StAddress, RegCount, HoldingRegisters,
					data_in);	//fill data in the data register
			AppendCRCtoMBRegister(3 + RegCount * 2);
		} else
			MBException(0x02);	//Exception code 02 = ILLEGAL DATA ADDRESS
	} else
		MBException(0x03);	//Exception code 03 = ILLEGAL DATA VALUE

	MBSendData(ResponseFrameSize);		//send response if not broadcast message

}


void AppendDatatoMBRegister(unsigned int StAddr, unsigned int count,
		unsigned int *inreg, volatile unsigned char *outreg) {
	/*
	 * @brief Append Data from unsigned integer Array to Modbus PDU during Read Operation
	 *
	 * @note
	 *
	 * @param TODO
	 *
	 * @retval None
	 *
	 * */
	for (unsigned char c = 0; c < count; c++) {
		*(outreg + 3 + c * 2) = (unsigned char) (*(inreg + StAddr + c) >> 8);//MSB IN HIGHER BYTE
		*(outreg + 3 + (c * 2 + 1)) = (unsigned char) (*(inreg + StAddr + c));//LSB IN LOWER BYTE
	}
}
//*********Modbus Register Read Operations*************

//*********Modbus Register Write Operations*************

void MBPresetSingleRegister(unsigned int *InArr, unsigned int InArrSize) {
	/*
	 * @brief C function to Write Single Analog register (Function code 06)
	 *
	 * @note
	 *
	 * @param TODO
	 *
	 * @retval None
	 *
	 * */
	unsigned int StAddress = MBStartAddress();
	//exception code 03 cannot be done for function code 06
	//check of exception code 02
	if ((StAddress) <= (InArrSize - 1u)) {
		*(InArr + StAddress) = (unsigned int) (data_in[4] << 8)
				| (unsigned int) (data_in[5]);	//MSB FIRST AND THEN LSB IS ORed
		ResponseFrameSize = CharCount;
	} else
		MBException(0x02);	//Exception code 02 = ILLEGAL DATA ADDRESS

	MBSendData(ResponseFrameSize);		//send response if not broadcast message

	// check if the slave id has changed
	Check_for_NEW_SLAVE_ID();
}

void MBPresetMultipleRegisters(unsigned int *InArr, unsigned int InArrSize) {
	/*
	 * @brief C function to Write Multiple Analog registers (Function code 16)
	 *
	 * @note
	 *
	 * @param TODO
	 *
	 * @retval None
	 *
	 * */

	//Get to know the starting address of the requested data
	unsigned int StAddress = MBStartAddress();
	unsigned int RegCount = MBRegisterCount();
	unsigned int EndAddress = StAddress + RegCount - 1;

	//| SLAVE_ID | FUNCTION_CODE | RETURN BYTES COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE            | N*2 BYTES | 2 BYTES |
	//So our final requested data should fit in above 256 size, so data should be max 256-6 bytes
	//As a safeguard we are also checking with maximum limits of query as per modbus function (m584 controller)
	if ((RegCount >= 1u) & (RegCount <= 100u) & (data_in[6] == RegCount * 2))//for fc16, number of bytes requested is embedded in modbus message
			{
		//We will simply check if the end address is inside the size of our holding register
		if ((StAddress >= 0) & (EndAddress <= (InArrSize - 1u))) {
			//Process the request
			WriteMBRegistertoData(StAddress, RegCount, data_in,
					HoldingRegisters);
			ResponseFrameSize = CharCount;
		} else
			MBException(0x02);	//Exception code 02 = ILLEGAL DATA ADDRESS
	} else
		MBException(0x03);	//Exception code 03 = ILLEGAL DATA VALUE

	MBSendData(ResponseFrameSize);		//send response if not broadcast message

	// check if the slave id has changed
	Check_for_NEW_SLAVE_ID();
}

void WriteMBRegistertoData(unsigned int StAddr, unsigned int count,
		volatile unsigned char *inreg, unsigned int *outreg) {
	/*
	 * @brief C function to Store Analog Write operations to unsigned integer Variable array
	 *
	 * @note
	 *
	 * @param TODO
	 *
	 * @retval None
	 *
	 * */
	for (unsigned char c = 0; c < count; c++)
		*(outreg + StAddr + c) = (unsigned int) (*(inreg + 7 + (c * 2)) << 8)
				| (unsigned int) (*(inreg + 7 + (c * 2) + 1));
}
//*********Modbus Register Write Operations*************

//*********Modbus Discrete Read Operations*************

void MBProcessBitsRead(unsigned char *InArr, unsigned int InArrSize) {
	/*
	 * @brief C function related to Discrete Read operations (Function code 01, 02)
	 *
	 * @note
	 *
	 * @param TODO
	 *
	 * @retval None
	 *
	 * */
	//First check what is the count of bits requested
	unsigned int BitCount = MBRegisterCount();
	//As a safeguard we are also checking with maximum limits of query as per modbus function (m584 controller)

	unsigned char ByteCount = (
			(BitCount <= 8u) ?
					1u :
					((BitCount % 8u == 0) ?
							(BitCount / 8u) : ((BitCount / 8u) + 1u)));
	//| SLAVE_ID | FUNCTION_CODE | RETURN BYTES COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE            | N BYTES   | 2 BYTES |
	if ((BitCount >= 1u) & (BitCount <= 2000u)) {
		//to check if the requested start and end addresses are available in out controller
		//As an example we configure 50 holding registers = 100 bytes of data array HoldingRegister
		//Get to know the starting address of the requested data
		unsigned int StAddress = MBStartAddress();			//start coil address
		unsigned int EndAddress = StAddress + BitCount - 1;	//end coil address

		if (EndAddress <= ((InArrSize * 8) - 1)) {
			//Process the request
			data_in[2] = (unsigned char) ByteCount;	//fill the byte count in the data array

			//Ex. if master request 4 coil statuses, this means that 1 register is response.
			//We need to clear the remaining 4 bits of the data if there is any.
			//Else there will be error
			unsigned char regtoclear = (
					(BitCount < 8u) ?
							3u :
							((BitCount % 8u > 0) ?
									((unsigned char) (BitCount / 8u + 3u)) : 0));
			//clearing last byte of response array
			if (regtoclear > 0)
				data_in[(unsigned char) regtoclear] = 0x00;

			AppendBitsToRegisters(StAddress, BitCount, InArr, data_in);
			AppendCRCtoMBRegister(3 + ByteCount);
		} else
			MBException(0x02);		//Exception code 02 = ILLEGAL DATA ADDRESS
	} else
		MBException(0x03);			//Exception code 03 = ILLEGAL DATA VALUE

	MBSendData(ResponseFrameSize);		//send response if not broadcast message
}


void AppendBitsToRegisters(unsigned int StAddr, unsigned int count,
		unsigned char *inreg, volatile unsigned char *outreg) {
	/*
	 * @brief C function to append array bits to modbus pDU
	 *
	 * @note
	 *
	 * @param TODO
	 *
	 * @retval None
	 *
	 * */
	for (unsigned int c = 0; c < count; c++) {
		if (*(inreg + ((StAddr + c) / 8u))
				& (1 << ((StAddr + c) - (((StAddr + c) / 8u) * 8u))))//if in outreg array, bit is 1?
				{
			*(outreg + 3 + (c / 8)) |= (1 << (c - ((c / 8) * 8)));//then set bit 1 in target array
		} else
			*(outreg + 3 + (c / 8)) &= ~(1 << (c - ((c / 8) * 8)));	//else clear the bit in target array
	}
}
//*********Modbus Discrete Read Operations*************

//*********Modbus Discrete Write Operations*************

void MBForceSingleCoil(unsigned char *InArr, unsigned int InArrSize) {
	/*
	 * @brief C function for writing one coil (Function code 05)
	 *
	 * @note
	 *
	 * @param TODO
	 *
	 * @retval None
	 *
	 * */
	unsigned int StAddress = MBStartAddress();
	unsigned int outputvalue = MBRegisterCount();//using existing c function instead of creating a new one
	if ((outputvalue == 0x0000) | (outputvalue == 0xff00)) {
		if (StAddress <= ((InArrSize * 8) - 1)) {
			//Not calling separate function. Writing here itself
			if (outputvalue == 0xff00) {
				*(InArr + (StAddress / 8)) |= (1u
						<< (StAddress - ((StAddress / 8) * 8)));
			} else
				*(InArr + (StAddress / 8)) &= ~(1u
						<< (StAddress - ((StAddress / 8) * 8)));
			ResponseFrameSize = CharCount;
		} else
			MBException(0x02);		//Exception code 02 = ILLEGAL DATA ADDRESS
	} else
		MBException(0x03);		//Exception code 03 = ILLEGAL DATA VALUE

	MBSendData(ResponseFrameSize);		//send response if not broadcast message
}


void MBForceMultipleCoils(unsigned char *InArr, unsigned int InArrSize) {
	/*
	 * @brief C function for writing multiple coils (Function code 15)
	 *
	 * @note
	 *
	 * @param TODO
	 *
	 * @retval None
	 *
	 * */
	//First check what is the count of bits requested
	unsigned int BitCount = MBRegisterCount();
	unsigned int StAddress = MBStartAddress();

	//As a safeguard we are also checking with maximum limits of query as per modbus function (m584 controller)
	unsigned char ByteCount = (
			(BitCount <= 8u) ?
					1u :
					((BitCount % 8u == 0) ?
							(BitCount / 8u) : ((BitCount / 8u) + 1u)));
	//| SLAVE_ID | FUNCTION_CODE | RETURN BYTES COUNT | DATA      | CRC     |
	//| 1 BYTE   |  1 BYTE       |  1 BYTE            | N BYTES   | 2 BYTES |
	if ((BitCount >= 1u) & (BitCount <= 800u) & (ByteCount == data_in[6])) {
		//to check if the requested start and end addresses are available in out controller
		//As an example we configure 50 holding registers = 100 bytes of data array HoldingRegister
		//Get to know the starting address of the requested data
		//start coil address
		unsigned int EndAddress = StAddress + BitCount - 1;	//end coil address

		if ((StAddress >= 0) & (EndAddress <= ((InArrSize * 8) - 1))) {
			//Process the requests
			WriteMBRegistersToBits(StAddress, BitCount, data_in, InArr);
			ResponseFrameSize = CharCount;
		} else
			MBException(0x02);	//Exception code 02 = ILLEGAL DATA ADDRESS
	} else
		MBException(0x03);	//Exception code 03 = ILLEGAL DATA VALUE

	MBSendData(ResponseFrameSize);		//send response if not broadcast message
}

void WriteMBRegistersToBits(unsigned int StAddr, unsigned int count,
		volatile unsigned char *inreg, unsigned char *outreg) {
	/*
	 * @brief C function to store MODBUS master's discrete signals to Coil array
	 *
	 * @note
	 *
	 * @param TODO
	 *
	 * @retval None
	 *
	 * */
	for (unsigned int c = 0; c < count; c++) {
		if (*(inreg + 7 + (c / 8u)) & (1 << (c - ((c / 8u) * 8u))))	//if in outreg array, bit is 1?
				{
			*(outreg + ((StAddr + c) / 8)) |= (1u
					<< ((StAddr + c) - (((StAddr + c) / 8) * 8)));
		} else
			*(outreg + ((StAddr + c) / 8)) &= ~(1u
					<< ((StAddr + c) - (((StAddr + c) / 8) * 8)));//else clear the bit in target array
	}
}
//*********Modbus Discrete Write Operations*************

void MBProcessDiagnostics(void) {
	/*
	 * @brief TODO
	 *
	 * @note
	 *
	 * @param None
	 *
	 * @retval None
	 *
	 * */
	unsigned int subcode = MBStartAddress();//using existing c function instead of creating new one
	unsigned int Data = MBRegisterCount();	//using existing c function
	switch (subcode) {
	case 0x00:			//Return Query Data
		ResponseFrameSize = CharCount;
		break;

	case 0x01:			//Restart Communications Option
		break;

	case 0x02://Return Diagnostic Register - No diagnostic register in modbus slave
		break;

	case 0x03://Change ASCII Input Delimiter - Not for our application as we are using Modbus RTU
		break;

	case 0x04:			//Force Listen Only Mode
		break;

	case 0x0a:			//Clear Counters and Diagnostic Register
		if (Data == 0x0000) {
			ClearModbusCounters();
			ResponseFrameSize = CharCount;
		} else
			MBException(0x03);			//Exception code 03 = ILLEGAL DATA VALUE
		break;

	case 0x0b:			//Return Bus Message Count
	case 0x0c:			//Return Bus Communication Error Count
	case 0x0d:			//Return Bus Exception Error Count
	case 0x0e:			//Return Slave Message Count
	case 0x0f:			//Return Slave No Response Count
	case 0x10:			//Return Slave NAK Count
	case 0x11:			//Return Slave Busy Count
	case 0x012:			//Return Bus Character Overrun Count
		if (Data == 0x0000) {
			unsigned int counterval = ReturnDiagCounter(subcode);
			data_in[4] = (counterval << 8);
			data_in[5] = (counterval);
			ResponseFrameSize = CharCount;
		} else
			MBException(0x03);			//Exception code 03 = ILLEGAL DATA VALUE
		break;

	case 0x14:			//Clear Overrun Counter and Flag
		if (Data == 0x0000) {
			BusChrOvrCount = 0;
			ResponseFrameSize = CharCount;
		} else
			MBException(0x03);			//Exception code 03 = ILLEGAL DATA VALUE
		break;

	default: {
		MBException(0x01); //Illegal function code 01
		MBSendData(ResponseFrameSize);		//send data if not broadcast command
		break;
	}

	}
	MBSendData(CharCount);	//echo back the data
}

unsigned int ReturnDiagCounter(unsigned int scode) {
	/*
	 * @brief TODO
	 *
	 * @note
	 *
	 * @param TODO
	 *
	 * @retval None
	 *
	 * */
	switch (scode) {
	case 0x0b:
		return BusMsgCount;
		break;

	case 0x0c:
		return BusCommErrCount;
		break;

	case 0x0d:
		return SlaveExErrCount;
		break;

	case 0x0e:
		return SlaveMsgCount;
		break;

	case 0x0f:
		return SlaveNoRspCount;
		break;

	case 0x10:
		return SlaveNAKCount;
		break;

	case 0x11:
		return SlaveBusyCount;
		break;

	case 0x12:
		return BusChrOvrCount;
		break;
	}
	return 0;
}
void ClearModbusCounters() {
	/*
	 * @brief TODO
	 *
	 * @note
	 *
	 * @param None
	 *
	 * @retval None
	 *
	 * */
	BusMsgCount = 0;		//Bus Message Count
	BusCommErrCount = 0; 	//Bus Communication Error Count
	SlaveExErrCount = 0;	//Bus Exception Error Count
	SlaveMsgCount = 0;		//Slave Message Count
	SlaveNoRspCount = 0;	//Slave No Response Count
	SlaveNAKCount = 0;		//Slave NAK Count
	SlaveBusyCount = 0;		//Slave Busy Count
	BusChrOvrCount = 0;		//Bus Character Overrun Count
}

void Update_Holding_Register_value(unsigned int data, unsigned int position) {
	/*
	 * @brief update holding reg data
	 *
	 * @note
	 *
	 * @param data is the data to be saved, and position is the position in the holding register
	 *
	 * @retval None
	 *
	 * */

	*(HoldingRegisters + position) = data;
}

int Check_Holding_Register_Value(unsigned int position) {
	/*
	 * @brief Holding register value check function
	 *
	 * @note
	 *
	 * @param Position is the positon of the data to be retrieved.
	 *
 	 * @retval Value that stored in the respective position on the holding register
	 *
	 * */
	int data = *(HoldingRegisters + position);
	return data;
}

void Check_for_NEW_SLAVE_ID(void){
	/*
	 * @brief TODO
	 *
	 * @note
	 *
	 * @param
	 *
 	 * @retval
	 *
	 * */

	uint32_t desired_ID;

	// third position(starting from zero) should hold the desired slave ID
	desired_ID = Check_Holding_Register_Value(2);

	if (desired_ID != SLAVE_ID && desired_ID < 100 && desired_ID > 0) { // SLAVE ID should be between 1 and 100
		Update_SLAVE_ID(desired_ID);
		return;
	} else {
		return;
	}
}

void Update_SLAVE_ID(uint32_t NEW_SLAVE_ID){
	/*
	 * @brief This function is used for changing the slave id. The new id will be saved to the flash
	 *
	 * @note
	 *
	 * @param NEW_SLAVE_ID is the new id
	 *
 	 * @retval None
	 *
	 * */

	uint32_t number;
	uint32_t *ptr2 = &number;

	//Change the SLAVE_ID
	SLAVE_ID = NEW_SLAVE_ID;
	number = NEW_SLAVE_ID;
	//Save the new ID to the EEPROM
	Flash_Write_Data(0x08012000, ptr2 , 1);
	//Re init with the new ID, if fails the id number will be one
	MB_Init();
}

void MB_USART3_IRQHandler(void) {
	/*
	 * @brief USART3 interrupt handler for modbus
	 *
	 * @note TODO this has become usart3 edit!!!
	 *
	 * @param None
	 *
	 * @retval None
	 *
	 * */
	/* USER CODE BEGIN USART3_IRQn 0 */
	TIM6->CR1 &= ~TIM_CR1_CEN;	//Disable timer6 counter
	TIM7->CR1 &= ~TIM_CR1_CEN;	//Disable timer7 counter
	TIM6->CNT = 0U;		//clear counters
	TIM7->CNT = 0U;		//clear

	//Check if there is an overrun error
	if (USART3->ISR & USART_ISR_ORE) { //TODO usart3 olmalı!
		BusChrOvrCount += 1;
		USART3->ICR |= USART_ICR_ORECF;	//CLEAR OVERRUN ERROR FLAG
		//We need to test this overrun feature

	}
	if (DataPos >= 256) //The maximum size of a MODBUS RTU frame is 256 bytes.
			{
		DataPos = 0;
	} else {
		data_in[DataPos] = USART3->RDR;
		DataPos += 1;
		TotalCharsReceived = DataPos;
	}
	/* USER CODE END USART3_IRQn 0 */
	//HAL_UART_IRQHandler(&huart1);
	/* USER CODE BEGIN USART3_IRQn 1 */
	TIM6->CR1 |= TIM_CR1_CEN;	//Enable timers
	TIM7->CR1 |= TIM_CR1_CEN;
	/* USER CODE END USART3_IRQn 1 */
}

void MB_TIM6_DAC_IRQHandler(void) {
	/*
	 * @brief TIM6 interrupt handler for modbus
	 *
	 * @note
	 *
	 * @param None
	 *
	 * @retval None
	 *
	 * */
	/* USER CODE BEGIN TIM6_DAC_IRQn 0 */
	if (TIM_SR_UIF == TIM6->SR) {
		TIM6->CR1 &= ~(TIM_CR1_CEN);	//STOP THE 1.5CHAR TIMER
		TIM6->CNT = 0U;					//CLEAR COUNTER
		DataPos = 0u;					//clear the Indexer and start over
		TIM6->SR &= ~TIM_SR_UIF;// clear the 1.5 char timer interrupt (if any)
	}
	/* USER CODE END TIM6_DAC_IRQn 0 */
	//HAL_TIM_IRQHandler(&htim6);
	/* USER CODE BEGIN TIM6_DAC_IRQn 1 */

	/* USER CODE END TIM6_DAC_IRQn 1 */

}

void MB_TIM7_IRQHandler(void) {
	/*
	 * @brief TIM7 interrupt handler for modbus
	 *
	 * @note
	 *
	 * @param None
	 *
	 * @retval None
	 *
	 * */
	/* USER CODE BEGIN TIM7_IRQn 0 */
	if (TIM_SR_UIF == TIM7->SR) {
		TIM7->CR1 &= ~(TIM_CR1_CEN);	//Stop 3.5 char timer
		TIM7->SR &= ~TIM_SR_UIF;		//Clear the interrupt flag
		DataPos = 0u;					//clear the Indexer
		CheckMBPDU();					//Process Modbus frame
	}
	/* USER CODE END TIM7_IRQn 0 */
	//HAL_TIM_IRQHandler(&htim7);
	/* USER CODE BEGIN TIM7_IRQn 1 */

	/* USER CODE END TIM7_IRQn 1 */

}

