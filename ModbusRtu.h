#pragma once
#include "Serial.h"

#define MAX_MSG_LENGTH 255

//Function Code
#define READ_REGS 0x03
#define WRITE_REG 0x06

//Exception Codes
#define EX_ILLEGAL_FUNCTION 0x01 // Function Code not Supported
#define EX_ILLEGAL_ADDRESS 0x02  // Output Address not exists
#define EX_ILLEGAL_VALUE 0x03    // Output Value not in Range
#define EX_SERVER_FAILURE 0x04   // Slave Deive Fails to process request
#define EX_ACKNOWLEDGE 0x05      // Service Need Long Time to Execute
#define EX_SERVER_BUSY 0x06      // Server Was Unable to Accept MB Request PDU
#define EX_NEGATIVE_ACK 0x07
#define EX_MEM_PARITY_PROB 0x08
#define EX_GATEWAY_PROBLEMP 0x0A // Gateway Path not Available
#define EX_GATEWAY_PROBLEMF 0x0B // Target Device Failed to Response
#define EX_BAD_DATA 0XFF         // Bad Data lenght or Address

class ModbusRtu: public Serial
{
public:
	ModbusRtu(int comPort, int baudRate, int byteSize = 8, 
		int stopBits = TWOSTOPBITS, int parity = NOPARITY);
	~ModbusRtu();

	int modbusConnect(void);
	bool modbusClose(void);
	int modbusReadRegisters(uint8_t id, uint16_t address, uint16_t amount, uint16_t* buffer);
	int modbusWriteRegister(uint8_t id, uint16_t address, const uint16_t& value);


private:
	
	bool isBusy = false;

	bool compare_arrays(unsigned char* a, unsigned char* b, size_t count);
	uint16_t crcCalculate(unsigned char* data, char lenght);
};

