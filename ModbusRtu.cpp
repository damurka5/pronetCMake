#include "ModbusRtu.h"

ModbusRtu::ModbusRtu(int comPort, int baudRate, int byteSize, 
	                 int stopBits, int parity) : Serial(comPort, baudRate, byteSize, stopBits, parity)
{
}

ModbusRtu::~ModbusRtu()
{
}

int ModbusRtu::modbusConnect(void)
{
	return connectRequest();
}

bool ModbusRtu::modbusClose(void)
{
	return closeSerialPort();
}

int ModbusRtu::modbusReadRegisters(uint8_t id, uint16_t address, uint16_t amount, uint16_t* buffer)
{
	while (isBusy) {}
	

	isBusy = true;

	unsigned char txBuffer[9] = { 0, };
	unsigned char rxBuffer[MAX_MSG_LENGTH] = { 0, };
	DWORD factSize = 0;

	unsigned char txMessage[] =
	{
		id,
		READ_REGS,
		(uint8_t)(address >> 8),
		(uint8_t)(address & 0x00FF),
		(uint8_t)(amount >> 8),
		(uint8_t)(amount & 0x00FF)
	};

	uint16_t crc = crcCalculate(txMessage, sizeof(txMessage));

	txBuffer[0] = txMessage[0];
	txBuffer[1] = txMessage[1];
	txBuffer[2] = txMessage[2];
	txBuffer[3] = txMessage[3];
	txBuffer[4] = txMessage[4];
	txBuffer[5] = txMessage[5];
	txBuffer[6] = (uint8_t)(crc & 0x00FF);
	txBuffer[7] = (uint8_t)(crc >> 8);

	if (!writeReadSerialPort(txBuffer, rxBuffer[0], MAX_MSG_LENGTH, factSize)) {
		isBusy = false;
		return -1;
	};

	if (factSize != 0) {
		crc = crcCalculate(rxBuffer, factSize - 2);
		uint16_t rxCrc = (rxBuffer[factSize - 1] << 8) | (rxBuffer[factSize - 2]);
		if (rxCrc != crc) {
			isBusy = false;
			return -2;
		}
		else {
			for (size_t i = 0; i < rxBuffer[2] / 2; i++) {
				buffer[i] = ((rxBuffer[i + i + 3]) << 8) | (rxBuffer[i + i + 4]);
			}
			isBusy = false;
		}
	}
	else {
		isBusy = false;
		return -3;
	}

	return 0;
}

int ModbusRtu::modbusWriteRegister(uint8_t id, uint16_t address, const uint16_t& value)
{
	while (isBusy) {}
		
	isBusy = true;

	int result = 0;
	unsigned char txBuffer[9] = { 0, };
	unsigned char rxBuffer[MAX_MSG_LENGTH] = { 0, };
	DWORD factSize = 0;

	unsigned char txMessage[] =
	{
		id,
		WRITE_REG,
		(uint8_t)(address >> 8),
		(uint8_t)(address & 0x00FF),
		(uint8_t)(value >> 8),
		(uint8_t)(value & 0x00FF)
	};

	uint16_t crc = crcCalculate(txMessage, 6);

	txBuffer[0] = txMessage[0];
	txBuffer[1] = txMessage[1];
	txBuffer[2] = txMessage[2];
	txBuffer[3] = txMessage[3];
	txBuffer[4] = txMessage[4];
	txBuffer[5] = txMessage[5];
	txBuffer[6] = (uint8_t)(crc & 0x00FF);
	txBuffer[7] = (uint8_t)(crc >> 8);

	if (!writeReadSerialPort(txBuffer, rxBuffer[0], MAX_MSG_LENGTH, factSize)) {
		isBusy = false;
		return -1;
	};
	
	if (id != 0) {
		if (compare_arrays(txBuffer, rxBuffer, sizeof(txBuffer)) != true) {
			isBusy = false;
			return -2;
		}
	}
	isBusy = false;

	return 0;
}

uint16_t ModbusRtu::crcCalculate(unsigned char* data, char lenght)
{
	int i, j;
	uint16_t crc_reg = 0xFFFF;
	
	while (lenght--) {
		crc_reg ^= *data++;
		for (j = 0; j < 8; j++) {
			if (crc_reg & 0x01) {
				crc_reg = (crc_reg >> 1) ^ 0xA001;
			}
			else {
				crc_reg = crc_reg >> 1;
			}
		}
	}	
	return crc_reg;
}

bool ModbusRtu::compare_arrays(unsigned char* a, unsigned char* b, size_t count) {
	for (size_t i = 0; i < count; ++i)
		if (a[i] != b[i])
			return false;
	return true;
}