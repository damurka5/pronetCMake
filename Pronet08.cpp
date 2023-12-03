#include "Pronet08.h"

#define ERRORCOUNTER 2


Pronet08::Pronet08(int comPort, int baudRate, int byteSize,
	int stopBits, int parity) : ModbusRtu(comPort, baudRate, byteSize, stopBits, parity)
{

}

Pronet08::~Pronet08()
{

}

int Pronet08::clearCurrentAlarm(int id)
{
	int ans = modbusWriteRegister(id, 0x1022, 0x0001);
	if (ans != 0) {
		return ans;
	}

	return 0;
}

int Pronet08::clearEncoderAlarm(int id)
{
	int ans = modbusWriteRegister(id, 0x1040, 0x0001); // Clear encoder alarm
	if (ans != 0) {
		return ans;
	}
	ans = modbusWriteRegister(id, 0x1022, 0x0001); // Clear current alarm
	if (ans != 0) {
		return ans;
	}

	return 0;
}

int Pronet08::clearEncoderMultiTurnData(int id)
{
	int ans = modbusWriteRegister(id, 0x1041, 0x0001);
	if (ans != 0) {
		return ans;
	}

	return 0;
}

int Pronet08::resetCommunicationSettings(int id)
{
	int ans = modbusWriteRegister(id, 0x02BC, 0x0151);
	if (ans != 0) {
		return ans;
	}

	return 0;
}

int Pronet08::writeRegister(int id, int reg, int data)
{
	int ans = modbusWriteRegister(id, reg, data);
	if (ans != 0) {
		return ans;
	}

	return 0;
}

int Pronet08::readRegister(int id, int reg, int amount, uint16_t* dataBuf)
{
	uint16_t rxBuffer[255] = { 0, };

	int ans = modbusReadRegisters(id, reg, amount, rxBuffer);
	if (ans == 0) {
		dataBuf[0] = rxBuffer[0];
		dataBuf[1] = rxBuffer[1];
		dataBuf[2] = rxBuffer[2];
		dataBuf[3] = rxBuffer[3];
		return 0;
	}
	else {
		return ans;
	}
}

int Pronet08::servoOn(int id)
{
	int ans = modbusWriteRegister(id, 0x1023, 0x0001);
	if (ans != 0) {
		return ans;
	}

	return 0;
}

int Pronet08::servoOff(int id)
{
	stopRotation(id);

	int ans = modbusWriteRegister(id, 0x1023, 0x0000);
	if (ans != 0) {
		return ans;
	}
	
	return 0;
}

int Pronet08::forwardStart(int id)
{
	int ans = modbusWriteRegister(id, 0x1024, 0x0001);
	if (ans != 0) {
		return ans;
	}

	return 0;
}

int Pronet08::reverseStart(int id)
{
	int ans = modbusWriteRegister(id, 0x1025, 0x0001);
	if (ans != 0) {
		return ans;
	}

	return 0;
}

int Pronet08::stopRotation(int id)
{
	int ans = modbusWriteRegister(id, 0x1024, 0x0000);
	if (ans != 0) {
		return ans;
	}
	ans = modbusWriteRegister(id, 0x1025, 0x0000);
	if (ans != 0) {
		return ans;
	}

	return 0;
}

int Pronet08::setSpeed(int id, int speed)
{
	int ans = modbusWriteRegister(id, 0x0131, speed);
	if (ans != 0) {
		return ans;
	}

	return 0;
}

int Pronet08::readAlarm(int id, uint16_t* dataBuf)
{
	uint16_t rxBuffer[255] = { 0, };

	int ans = modbusReadRegisters(id, 0x081D, 1, rxBuffer);
	if (ans == 0) {
		dataBuf[0] = rxBuffer[0];
		return 0;
	}
	else {
		return ans;
	}
}

int Pronet08::readActualSpeed(int id, uint16_t* dataBuf)
{
	uint16_t rxBuffer[255] = { 0, };

	int ans = modbusReadRegisters(id, 0x0806, 1, rxBuffer);
	if (ans == 0) {
		dataBuf[0] = rxBuffer[0];
		return 0;
	}
	else {
		return ans;
	}
}

int Pronet08::readSetSpeed(int id, uint16_t* dataBuf)
{
	uint16_t rxBuffer[255] = { 0, };

	int ans = modbusReadRegisters(id, 0x0131, 1, rxBuffer);
	if (ans == 0) {
		dataBuf[0] = rxBuffer[0];
		return 0;
	}
	else {
		return ans;
	}
}

int Pronet08::readActualTorqueReference(int id, uint16_t* dataBuf)
{
	uint16_t rxBuffer[255] = { 0, };

	int ans = modbusReadRegisters(id, 0x0809, 1, rxBuffer);
	if (ans == 0) {
		dataBuf[0] = rxBuffer[0];
		return 0;
	}
	else {
		return ans;
	}
}

int Pronet08::readActualPosition(int id, uint16_t* dataBuf)
{
	uint16_t rxBuffer[255] = { 0, };

	int ans = modbusReadRegisters(id, 0x1010, 3, rxBuffer);
	if (ans == 0) {
		dataBuf[0] = rxBuffer[0];
		dataBuf[1] = rxBuffer[1];
		dataBuf[2] = rxBuffer[2];
		return 0;
	}
	else {
		return ans;
	}
}

int Pronet08::checkAlarm(void)
{
	int ans = 0;
	uint16_t data[1] = { 0, };

	for (auto i = 0; i < ERRORCOUNTER; i++) {
		for (size_t id = 1; id <= 4; id++) {
			ans = readAlarm(id, data);
			if (ans != 0 || data[0] != 0) {
				errCnt++;
			}
		}
	}
	
	if (errCnt >= ERRORCOUNTER) {
		servoOff(0);
		errCnt = 0;
		return 1;
	}

	errCnt = 0;
	return 0;
}

