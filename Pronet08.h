#pragma once
#include "ModbusRtu.h"

class Pronet08: public ModbusRtu
{
public:
	Pronet08(int comPort, int baudRate, int byteSize = 8,
		int stopBits = TWOSTOPBITS, int parity = NOPARITY);
	~Pronet08();

	int clearCurrentAlarm(int id);
	int clearEncoderAlarm(int id);
	int clearEncoderMultiTurnData(int id);
	int resetCommunicationSettings(int id);
	int writeRegister(int id, int reg, int data);
	int readRegister(int id, int reg, int amount, uint16_t* dataBuf);
	int servoOn(int id);
	int servoOff(int id);
	int forwardStart(int id);
	int reverseStart(int id);
	int stopRotation(int id);
	int setSpeed(int id, int speed);
	int readAlarm(int id, uint16_t* dataBuf);
	int readActualSpeed(int id, uint16_t* dataBuf);
	int readSetSpeed(int id, uint16_t* dataBuf);
	int readActualTorqueReference(int id, uint16_t* dataBuf);
	int readActualPosition(int id, uint16_t* dataBuf);
	int checkAlarm(void);
private:

	int errCnt = 0;

};

