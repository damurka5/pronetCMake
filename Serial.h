#pragma once

#include <Windows.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <fstream>
#include <stdint.h>

class Serial
{
public:
	Serial(int comPort, int baudRate, int byteSize = 8, int stopBits = TWOSTOPBITS, int parity = NOPARITY);
	~Serial();

	int  connectRequest(void);
	bool closeSerialPort();
	bool readSerialPort(unsigned char& rxBuf, int requestSize, DWORD& factSize);
	bool checkStatus();
	bool writeSerialPort(unsigned char* txBuf);
	bool writeReadSerialPort(unsigned char* txBuf, unsigned char& rxBuf, int requestSize, DWORD& factSize, int timeDelay = 1);
	
	bool isConnected_;	// ���� �����������

private:
	HANDLE connectedPort_;
	COMSTAT status_;
	DWORD errors_;
	COMMTIMEOUTS SerialTimeouts;

	int comPort_;
	int baudRate_;
	int byteSize_;
	int stopBits_;
	int parity_;


	int openSerialPort();
};