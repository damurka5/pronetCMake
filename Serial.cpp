#include "Serial.h"

Serial::Serial(int comPort, int baudRate, int byteSize, int stopBits, int parity)
{
	isConnected_ = false; // Сбрасываем флаг подключения
	comPort_ = comPort;
	baudRate_ = baudRate;
	byteSize_ = byteSize;
	stopBits_ = stopBits;
	parity_ = parity;
}

Serial::~Serial()
{
	if (isConnected_) {
		isConnected_ = false;
		CloseHandle(connectedPort_);
	}
}

int Serial::connectRequest(void)
{
	if (isConnected_) {
		CloseHandle(connectedPort_); // Закрываем соединение, если было открыто
		isConnected_ = false;
		return -5;
	}

	int error = 0;

	switch (openSerialPort()) // Пытаемся подключиться
	{
	case -5:
		std::cout << "Warning: Access to COM" << this->comPort_ << " is denied" << std::endl;
		error = -5;
		break;
	case -4:
		std::cout << "Warning: Handle was not attached. Reason: COM" << this->comPort_ << " is not available" << std::endl;
		error = -4;
		break;
	case -3:
		std::cout << "Warning: Failed to get current serial params" << std::endl;
		error = -3;
		break;
	case -2:
		std::cout << "Warning: could not set serial port params" << std::endl;
		error = -2;
		break;
	case -1:
		std::cout << "Warning: Set timeouts error" << std::endl;
		error = -1;
		break;
	case 0:
		std::cout << "Connected to COM" << comPort_ << std::endl;
		isConnected_ = true; // Ставим флаг успешного подключения
		return 0;
	default:
		error = 99;
		break;
	}

	CloseHandle(connectedPort_); // Закрываем соединение, если была ошибка и возвращаем код ошибки
	return error;
}

bool Serial::closeSerialPort()
{
	if (isConnected_) {
		isConnected_ = false;
		CloseHandle(connectedPort_);
		return true;
	}
	else
		return false;
}

bool Serial::readSerialPort(unsigned char& rxBuf, int requestSize, DWORD& factSize)
{
	if (!isConnected_)
		return false;

	DWORD iSize;
	bool status = ReadFile(connectedPort_, &rxBuf, requestSize, &iSize, 0);  // пытаемся получить requestSize байт
	factSize = iSize;

	return status;
}

bool Serial::checkStatus()
{
	DWORD bytes_sent;
	
	unsigned char txBuf[1] = { 0, };

	if (!WriteFile(connectedPort_, (void*)txBuf, 1, &bytes_sent, NULL)) {
		ClearCommError(connectedPort_, &errors_, &status_);
		isConnected_ = false;
		return false;
	}
	else
		return true;
}

bool Serial::writeSerialPort(unsigned char* txBuf)
{
	DWORD bytes_sent;

	unsigned int data_sent_length = sizeof(txBuf);

	if (!WriteFile(connectedPort_, (void*)txBuf, data_sent_length, &bytes_sent, NULL)) {
		ClearCommError(connectedPort_, &errors_, &status_);
		return false;
	}
	else
		return true;
}

bool Serial::writeReadSerialPort(unsigned char* txBuf, unsigned char& rxBuf, int requestSize, DWORD& factSize, int timeDelay)
{
	bool ans = true;
	
	ans = writeSerialPort(txBuf);
	Sleep(timeDelay);
	ans = readSerialPort(rxBuf, requestSize, factSize);

	return ans;
}

int Serial::openSerialPort()
{
	connectedPort_ = CreateFileA(
		("\\\\.\\COM" + std::to_string(this->comPort_)).c_str(),	// Открываем подключение к выбранному порту
		GENERIC_READ | GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		NULL);

	if (connectedPort_ == INVALID_HANDLE_VALUE) { // Если порт не найден, выводим ошибку
		if (GetLastError() == ERROR_ACCESS_DENIED) return -5;
		if (GetLastError() == ERROR_FILE_NOT_FOUND) return -4;
	}
	else {

		DCB dcbSerialParams = { 0 };

		if (!GetCommState(connectedPort_, &dcbSerialParams)) 
			return -3; // Запрашиваем начальные настройки, если не получили - выводим ошибку
		else {	// Настраиваем параметры подключения
			dcbSerialParams.BaudRate = this->baudRate_;
			dcbSerialParams.ByteSize = this->byteSize_;
			dcbSerialParams.StopBits = this->stopBits_;
			dcbSerialParams.Parity = this->parity_;
			dcbSerialParams.fDtrControl = DTR_CONTROL_ENABLE;

			if (!SetCommState(connectedPort_, &dcbSerialParams)) 
				return -2; // Если не удалось задать параметры
			else {
				// Настройка таймаутов
				SerialTimeouts.ReadIntervalTimeout = 1;
				SerialTimeouts.ReadTotalTimeoutConstant = 1;
				SerialTimeouts.ReadTotalTimeoutMultiplier = 1;
				SerialTimeouts.WriteTotalTimeoutConstant = 1;
				SerialTimeouts.WriteTotalTimeoutMultiplier = 1;
				if (!SetCommTimeouts(connectedPort_, &SerialTimeouts)) 
					return -1; // Если не удалось задать длительность ожидания
				PurgeComm(connectedPort_, PURGE_RXCLEAR | PURGE_TXCLEAR); // Cбрасывает все символы из буфера вывода или ввода данных
				return 0;
			}
		}
	}
}