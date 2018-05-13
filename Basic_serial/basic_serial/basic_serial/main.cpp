#include <cstdio>
#include <cstdlib>
#include <windows.h>
#include "header_main.h"
#include <iostream>

// timer flag
bool flag_timer = false;
int first_motor;
int second_motor;
int cnt = 0;
MSG Msg;

int main() 
{

    #define COMMAND_SIZE          (5)

	// RL RH LL LH wait in 100ms
	// to go forward, L should bi negative
	char COMMAND_CAPTURE[COMMAND_SIZE] = {-0xff, -0x04, 0xe8, 0x03, 0x07};

	int err;
	HANDLE CommPort = NULL;

	//timer
	//UINT TimerId = SetTimer(NULL, 0, 1000, NULL); //1000 milliseconds
	//if (!TimerId) 
	//	return 16;

	/* Initialize serial communication. */
	CommPort = ComPortInit("COM3"); // COM3 for NUC, COM4 for my laptop
	if (CommPort == INVALID_HANDLE_VALUE) {
		printf("com port initialization failed");
		return -1;
	}


	while (1) {
		
		// First motor is right motor, second is left motor
		// Both speeds should be positive, for going farward,
		// the second speed is turned negative automaticaly
		std::cout << "Insert the speed of first motor:" << std::endl;
		std::cin >> first_motor;
		std::cout << "Insert the speed of second motor:" << std::endl;
		std::cin >> second_motor;

		COMMAND_CAPTURE[0] = first_motor & 0x00FF;
		COMMAND_CAPTURE[1] = first_motor >> 8;
		COMMAND_CAPTURE[2] = second_motor & 0x00FF;
		COMMAND_CAPTURE[2] = -COMMAND_CAPTURE[2];
		COMMAND_CAPTURE[3] = second_motor >> 8;
		COMMAND_CAPTURE[3] = -COMMAND_CAPTURE[3];

		//while (1)
		//{
		//	if (GetMessage(&Msg, NULL, 0, 0)) {
		//		if (Msg.message == WM_TIMER) {
		//			cnt = cnt + 1;
		//			if (cnt == 11) break;
		/* Send command. */

		err = sendData(CommPort, COMMAND_CAPTURE, COMMAND_SIZE);
		if (err) {
		    printf("failed to send command ping");
			return -1;
		} else {
			std::cout << "Command sent!" << std::endl;
		}

		//		}
		//		DispatchMessage(&Msg);
		//	}
		//}
	}

	CloseHandle(CommPort);
	//KillTimer(NULL, TimerId);

}

HANDLE ComPortInit(const char * comName)
{
	HANDLE comHandle;

	comHandle = ::CreateFile(
		comName,
		GENERIC_READ | GENERIC_WRITE,
		0,
		NULL,
		OPEN_EXISTING,
		0,
		NULL);

	if (comHandle == INVALID_HANDLE_VALUE) {
		printf("failed to open com port with error %d", GetLastError());
		return NULL;
	}

	DCB dcb = { 0 };
	dcb.DCBlength = sizeof(DCB);

	if (!::GetCommState(comHandle, &dcb)) {
		printf("fail to get comm state with error %d", GetLastError());
		return NULL;
	}

	dcb.BaudRate = CBR_115200;
	dcb.Parity = 0;
	dcb.StopBits = 1;
	dcb.ByteSize = 8;
	dcb.fDtrControl = 0x00;
	dcb.fRtsControl = 0x00;

	if (!::SetCommState(comHandle, &dcb)) {
		printf("fail to set comm state with error %d", GetLastError());
		return NULL;
	}

	return comHandle;
}

int sendData(HANDLE comPort, const char * data, int len) {
	DWORD dwBytesWritten;
	BOOL status;

	status = WriteFile(comPort, data, len, &dwBytesWritten, NULL);
	if (len != dwBytesWritten) {
		printf("tried to send %d bytes, only %d was sent", len, dwBytesWritten);
		printf("error is %d", GetLastError());
		return NULL;
	}

	return 0;
}

