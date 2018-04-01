#include <cstdio>
#include <cstdlib>
#include <windows.h>
#include "header_main.h"
#include <iostream>

// timer flag
bool flag_timer = false;
int cnt = 0;
MSG Msg;

int main() 
{

    #define COMMAND_SIZE          (5)
	char COMMAND_CAPTURE[COMMAND_SIZE] = { -0xff, -0x04, 0xe8, 0x03, 0x07};

	int err;
	HANDLE CommPort = NULL;

	//timer
	UINT TimerId = SetTimer(NULL, 0, 1000, NULL); //1000 milliseconds
	if (!TimerId) 
		return 16;

	/* Initialize serial communication. */
	CommPort = ComPortInit("COM4");
	if (CommPort == INVALID_HANDLE_VALUE) {
		printf("com port initialization failed");
		return -1;
	}

	while(1) 
	{
		if (GetMessage(&Msg, NULL, 0, 0)) {
			if (Msg.message == WM_TIMER) {
				cnt = cnt + 1;
				if (cnt == 11) break;
				/* Send command. */
				err = sendData(CommPort, COMMAND_CAPTURE, COMMAND_SIZE);
				if (err) {
					printf("failed to send command ping");
					return -1;
				}

			}
			DispatchMessage(&Msg);
		}
	}

	

	CloseHandle(CommPort);
	KillTimer(NULL, TimerId);

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

