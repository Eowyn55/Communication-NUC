#pragma once

/* Function declaration*/

HANDLE ComPortInit(const char * comName); // init com port
int sendData(HANDLE comPort, const char * data, int len); // send data through com port
int write_speeds(int* left_speeds, int* right_speeds, int* errors, int n);// write speeds and errors in file