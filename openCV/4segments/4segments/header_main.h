#pragma once

/* Function declaration*/

HANDLE ComPortInit(const char * comName); // init com port
int sendData(HANDLE comPort, const char * data, int len); // send data through com port