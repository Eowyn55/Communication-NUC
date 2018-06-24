#pragma once

/* Function declaration*/

HANDLE ComPortInit(const char * comName); // init com port
int sendData(HANDLE comPort, const char * data, int len); // send data through com port
int write_speeds(int* left_speeds, int* right_speeds, int* errors, int n);// write speeds and errors in file
int detectStop(cv::Mat& img, cv::Ptr<cv::CascadeClassifier> classifier, double scale = 1.3); // detect stop sign using haar cascade you've created
int detectTraficLight(cv::Mat& img, cv::Ptr<cv::CascadeClassifier> classifier, double scale = 1.3); // detect stop sign using haar cascade you've created
int sum_of_gray(cv::Mat input_image); // sum all the pixels from an gray image