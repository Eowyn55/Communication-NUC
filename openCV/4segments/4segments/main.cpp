#include <opencv2/opencv.hpp>
#include <cstdio>
#include <cstdlib>
#include <windows.h>
#include <iostream>
#include <fstream>
#include <string>
#include "header_main.h"

void help(char** argv) {
	std::cout << "\n"
		<< "The same object can load videos from a camera or a file"
		<< "\nCall:\n"
		<< argv[0] << " [path/image]\n"
		<< "\nor, read from camera:\n"
		<< argv[0]
		<< "\nFor example:\n"
		<< argv[0] << " ../tree.avi\n"
		<< std::endl;
}


int main(int argc, char** argv) {

	help(argv);
	cv::namedWindow("Final", cv::WINDOW_AUTOSIZE);

	cv::VideoCapture cap;

	if (argc == 1) {
		cap.open(0); // open the camera, 0 for NUC, 1 for laptop
	}
	else {
		cap.open(argv[1]);
	}

	if (!cap.isOpened()) { // check if we succeeded
		std::cerr << "Couldn't open capture." << std::endl;
		return -1;
	}

	cv::Mat frame;
	cv::Mat filtered;
	cv::Mat no_background;

	//cv::Mat final_image(frame.rows, frame.cols, CV_8UC3);  //cv_8u3c
	cv::Mat final_image;

	cv::Mat sliced[4];

	int height;
	int width;
	int sl;
	int NUM_SLICES = 4;

	/* PID parameters*/

	//slow movement 
	int REF_SPEED_LEFT = -1779; // was -1279
	int REF_SPEED_RIGHT = 1500; // was 1000
	int SPEED_HARD_LIMIT = 2600;
	int SPEED_HARD_LIMIT_LOW = 450;
	float Kp = 1; // pid proportional component working with 0.4
	float Kd = 0; // pid derivative component
	float Ki = 0; // pid integral component
	int slice_errors[4];
	int error_cur = 0;
	int error_prev = 0;
	int current_speed_left = 0;
	int current_speed_right = 0;
	int num_array = 0;
	int left_array[15000];
	int right_array[15000];
	int error_array[15000];
	float proportional = 0;
	float derivative = 0;

	//sending speeds
    #define COMMAND_SIZE          (5)
	// RL RH LL LH wait in 100ms
	// to go forward, L should bi negative
	char COMMAND_CAPTURE[COMMAND_SIZE] = { -0xff, -0x04, 0xe8, 0x03, 0x07 };
	int err;
	HANDLE CommPort = NULL;

	//open COM port for communication
	CommPort = ComPortInit("COM3"); // COM4 for laptop, COM3 for NUC
	if (CommPort == INVALID_HANDLE_VALUE) {
		printf("com port initialization failed");
		return -1;
	}

	for (;;) {

		cap >> frame;
		height = frame.rows;
		width = frame.cols;
		frame = frame(cv::Rect(0, height/2, width, height/2));

		if (frame.empty()) break; // Ran out of film

		// Remove background

		// METHOD2
		cv::GaussianBlur(frame, filtered, cv::Size(7,7), 0);
		cv::inRange(filtered, cv::Scalar(0, 0, 0), cv::Scalar(100, 100, 100), no_background);

		// Slice image to 4 segments:
		height = frame.rows;
		width = frame.cols;
		sl = int(height / NUM_SLICES);

		for (int i = 0; i < NUM_SLICES; i++) {

			int x, y; // position of the centre of the contour of this slice

			int largest_area = 0;
			int largest_contour_index = 0;

			int part = i*sl;
			sliced[i] = frame(cv::Rect(0,part, width,sl));
		
			//Conturing on every segment
			cv::Mat gray;
			cv::Mat tresh;
			std::vector< std::vector< cv::Point> > contours;
			std::vector<cv::Vec4i> hierarchy;
			cv::cvtColor(sliced[i], gray, cv::COLOR_BGR2GRAY);
			cv::threshold(gray, tresh, 100, 255, cv::THRESH_BINARY_INV); //Get Threshold
			
			cv::findContours(tresh, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
			
			for (int j = 0; j< contours.size(); j++) // iterate through each contour. 
			{
				double a = contourArea(contours[j], false);  //  Find the area of contour
				if (a>largest_area) {
		        	largest_area = a;
					largest_contour_index = j;                //Store the index of largest contour
				}

			}

			// center of the slice
			cv::circle(sliced[i], cv::Point(sliced[i].cols / 2, sliced[i].rows / 2), 7, cv::Scalar(0, 0, 255), -1);
			
			if (contours.size() > 0) {
				cv::drawContours(sliced[i], contours, largest_contour_index, cv::Scalar(0, 255, 0), 2);

				//center of the largest contour
				cv::Moments M;
				M = cv::moments(contours[largest_contour_index]);

				if (M.m00 == 0) {
					x = 7;
					y = 0;
				}
				else {

					x = int(M.m10 / M.m00);
					y = int(M.m01 / M.m00);

				}

				cv::circle(sliced[i], cv::Point(x, sliced[i].rows / 2), 7, cv::Scalar(255, 255, 255), -1);

			}
			
			slice_errors[i] = (sliced[i].cols / 2) - x;
			cv::putText(sliced[i], std::to_string(slice_errors[i]), cv::Point(sliced[i].cols / 2 + 20, sliced[i].rows / 2), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0), cv::LINE_4);
			
		}


		for (int j = 0; j < NUM_SLICES; j++) {
			frame(cv::Rect(0, j*sl, width, sl)) = sliced[j];
		}

		final_image = frame;

		if ((char)cv::waitKey(33) >= 0) break;

		/*************** PID ********************/
		error_cur = slice_errors[0] + slice_errors[1] + slice_errors[2] + slice_errors[3];
		proportional = error_cur * Kp;
		derivative = (error_cur - error_prev) * Kd;
		
		error_prev = error_cur;

		current_speed_left = static_cast<int>(REF_SPEED_LEFT + proportional + derivative);
		current_speed_right = static_cast<int>(REF_SPEED_RIGHT + proportional + derivative);

		if (current_speed_right > SPEED_HARD_LIMIT) current_speed_right = SPEED_HARD_LIMIT;
		if (current_speed_left < -SPEED_HARD_LIMIT) current_speed_left = -SPEED_HARD_LIMIT;

		if (current_speed_right < SPEED_HARD_LIMIT_LOW) current_speed_right = SPEED_HARD_LIMIT_LOW;
		if (current_speed_left > -SPEED_HARD_LIMIT_LOW) current_speed_left = -SPEED_HARD_LIMIT_LOW;

		left_array[num_array] = current_speed_left;
		right_array[num_array] = current_speed_right;
		error_array[num_array] = error_cur;
		num_array++;

		cv::putText(final_image, ("Error is: " + (std::to_string(error_cur))), cv::Point(30, final_image.rows - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_4);
		cv::putText(final_image, ("Left speed is: " + (std::to_string(current_speed_left))), cv::Point(30, final_image.rows - 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_4);
		cv::putText(final_image, ("Right speed is: " + (std::to_string(current_speed_right))), cv::Point(30, final_image.rows - 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_4);
	
		cv::imshow("Final", final_image);

		current_speed_left = -current_speed_left;

		// send speeds
		COMMAND_CAPTURE[0] = current_speed_right & 0x00FF;
		COMMAND_CAPTURE[1] = current_speed_right >> 8;
		COMMAND_CAPTURE[2] = current_speed_left & 0x00FF;
		COMMAND_CAPTURE[2] = -COMMAND_CAPTURE[2];
		COMMAND_CAPTURE[3] = current_speed_left >> 8;
		COMMAND_CAPTURE[3] = -COMMAND_CAPTURE[3];

		err = sendData(CommPort, COMMAND_CAPTURE, COMMAND_SIZE);
		if (err) {
			printf("failed to send command ping");
			return -1;
		}

		// if you press exit, break
		if ((char)cv::waitKey(33) >= 0) break;

    }


    // write speeds in a file

	std::ofstream myfile("speeds1.txt");
	if (myfile.is_open()) {
		myfile << "Left motor.\n";
		for (int count = 0; count < num_array; count++) {
			myfile << left_array[count] << " ";
		}
		myfile << "\n";
		myfile << "Right motor.\n";
		for (int count = 0; count < num_array; count++) {
			myfile << right_array[count] << " ";
		}
		myfile << "\n";
		myfile << "Error.\n";
		for (int count = 0; count < num_array; count++) {
			myfile << error_array[count] << " ";
		}

	myfile.close();
	}
	else {
		std::cout << "Unable to open file";
	}

	// close COM port
	CloseHandle(CommPort);

	return 0;

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