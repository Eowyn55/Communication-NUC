#define _CRT_SECURE_NO_DEPRECATE
#include <opencv2/opencv.hpp>
#include "opencv2/objdetect.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cstdio>
#include <cstdlib>
#include <windows.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include "header_main.h"

cv::VideoCapture cap;
cv::VideoCapture obj;

// line tracking
cv::Mat frame;
cv::Mat filtered;
cv::Mat no_background;

// object detection
cv::Mat frame_obj;

cv::Mat final_image;
cv::Mat sliced[4];

int height;
int width;
int sl;
const int NUM_SLICES = 4;

int contour_num[NUM_SLICES];

/* PID parameters*/

//slow movement 
int REF_SPEED_LEFT = -1279; // was -1779 // 1279
int REF_SPEED_RIGHT = 1000; // was 1500 // 1000
int SPEED_HARD_LIMIT = 2600;
int SPEED_HARD_LIMIT_LOW = 450;
float Kp = 0.5; // pid proportional component working with 0.4 , was even 1
float Kd = 0; // pid derivative component
float Ki = 0; // pid integral component
int slice_errors[4];
int error_cur = 0;
int error_prev = 0;
int current_speed_left = 0;
int current_speed_right = 0;
int num_array = 0;  // how many elements there is
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

//frame sizes
cv::Size size_line(
(int)cap.get(CV_CAP_PROP_FRAME_WIDTH),
(int)cap.get(CV_CAP_PROP_FRAME_HEIGHT) / 2
);

//cv::Size size_obj(
//(int)obj.get(CV_CAP_PROP_FRAME_WIDTH),
//(int)obj.get(CV_CAP_PROP_FRAME_HEIGHT)
//);

//writer to a avi file
//cv::VideoWriter writer_line("line.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, size_line, true);
//cv::VideoWriter writer_obj("object.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, size_obj, true);


//Haar cascade Stop sign
std::string cascade_stop_file_name = "cascade_0999_25.xml";
cv::Ptr<cv::CascadeClassifier> cascade_stop(new cv::CascadeClassifier(cascade_stop_file_name));
int stop_cnt = 0;

//Haar cascade Traficlight
std::string cascade_trafic_file_name = "cascade_traficlight.xml";
cv::Ptr<cv::CascadeClassifier> cascade_trafic(new cv::CascadeClassifier(cascade_trafic_file_name));
int trafic_cnt = 0;
int light = 0;
std::vector<cv::Rect> objects_trafic;
std::vector<cv::Rect> objects_trafic_save;
bool flag_red_detected = false;
bool flag_green_detected = false;
int cnt_red = 0;
int cnt_green = 0;

//Timer 
enum timStatus {NOT_SET, WAIT_ON_STOP, WAIT_FOR_PERMISION, SET_TRAFIC};
timStatus set_timer = NOT_SET;
bool flag_timer = false;
std::chrono::high_resolution_clock::time_point start_time;
std::chrono::high_resolution_clock::time_point end_time;
std::chrono::duration<float> duration;
float max_time_stop = 5;
float max_time_trafic = 2;
float max_time_wait = 15;


int main(int argc, char** argv) {

	cv::namedWindow("Line tracking", cv::WINDOW_AUTOSIZE);
	cv::namedWindow("Object detection", cv::WINDOW_AUTOSIZE);

	cap.open(0); // open the camera, 0 for NUC, 1 for laptop - line tracking
	obj.open(1); // open the camera, 1 for NUC, 2 for laptop - object detection

	if (!cap.isOpened()) { // check if we succeeded
		std::cerr << "Couldn't open capture for line tracking." << std::endl;
		return -1;
	}

	if (!obj.isOpened()) { // check if we succeeded
		std::cerr << "Couldn't open capture for object detection." << std::endl;
		return -1;
	}

	//open COM port for communication
	CommPort = ComPortInit("COM5"); // COM4 for laptop, COM3 for NUC
	if (CommPort == INVALID_HANDLE_VALUE) {
		printf("com port initialization failed");
		return -1;
	}

	for (;;) {

		//object detection
		obj >> frame_obj;
		stop_cnt = detectStop(frame_obj, cascade_stop);
		if (flag_green_detected == false) {
			light = detectTraficLight(frame_obj, cascade_trafic);
			if (light == 1) {
				cv::putText(frame_obj, "RED", cv::Point(frame.cols / 2, frame.rows / 2), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2, cv::LINE_4);
				cnt_red++;
				cnt_green = 0;
			}
			else if (light == 2) {
				cv::putText(frame_obj, "GREEN", cv::Point(frame.cols / 2, frame.rows / 2), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2, cv::LINE_4);
				cnt_green++;
				cnt_red = 0;
			}
			else {
				cnt_red = 0;
				cnt_green = 0;
			}
		}
		cv::imshow("Object detection", frame_obj);
		//writer_obj.write(frame_obj);

		// line tracking
		cap >> frame;
		height = frame.rows;
		width = frame.cols;
		frame = frame(cv::Rect(0, height/2, width, height/2));

		if (frame.empty()) break; // Ran out of film

		// Remove background
		cv::GaussianBlur(frame, filtered, cv::Size(7,7), 0);
		cv::inRange(filtered, cv::Scalar(0, 0, 0), cv::Scalar(100, 100, 100), no_background);

		//cv::imshow("Line tracking", no_background);

		// Slice image to 4 segments:
		height = frame.rows;
		width = frame.cols;
		sl = int(height / NUM_SLICES);

		for (int i = 0; i < NUM_SLICES; i++) {

			int x, y = 0; // position of the centre of the contour of this slice

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
			contour_num[i] = contours.size();

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

		// stop on STOP sign
		if ((stop_cnt >= 1) && (set_timer == NOT_SET)) {
			start_time = std::chrono::high_resolution_clock::now();
			set_timer = WAIT_ON_STOP;
			current_speed_right = 0;
			current_speed_left = 0;
		}

		// check timer
		if (set_timer == WAIT_ON_STOP) {
			end_time = std::chrono::high_resolution_clock::now();
			duration = end_time - start_time;
			current_speed_right = 0;
			current_speed_left = 0;
			if (duration.count() > max_time_stop) {
				set_timer = WAIT_FOR_PERMISION;
			}
		}
		else if (set_timer == WAIT_FOR_PERMISION) {
			end_time = std::chrono::high_resolution_clock::now();
			duration = end_time - start_time;
			if (duration.count() > max_time_wait) {
				set_timer = NOT_SET;
			}
		}

		//stop on trafic light
		if ((trafic_cnt >= 1) && (flag_green_detected != true)) {
			if (set_timer == NOT_SET) {
				current_speed_right = 0;
				current_speed_left = 0;
				start_time = std::chrono::high_resolution_clock::now();
				set_timer = SET_TRAFIC;
			} 
			else if (set_timer == SET_TRAFIC) {
				end_time = std::chrono::high_resolution_clock::now();
				duration = end_time - start_time;
				if (duration.count() > max_time_trafic) {
					if (cnt_red > 0) {
						flag_red_detected = true;
						current_speed_right = 0;
						current_speed_left = 0;
					}
					else if (cnt_green > 0) {
						flag_green_detected = true;
					}
					set_timer = NOT_SET;
				}
				else {
					current_speed_right = 0;
					current_speed_left = 0;
				}
			}
			else if (flag_red_detected) {
				if (cnt_green > 0) {
					flag_green_detected = true;
				}
			}
		}

		left_array[num_array] = current_speed_left;
		right_array[num_array] = current_speed_right;
		error_array[num_array] = error_cur;
		num_array++;

		cv::putText(final_image, ("Error is: " + (std::to_string(error_cur))), cv::Point(30, final_image.rows - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_4);
		cv::putText(final_image, ("Left speed is: " + (std::to_string(current_speed_left))), cv::Point(30, final_image.rows - 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_4);
		cv::putText(final_image, ("Right speed is: " + (std::to_string(current_speed_right))), cv::Point(30, final_image.rows - 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_4);
	
		cv::imshow("Line tracking", final_image);
		//writer_line.write(final_image);

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
	err = write_speeds(left_array, right_array, error_array, num_array);
	if (err) {
		printf("Failed to write speeds and errors in file");
		return -1;
	}

	// close COM port
	CloseHandle(CommPort);

	return 0;

}

int write_speeds(int* left_speeds, int* right_speeds, int* errors, int n)
{
	std::ofstream myfile("speeds1.txt");
	if (myfile.is_open()) {
		myfile << "Left motor.\n";
		for (int count = 0; count < n; count++) {
			myfile << left_speeds[count] << " ";
		}
		myfile << "\n";
		myfile << "Right motor.\n";
		for (int count = 0; count < n; count++) {
			myfile << right_speeds[count] << " ";
		}
		myfile << "\n";
		myfile << "Error.\n";
		for (int count = 0; count < n; count++) {
			myfile << errors[count] << " ";
		}

		myfile.close();
		return 0;
	}
	else {
		std::cout << "Unable to open file";
		return 1;
	}
	return 1;
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

/*
 * description : detect stop sign using haar cascade you've created
 * img: input image
 * classifier : preloaded classifier
 * scale : resize image by
 * return int: how many object has been found
 */
int detectStop(cv::Mat& img, cv::Ptr<cv::CascadeClassifier> classifier, double scale) {

	enum { BLUE, AQUA, CYAN, GREEN };           // Just some pretty colors to draw with
	static cv::Scalar colors[] = {
		cv::Scalar(0, 0, 255),
		cv::Scalar(0, 128, 255),
		cv::Scalar(0, 255, 255),
		cv::Scalar(0, 255, 0)
	};

	// Image preparation:
	cv::Mat gray(img.size(), CV_8UC1);
	cv::Mat small_img(cvSize(cvRound(img.cols / scale),
		cvRound(img.rows / scale)), CV_8UC1);
	cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
	cv::resize(gray, small_img, small_img.size(), 0.0, 0.0, cv::INTER_LINEAR);
	cv::equalizeHist(small_img, small_img);

	// Detect objects if any
	std::vector<cv::Rect> objects;
	classifier->detectMultiScale(
		small_img,                  // input image
		objects,                    // place for the results
		1.1,                        // scale factor
		3,                          // minimum number of neighbors
		CV_HAAR_DO_CANNY_PRUNING,   // (old format cascades only)
		cv::Size(100, 100));          // throw away detections smaller than this

	// Loop through to found objects and draw boxes around them
	int i = 0;
	for (std::vector<cv::Rect>::iterator r = objects.begin();
		r != objects.end(); r++, ++i) {
		cv::Rect r_ = (*r);
		r_.x *= scale;
		r_.y *= scale;
		r_.width *= scale;
		r_.height *= scale;
		cv::rectangle(img, r_, colors[i % 4], 3);
	}
	return i;
}

/*
* description : detect stop sign using haar cascade you've created
* img: input image
* classifier : preloaded classifier
* scale : resize image by
* return int: how many object has been found
*/
int detectTraficLight(cv::Mat& img, cv::Ptr<cv::CascadeClassifier> classifier, double scale) {

	enum { BLUE, AQUA, CYAN, GREEN };           // Just some pretty colors to draw with
	static cv::Scalar colors[] = {
		cv::Scalar(0, 255, 0),
		cv::Scalar(0, 255, 255),
		cv::Scalar(0, 128, 255),
		cv::Scalar(0, 0, 255)
	};

	// Image preparation:
	cv::Mat object_rect;
	cv::Mat object_rect_up;
	cv::Mat object_rect_down;
	cv::Mat gray(img.size(), CV_8UC1);
	cv::Mat small_img(cvSize(cvRound(img.cols / scale),
		cvRound(img.rows / scale)), CV_8UC1);
	cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
	cv::resize(gray, small_img, small_img.size(), 0.0, 0.0, cv::INTER_LINEAR);
	//cv::equalizeHist(small_img, small_img);

	// Detect objects if any
	//if (trafic_cnt == 0) {
		classifier->detectMultiScale(
			small_img,                  // input image
			objects_trafic,                    // place for the results
			1.1,                        // scale factor
			3,                          // minimum number of neighbors
			CV_HAAR_DO_CANNY_PRUNING,   // (old format cascades only)
			cv::Size(30, 30));          // throw away detections smaller than this
	//}

	if (!objects_trafic.empty()) {
		trafic_cnt++;
		if (set_timer != WAIT_FOR_PERMISION) {
			objects_trafic_save = objects_trafic;
		}
	}
	else if (set_timer != WAIT_FOR_PERMISION){
		objects_trafic = objects_trafic_save;
	}
									  // Loop through to found objects and draw boxes around them
	int i = 0;
	for (std::vector<cv::Rect>::iterator r = objects_trafic.begin();
		r != objects_trafic.end(); r++, ++i) {
		cv::Rect r_ = (*r);
		r_.x *= scale;
		r_.y *= scale;
		r_.width *= scale;
		r_.height *= scale;
		cv::rectangle(img, r_, colors[i % 4], 3);
		object_rect = cv::Mat(img, r_);
		cvtColor(object_rect, object_rect, cv::COLOR_BGR2GRAY);
		cv::GaussianBlur(object_rect, object_rect, cv::Size(25, 25), 0, 0);
		int height = object_rect.rows;
		int width = object_rect.cols;
		object_rect_up = object_rect(cv::Rect(0, 0, width, height / 2));
		object_rect_down = object_rect(cv::Rect(0, height / 2, width, height / 2));
		int sum_up = sum_of_gray(object_rect_up);
		int sum_down = sum_of_gray(object_rect_down);
		if (sum_up > sum_down) {
			return 1;
		}
		else if (sum_up < sum_down) {
			return 2;
		}
		else {
			return 0;
		}
	}
}

int sum_of_gray(cv::Mat input_image) {
	int height = input_image.rows;
	int width = input_image.cols;
	int sum = 0;
	for (int j = 0; j<height; j++)
	{
		for (int i = 0; i<width; i++)
		{
			int d = input_image.at<uchar>(j, i);
			sum += d;
		}
	}
	return sum;
}