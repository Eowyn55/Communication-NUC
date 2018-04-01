#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>

void help(char** argv) {
	std::cout << "\n"
		<< "\The same object can load videos from a camera or a file"
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
		cap.open(1); // open the first camera
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
	int REF_SPEED_LEFT = 1000;
	int REF_SPEED_RIGHT = -1279;
	int SPEED_HARD_LIMIT = 2600;
	float Kp = 1; // pid proportional component
	float Kd = 0; // pid derivative component
	float Ki = 0; // pid integral component
	int slice_errors[4];
	int error_cur = 0;
	int error_prev = 0;
	int current_speed_left = 0;
	int current_speed_right = 0;
	float proportional = 0;
	float derivative = 0;

	for (;;) {

		cap >> frame;

		if (frame.empty()) break; // Ran out of film

		// Remove background

		// METHOD1
		//// convert to HSV color space
		//cv::cvtColor(frame, hsvImage, CV_BGR2HSV);
		//// for black line, 
		//cv::inRange(hsvImage, cv::Scalar(0, 0, 0, 0), cv::Scalar(180, 255, 40, 0), no_background);

		// METHOD2
		cv::GaussianBlur(frame, filtered, cv::Size(7,7), 0);
		cv::inRange(filtered, cv::Scalar(0, 0, 0), cv::Scalar(100, 100, 100), no_background);
		//cv::bitwise_not(no_background, no_background, cv::noArray());

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
			//cv::findContours(tresh, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0, 0));  // cv::RETR_TREE
			//contours = std::max(contours, cv::contourArea);
			
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
			
			slice_errors[i] = x - (sliced[i].cols / 2);
			cv::putText(sliced[i], std::to_string(slice_errors[i]), cv::Point(sliced[i].cols / 2 + 20, sliced[i].rows / 2), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 0), cv::LINE_4);
			
		}


		for (int j = 0; j < NUM_SLICES; j++) {
			//sliced[i].copyTo(final_image); //(cv::Rect(0, i*sliced[i].rows, sliced[i].cols, sliced[i].rows))
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

		if (current_speed_left > SPEED_HARD_LIMIT) current_speed_left = SPEED_HARD_LIMIT;
		if (current_speed_right > SPEED_HARD_LIMIT) current_speed_right = SPEED_HARD_LIMIT;

		cv::putText(final_image, ("Error is: " + (std::to_string(error_cur))), cv::Point(30, final_image.rows - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_4);
		cv::putText(final_image, ("Left speed is: " + (std::to_string(current_speed_left))), cv::Point(30, final_image.rows - 30), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_4);
		cv::putText(final_image, ("Right speed is: " + (std::to_string(current_speed_right))), cv::Point(30, final_image.rows - 60), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 255), 1, cv::LINE_4);
	
		cv::imshow("Final", final_image);

		//BUG : kada su neke sa jedne a neke sa druge strane, sve greske budu istog znaka!

}

	return 0;

}