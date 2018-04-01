#include <opencv2/opencv.hpp>
#include <iostream>

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
			
			for (int i = 0; i< contours.size(); i++) // iterate through each contour. 
			{
				double a = contourArea(contours[i], false);  //  Find the area of contour
				if (a>largest_area) {
		        	largest_area = a;
					largest_contour_index = i;                //Store the index of largest contour
				}

			}

			// center of the slice
			cv::circle(sliced[i], cv::Point(sliced[i].cols / 2, sliced[i].rows / 2), 7, cv::Scalar(0, 0, 255), -1);

			if (contours.size() > 0) {
				cv::drawContours(sliced[i], contours, largest_contour_index, cv::Scalar(0, 255, 0), 2);

				//center of the largest contour
				cv::Moments M;
				M = cv::moments(contours[largest_contour_index]);

				int x, y;

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
			
		}


		for (int i = 0; i < NUM_SLICES; i++) {
			//sliced[i].copyTo(final_image); //(cv::Rect(0, i*sliced[i].rows, sliced[i].cols, sliced[i].rows))
			frame(cv::Rect(0, i*sl, width, sl)) = sliced[i];
		}

		final_image = frame;
		cv::imshow("Final", final_image);

		if ((char)cv::waitKey(33) >= 0) break;

	}

	return 0;

}