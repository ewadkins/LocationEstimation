/*
 * LocationEstimation.cpp
 *
 *  Created on: Jun 24, 2016
 *      Author: ericwadkins
 */

#include "LocationEstimation.h"

const int LocationEstimation::MAX_WIDTH = 600;
const int LocationEstimation::MAX_HEIGHT = 400;

void LocationEstimation::start(const char* imagePath) {
	std::cout << "Image path: " << imagePath << std::endl;

	// Load source image
	cv::Mat srcImg = cv::imread(imagePath, cv::IMREAD_UNCHANGED);
	std::cout << srcImg.cols << " x " << srcImg.rows << std::endl;

	// Create grayscale and then black and white image
	cv::Mat grayImg;
	cv::cvtColor(srcImg, grayImg, cv::COLOR_BGR2GRAY);
	cv::Mat bwImg = grayImg > 128;

	// Display image
	cv::namedWindow("display", CV_WINDOW_AUTOSIZE);
	cv::imshow("display", resizeImage(bwImg));

	int key = cv::waitKey(0);
	std::cout << "Key pressed: " << key << std::endl;
	exit(0);
}

cv::Mat LocationEstimation::resizeImage(cv::Mat img) {
	cv::Mat resizedImg;
	double scale = std::max((float) MAX_WIDTH / img.cols,
			(float) MAX_HEIGHT / img.rows);
	cv::resize(img, resizedImg,
			cv::Size(img.cols * scale, img.rows * scale));
	return resizedImg;
}

int main(int argc, const char *argv[]) {

	// Check arguments
	if (argc < 2) {
		std::cerr << "ERROR No input image provided" << std::endl;
		std::cout << "Usage: ./main <image path>" << std::endl;
		exit(1);
	}

	// Get image path
	const char* imagePath = argv[1];
	LocationEstimation::start(imagePath);
}
