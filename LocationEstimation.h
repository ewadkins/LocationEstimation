/*
 * LocationEstimation.h
 *
 *  Created on: Jun 24, 2016
 *      Author: ericwadkins
 */

#ifndef LOCATIONESTIMATION_H_
#define LOCATIONESTIMATION_H_

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include <iostream>

class LocationEstimation {
public:
	static const int MAX_WIDTH;
	static const int MAX_HEIGHT;
	static void start(const char* imagePath);
	static cv::Mat resizeImage(cv::Mat img);
};

#endif /* LOCATIONESTIMATION_H_ */
