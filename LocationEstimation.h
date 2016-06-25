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
#include <vector>
#include <math.h>

class LocationEstimation {
public:
	static const int MAX_WIDTH;
	static const int MAX_HEIGHT;
	static const int CURSOR_SIZE;
	static const int RAY_DISTANCE;
	static const int RAY_WIDTH;
	static const bool ENABLE_NOISE;
	static const bool ENABLE_FAILURE;
	static const double RAY_FAILURE_RATE;
	static const double RAY_MAX_NOISE;
	static cv::Mat map;
	static cv::Point pos;
	static void start(const char* imagePath, double scale);
	static double rayCast(int x, int y, double angle, double maxDist);
	static double rayCast(int x, int y, double angle, double maxDist, cv::Mat* display, double scale);
	static void moveUp();
	static void moveDown();
	static void moveLeft();
	static void moveRight();
	static bool canMove(int x, int y);
	static bool isBarrier(int x, int y);
	static cv::Mat resizeDisplay(cv::Mat img, double scale);
};

#endif /* LOCATIONESTIMATION_H_ */
