/*
 * LocationEstimation.h
 *
 *  Created on: Jun 24, 2016
 *      Author: ericwadkins
 */

#ifndef SIMULATION_H_
#define SIMULATION_H_

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <functional>
#include <math.h>

class Simulation {
public:
	Simulation(const char* imagePath);
	virtual ~Simulation() {}
	void start();
	double rayCast(int x, int y, double angle, double maxDist);
	double rayCast(int x, int y, double angle, double maxDist, cv::Mat* display, double scale);
	void moveForward();
	void moveBackward();
	void moveLeft();
	void moveRight();
	void moveNorth();
	void moveSouth();
	void moveWest();
	void moveEast();
	bool canMove(int x, int y);
	bool isBarrier(int x, int y);
	//void setKeyListener(std::function<void, int> keyListener);
	virtual void keyListener(int key);
	static cv::Mat resizeDisplay(cv::Mat img, double scale);
private:
	static int simulationCount;

	int simulationId;

	const char* imagePath;

	int maxWindowWidth;
	int maxWindowHeight;

	int desiredMapSize;

	int cursorSize;

	int numRays;
	int maxRayDistance;
	int rayWidth;
	double rayRotationSpeed;

	bool enableRayNoise;
	bool enableRayFailure;
	double rayFailureRate;
	double rayMaxNoise;

	cv::Mat map;
	cv::Point pos;

	//void (*keyListener)(int);
	//std::function<void, int> keyListener;
};

#endif /* SIMULATION_H_ */
