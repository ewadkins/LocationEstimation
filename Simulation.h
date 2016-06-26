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

#include "Ray.h"

class Simulation {
public:
	Simulation(const char* imagePath, bool allowLocationAccess, bool allowMapAccess);
	virtual ~Simulation() {}
	void start();
	double rayCast(int x, int y, double angle, double maxDist);
	void registerRay(double angle, double maxDist);
	cv::Point getPosition();
	cv::Point getCenterPosition();
	cv::Mat getMap();
	int getMapWidth();
	int getMapHeight();
	void moveUp();
	void moveDown();
	void moveLeft();
	void moveRight();
	void rotate(double angle);
	bool canMove(int x, int y);
	bool isBarrier(int x, int y);
	virtual void onStart() = 0;
	virtual void onData(std::vector<std::pair<Ray, double> > rayMap) = 0;
	virtual void keyListener(int key) = 0;
	void kill();
	void kill(const char* msg);
private:
	static int simulationCount;

	int simulationId;
	const bool locationAccess;
	const bool mapAccess;

	const char* imagePath;

	int maxWindowWidth;
	int maxWindowHeight;

	int desiredMapSize;

	int cursorSize;

	std::vector<Ray> rays;

	int rayWidth;
	int targetPointSize;

	bool enableRayNoise;
	bool enableRayFailure;
	double rayFailureRate;
	double rayMaxNoise;

	cv::Mat map;
	cv::Point pos;

	bool _canMove(int x, int y);
	bool _isBarrier(int x, int y);
};

#endif /* SIMULATION_H_ */
