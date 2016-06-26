/*
 * SpatialMapping.cpp
 *
 *  Created on: Jun 25, 2016
 *      Author: ericwadkins
 */

#include "SpatialMapping.h"

const bool SpatialMapping::allowLocationAccess = true;
const bool SpatialMapping::allowMapAccess = false;

SpatialMapping::SpatialMapping(const char* imagePath)
			: Simulation(imagePath, allowLocationAccess, allowMapAccess) {
}

void SpatialMapping::onStart() {
	observedMap = cv::Mat1f(getMapHeight(), getMapWidth(), float(0));
	cv::namedWindow("Observed Map", CV_WINDOW_AUTOSIZE);

	int numRays = 12;
	double maxDistance = 100;

	for (int i = 0; i < numRays; i++) {
		registerRay(M_PI * i / numRays * 2, maxDistance);
	}
}

void SpatialMapping::onData(std::vector<std::pair<Ray, double> > rayMap) {
	cv::Point origin = getCenterPosition();
	for (int i = 0; i < rayMap.size(); i++) {
		Ray ray = rayMap[i].first;
		double dist = rayMap[i].second;
		double angle = ray.getAngle();
		if (dist > -1) {
			double targetX = origin.x + std::cos(angle) * dist;
			double targetY = origin.y + std::sin(angle) * dist;
			observedMap(targetY, targetX) =
					std::min(1.0, observedMap(targetY, targetX) + 1.0 / 3);
		}
	}

	double rotateSpeed = 1.0;
	rotate(M_PI / 180 * rotateSpeed);

	cv::Mat display;
	cv::cvtColor(observedMap, display, CV_GRAY2BGR);
	cv::circle(display, origin, 3, cv::Scalar(0, 255, 0), -1);


	cv::imshow("Observed Map", display);
}

void SpatialMapping::keyListener(int key) {
	switch(key) {
	case 63232: moveUp(); break; // up
	case 63233: moveDown(); break; // down
	case 63234: moveLeft(); break; // left
	case 63235: moveRight(); break; // right
	case 27: exit(1); break; // esc
	default: std::cout << "Key pressed: " << key << std::endl;
	}
}

int main(int argc, const char *argv[]) {

	// Check arguments
	if (argc < 2) {
		std::cerr << "ERROR No input image provided" << std::endl;
		std::cout << "Usage: ./SpatialMapping <image path>" << std::endl;
		exit(1);
	}

	// Get image path
	const char* imagePath = argv[1];
	Simulation* sim = new SpatialMapping(imagePath);
	sim->start();
}
