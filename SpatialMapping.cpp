/*
 * SpatialMapping.cpp
 *
 *  Created on: Jun 25, 2016
 *      Author: ericwadkins
 */

#include "SpatialMapping.h"

SpatialMapping::SpatialMapping(const char* imagePath) : Simulation(imagePath) {

}

void Simulation::keyListener(int key) {
	switch(key) {
	case 63232: moveForward(); break; // up
	case 63233: moveBackward(); break; // down
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
