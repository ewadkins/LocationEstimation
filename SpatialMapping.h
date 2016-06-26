/*
 * SpatialMapping.h
 *
 *  Created on: Jun 25, 2016
 *      Author: ericwadkins
 */

#ifndef SPATIALMAPPING_H_
#define SPATIALMAPPING_H_

#include "Simulation.h"
#include "Ray.h"

class SpatialMapping : public Simulation {
public:
	cv::Mat1f observedMap;

	SpatialMapping(const char* imagePath);
	~SpatialMapping() {}
	void onStart();
	void onData(std::vector<std::pair<Ray, double> > rayMap);
	void keyListener(int key);
private:
	static const bool allowLocationAccess;
	static const bool allowMapAccess;
};

#endif /* SPATIALMAPPING_H_ */
