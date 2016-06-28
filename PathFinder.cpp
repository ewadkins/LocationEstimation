/*
 * SpatialMapping.cpp
 *
 *  Created on: Jun 25, 2016
 *      Author: ericwadkins
 */

#include "PathFinder.h"

const bool PathFinder::allowLocationAccess = true;
const bool PathFinder::allowMapAccess = false;
const bool PathFinder::allowImageAccess = true;

PathFinder::PathFinder(const char* imagePath)
			: Simulation(imagePath, allowLocationAccess, allowMapAccess, allowImageAccess) {
	start = cv::Point(-1, -1);
	goal = cv::Point(-1, -1);
	path = std::vector<cv::Point>();
}

void PathFinder::onStart() {
	observedMap = cv::Mat1f(getMapHeight(), getMapWidth(), float(1));
	cv::namedWindow("Observed Map", CV_WINDOW_AUTOSIZE);

	int numRays = 12;
	double maxDistance = 100;

	for (int i = 0; i < numRays; i++) {
		registerRay(M_PI * i / numRays * 2, maxDistance);
	}

	std::vector<std::pair<cv::Scalar, cv::Scalar> > ranges;
	ranges.push_back(std::pair<cv::Scalar, cv::Scalar>(cv::Scalar(100, 127, 127), cv::Scalar(130, 255, 255)));
	std::vector<std::pair<cv::Point, double> > blueBlobs = getBlobs(ranges);

	ranges.clear();
	ranges.push_back(std::pair<cv::Scalar, cv::Scalar>(cv::Scalar(45, 127, 127), cv::Scalar(75, 255, 255)));
	std::vector<std::pair<cv::Point, double> > greenBlobs = getBlobs(ranges);

	if (blueBlobs.size() > 0) {
		cv::Point startingPosition = blueBlobs[0].first;
		setStartingPosition(startingPosition.x, startingPosition.y);
	}
	if (greenBlobs.size() > 0) {
		goal = greenBlobs[0].first;
	}
	else {
		kill("Goal not found! Add a green blob to the image.");
	}

	start = getPosition();
}

void PathFinder::onData(std::vector<std::pair<Ray, double> > rayMap) {
	cv::Point origin = getCenterPosition();
	for (int i = 0; i < rayMap.size(); i++) {
		Ray ray = rayMap[i].first;
		double dist = rayMap[i].second;
		double angle = ray.getAngle();
		if (dist > -1) {
			int targetX = origin.x + std::cos(angle) * dist;
			int targetY = origin.y + std::sin(angle) * dist;
			if (targetX >= 0 && targetX < observedMap.cols
					&& targetY >= 0 && targetY < observedMap.rows) {
				observedMap(targetY, targetX) =
						std::max(0.0, observedMap(targetY, targetX) - 1.0 / 3);
			}
		}
	}

	double rotateSpeed = 1.0;
	rotate(M_PI / 180 * rotateSpeed);

	cv::Mat display;
	cv::cvtColor(observedMap, display, CV_GRAY2BGR);
	cv::circle(display, origin, 3, cv::Scalar(0, 255, 0), -1);


	cv::imshow("Observed Map", display);
}

void PathFinder::onDisplayBackground(cv::Mat display, double scale) {
	int size = 7;
	cv::circle(display, cv::Point(start.x * scale, start.y * scale), size, cv::Scalar(255, 127, 0), -1);
	cv::circle(display, cv::Point(goal.x * scale, goal.y * scale), size, cv::Scalar(0, 255, 0), -1);
	cv::circle(display, cv::Point(start.x * scale, start.y * scale), size, cv::Scalar(0, 0, 0), 2);
	cv::circle(display, cv::Point(goal.x * scale, goal.y * scale), size, cv::Scalar(0, 0, 0), 2);

	/*cv::rectangle(display, cv::Rect(start.x * scale - size, start.y * scale - size, size * 2, size * 2), cv::Scalar(255, 127, 0), -1);
	cv::rectangle(display, cv::Rect(goal.x * scale - size, goal.y * scale - size, size * 2, size * 2), cv::Scalar(0, 255, 0), -1);
	cv::rectangle(display, cv::Rect(start.x * scale - size, start.y * scale - size, size * 2, size * 2), cv::Scalar(0, 0, 0), 2);
	cv::rectangle(display, cv::Rect(goal.x * scale - size, goal.y * scale - size, size * 2, size * 2), cv::Scalar(0, 0, 0), 2);*/

	for (int i = 1; i < path.size(); i++) {
		int p1x = path[i-1].x;
		int p1y = path[i-1].y;
		int p2x = path[i].x;
		int p2y = path[i].y;
		cv::line(display, cv::Point(p1x * scale, p1y * scale),
				cv::Point(p2x * scale, p2y * scale), cv::Scalar(255, 0, 0), 2);
	}

}

void PathFinder::onDisplayForeground(cv::Mat display, double scale) {
}

void PathFinder::keyListener(int key) {
	if (key == 'g') {

		/*cv::namedWindow("test");
		cv::Mat temp;
		cv::cvtColor(observedMap, temp, cv::COLOR_GRAY2BGR);
		cv::Mat pathMap(observedMap.cols, observedMap.rows, CV_8UC3, cv::Scalar(0, 0, 0));
		temp.convertTo(pathMap, pathMap.type());
		//pathMap = pathMap > 128;

		//cv::Mat hsv;
		//cv::cvtColor(pathMap, hsv, cv::COLOR_BGR2HSV);
		//cv::Mat binaryImg;
		//cv::inRange(hsv, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 50), binaryImg);
		//binaryImg = ~binaryImg;

		// Save copy of the map
		//cv::cvtColor(binaryImg, pathMap, cv::COLOR_GRAY2BGR);

		cv::imshow("test", pathMap);
		cv::waitKey(0);
		exit(1);*/

		path = findPath(observedMap, getPosition(), goal);
	}
	switch(key) {
	case 63232: moveUp(); break; // up
	case 63233: moveDown(); break; // down
	case 63234: moveLeft(); break; // left
	case 63235: moveRight(); break; // right
	case 27: exit(1); break; // esc
	case 'g': break;
	default: std::cout << "Key pressed: " << key << std::endl;
	}
}

std::vector<cv::Point> PathFinder::findPath(cv::Mat1f map, cv::Point start, cv::Point goal) {
	if (map.channels() != 1)
		throw std::invalid_argument("Map must have only 1 channel.");

	std::pair<int, int> startPair = std::pair<int, int>(start.x, start.y);
	std::pair<int, int> goalPair = std::pair<int, int>(goal.x, goal.y);

	std::set<std::pair<int, int> > closedSet;
	std::set<std::pair<int, int> > openSet;
	openSet.insert(startPair);
	std::map<std::pair<int, int>, std::pair<int, int> > cameFrom;

	std::map<std::pair<int, int>, double> gScore;
	std::map<std::pair<int, int>, double> fScore;
	for (int i = 0; i < map.rows; i++) {
		for (int j = 0; j < map.cols; j++) {
			gScore[std::pair<int, int>(i, j)] = std::numeric_limits<double>::infinity();
			fScore[std::pair<int, int>(i, j)] = std::numeric_limits<double>::infinity();
		}
	}

	gScore[startPair] = 0;

	double startCost = std::sqrt(
			std::pow(goal.x - start.x, 2) + std::pow(goal.y - start.y, 2));
	fScore[startPair] = startCost;

	int dx[] = { -1, 0, 1, -1, 1, -1, 0, 1 };
	int dy[] = { -1, -1, -1, 0, 0, 1, 1, 1 };
	//int dx[] = { 0, 1, -1, 0 };
	//int dy[] = { 1, 0, 0, -1 };

	int n = sizeof(dx) / sizeof(dx[0]);
	int indices[n];
	for (int i = 0; i < n; i++) {
		indices[i] = i;
	}

	while (!openSet.empty()) {
		std::random_shuffle(&indices[0], &indices[n]);

		std::set<std::pair<int, int> >::iterator currentPtr;
		double minFScore = std::numeric_limits<double>::infinity();
		for (std::set<std::pair<int, int> >::iterator it = openSet.begin(); it != openSet.end(); it++) {
			if (it->second < minFScore) {
				currentPtr = it;
				minFScore = it->second;
			}
		}
		std::pair<int, int> current = *currentPtr;

		if (current == goalPair) {
			std::vector<cv::Point> path;
			while (cameFrom.find(current) != cameFrom.end()) {
				current = cameFrom[current];
				path.push_back(cv::Point(current.first, current.second));
			}
			return path;
		}

		openSet.erase(currentPtr);
		closedSet.insert(current);

		for (int k = 0; k < n; k++) {
			int nx = current.first + dx[indices[k]];
			int ny = current.second + dy[indices[k]];
			if (nx >= 0 && nx < map.rows && ny >= 0 && ny < map.cols
					&& (map(ny, nx) > 0)) {
				std::pair<int, int> neighbor(nx, ny);
				if (closedSet.find(neighbor) != closedSet.end()) {
					continue;
				}
				double dist = std::sqrt(std::pow(nx - current.first, 2) +
						std::pow(ny - current.second, 2));
				double tentative_gScore = gScore[current] + dist;
				if (openSet.find(neighbor) == openSet.end()) {
					openSet.insert(neighbor);
				}
				else if (tentative_gScore >= gScore[neighbor]) {
					continue;
				}

				cameFrom[neighbor] = current;
				gScore[neighbor] = tentative_gScore;
				double cost = std::sqrt(
						std::pow(goal.x - neighbor.first, 2) +
						std::pow(goal.y - neighbor.second, 2));
				fScore[neighbor] = gScore[neighbor] + cost;
			}
		}
	}
	return std::vector<cv::Point>();
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
	Simulation* sim = new PathFinder(imagePath);
	sim->start();
}
