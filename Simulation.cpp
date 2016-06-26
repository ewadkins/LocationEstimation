/*
 * LocationEstimation.cpp
 *
 *  Created on: Jun 24, 2016
 *      Author: ericwadkins
 */

#include "Simulation.h"

int Simulation::simulationCount = 0;

Simulation::Simulation(const char* imagePath) {
	simulationCount++;
	this->simulationId = simulationCount;
	this->imagePath = imagePath;

	maxWindowWidth = 600;
	maxWindowHeight = 400;

	desiredMapSize = 400 * 400;

	cursorSize = 9;

	numRays = 8;
	maxRayDistance = 150;
	rayWidth = 2;
	rayRotationSpeed = 0.0;

	enableRayNoise = false;
	enableRayFailure = false;
	rayFailureRate = 0.01;
	rayMaxNoise = maxRayDistance / 40;
}

cv::Rect getMaxRectSingleContour(const cv::Mat1b& src) {
    cv::Mat1f width(src.rows, src.cols, float(0));
    cv::Mat1f height(src.rows, src.cols, float(0));

    cv::Rect maxRect(0, 0, 0, 0);
    double maxArea = 0;

    for (int r = 0; r < src.rows; r++) {
        for (int c = 0; c < src.cols; c++) {
            if (src(r, c) == 0) {
            	height(r, c) = 1.0 + ((r>0) ? height(r - 1, c) : 0);
                width(r, c) = 1.0 + ((c>0) ? width(r, c - 1) : 0);
            }
            float minw = width(r, c);
            for (int h = 0; h < height(r, c); h++) {
                minw = std::min(minw, width(r - h, c));
                float area = (h + 1) * minw;
                if (area > maxArea) {
                    maxArea = area;
                    maxRect = cv::Rect(cv::Point(c - minw + 1, r - h),
                    		cv::Point(c + 1, r + 1));
                }
            }
        }
    }

    return maxRect;
}

cv::Rect getMaxRect(cv::Mat binaryImg) {
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(binaryImg, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	cv::Rect maxRect(0, 0, 0, 0);
	int maxArea = 0;
	for (int i = 0; i < contours.size(); ++i) {
		std::cout << i << std::endl;
		cv::Mat maskSingleContour(binaryImg.rows, binaryImg.cols, uchar(0));
		cv::drawContours(maskSingleContour, contours, i, cv::Scalar(255),
				CV_FILLED);

		cv::Rect rect = getMaxRectSingleContour(~maskSingleContour);
		int area = rect.width * rect.height;
		if (area > maxArea) {
			maxRect = rect;
			maxArea = area;
		}
	}
	return maxRect;
}

void Simulation::start() {
	cv::namedWindow("Simulation " + std::to_string(simulationId), CV_WINDOW_AUTOSIZE);
	cv::namedWindow("observed map", CV_WINDOW_AUTOSIZE);

	std::cout << "Image path: " << imagePath << std::endl;

	// Load source image, and scale to desired map size
	cv::Mat srcImg = cv::imread(imagePath, cv::IMREAD_UNCHANGED);
	double scale = std::sqrt(desiredMapSize / (srcImg.cols * srcImg.rows));
	cv::resize(srcImg, srcImg, cv::Size(srcImg.cols * scale, srcImg.rows * scale), 0, 0, cv::INTER_NEAREST);
	std::cout << srcImg.cols << " x " << srcImg.rows << std::endl;

	// Create grayscale and then black and white image
	cv::Mat grayImg;
	cv::cvtColor(srcImg, grayImg, cv::COLOR_BGR2GRAY);
	cv::Mat binaryImg = grayImg > 128;

	// Save copy of the map
	cv::cvtColor(binaryImg, map, cv::COLOR_GRAY2BGR);

	// Determine starting position (largest available rectangle)
	cv::Rect maxRect = getMaxRect(binaryImg);
	//cv::rectangle(map, maxRect, cv::Scalar(220, 255, 220), -1); // Draws starting area
	pos = cv::Point(maxRect.x + (maxRect.width - 1) / 2,
			maxRect.y + (maxRect.height - 1) / 2);


    cv::Mat1f observedMap(map.rows, map.cols, float(0));


	long count = 0;
	while (true) {

		cv::Mat display = map.clone();

		// Create the display image
		double scale = std::max((double) maxWindowWidth / display.cols,
				(double) maxWindowHeight / display.rows);
		cv::resize(display, display,
				cv::Size(display.cols * scale, display.rows * scale),
				0, 0, cv::INTER_NEAREST);

		// Draws rays and get data
		double offset = -M_PI / 180 * count * rayRotationSpeed;
		for (int i = 0; i < numRays; i++) {
			double angle = i * M_PI / numRays * 2+ offset;
			double dist = rayCast(pos.x, pos.y, angle, maxRayDistance,
					&display, scale);
			double targetX = pos.x + 0.5 + std::cos(angle) * dist;
			double targetY = pos.y + 0.5 + std::sin(angle) * dist;

			if (dist > -1) {
				observedMap(targetY, targetX) =
						std::min(1.0, observedMap(targetY, targetX) + 0.25);
			}

			std::cout << dist << "\t";
		}
		std::cout << std::endl;

		// Draw cursor
		/*cv::circle(display, cv::Point((pos.x + 0.5) * scale, (pos.y + 0.5) * scale),
				cursorSize, cv::Scalar(255, 0, 0), -1);*/
		cv::rectangle(display, cv::Rect((pos.x + 0.5) * scale - cursorSize,
				(pos.y + 0.5) * scale - cursorSize, cursorSize * 2,
				cursorSize * 2), cv::Scalar(255, 0, 0), -1);
		if (cursorSize >= 5) {
			cv::circle(display, cv::Point((pos.x + 0.5) * scale, (pos.y + 0.5) * scale),
								2, cv::Scalar(0, 0, 255), -1);
		}

		// Display image
		cv::imshow("Simulation " + std::to_string(simulationId), display);

		cv::imshow("observed map", observedMap);

		// Handle keyboard input
		int key = cv::waitKey(1);
		if (key > -1) {
			this->keyListener(key);
		}

		count++;
	}
}

double Simulation::rayCast(int x, int y, double angle, double maxDist) {
	return rayCast(x, y, angle, maxDist, nullptr, 1);
}

double Simulation::rayCast(int x, int y, double angle, double maxDist,
		cv::Mat* display, double scale) {

	double stepSize;
	if (scale >= 1) { // Image had to be enlarged (each position occupies more than 1 pixel)
		stepSize = 1.0 / scale * std::fmin(scale, rayWidth);
	}
	else { // Image had to be reduced in size, so every pixel is fine
		stepSize = 1.0;
	}
	stepSize /= 2; // Step size is at least half the size of a pixel

	std::vector<std::pair<double, double> > rayList;
	bool failed = false;
	for (int i = 0; i * stepSize <= maxDist; i++) {
		double dist = i * stepSize;
		double targetXf = x + 0.5 + std::cos(angle) * dist;
		double targetYf = y + 0.5 + std::sin(angle) * dist;
		int targetX = targetXf;
		int targetY = targetYf;
		if (display != nullptr) {
			int scaledX = targetXf * scale;
			int scaledY = targetYf * scale;
			rayList.push_back(std::pair<double, double>(scaledX, scaledY));
		}
		if (!failed && isBarrier(targetX, targetY)) {
			if (enableRayFailure && rand() / (RAND_MAX + 1.0) < rayFailureRate) {
				failed = true;
				continue;
			}
			for (int j = 0; j < rayList.size(); j++) {
				cv::rectangle(*display, cv::Rect(rayList[j].first, rayList[j].second, rayWidth, rayWidth), cv::Scalar(100, 100, 255), -1);
			}
			double noisyDistance = dist + (enableRayNoise ? 1 : 0)
					* rand() / (RAND_MAX + 1.0) * rayMaxNoise / scale;
			double hitX = (x + 0.5 + std::cos(angle) * noisyDistance) * scale;
			double hitY = (y + 0.5 + std::sin(angle) * noisyDistance) * scale;
			cv::circle(*display, cv::Point(hitX, hitY), rayWidth * 2, cv::Scalar(0, 0, 255), -1);
			return noisyDistance;
		}
	}
	int n = rayList.size();
	if (enableRayFailure && rand() / (RAND_MAX + 1.0) < rayFailureRate) {
		n = rand() / (RAND_MAX + 1.0) * rayList.size();
	}
	for (int j = 0; j < n; j++) {
		cv::rectangle(*display, cv::Rect(rayList[j].first, rayList[j].second, rayWidth, rayWidth), cv::Scalar(100, 100, 255), -1);
	}
	if (n == rayList.size()) {
		return -1;
	}
	else {
		cv::circle(*display, cv::Point(rayList[n].first, rayList[n].second), rayWidth * 2, cv::Scalar(0, 0, 255), -1);
		return n * stepSize;
	}
}

void Simulation::moveForward() {
	if (canMove(pos.x, pos.y - 1)) {
		pos = cv::Point(pos.x, pos.y - 1);
	}
}

void Simulation::moveBackward() {
	if (canMove(pos.x, pos.y + 1)) {
		pos = cv::Point(pos.x, pos.y + 1);
	}
}

void Simulation::moveLeft() {
	if (canMove(pos.x - 1, pos.y)) {
		pos = cv::Point(pos.x - 1, pos.y);
	}
}

void Simulation::moveRight() {
	if (canMove(pos.x + 1, pos.y)) {
		pos = cv::Point(pos.x + 1, pos.y);
	}
}

bool Simulation::canMove(int x, int y) {
	if (x < 0 || x >= map.cols || y < 0 || y >= map.rows) {
		return false;
	}
	return map.at<cv::Vec3b>(cv::Point(x, y))[0] > 0;
}

bool Simulation::isBarrier(int x, int y) {
	if (x < 0 || x >= map.cols || y < 0 || y >= map.rows) {
		return false;
	}
	return map.at<cv::Vec3b>(cv::Point(x, y))[0] == 0;
}

/*void Simulation::setKeyListener(std::function<void(int key)> keyListener) {
	this->keyListener = keyListener;
}*/
