/*
 * LocationEstimation.cpp
 *
 *  Created on: Jun 24, 2016
 *      Author: ericwadkins
 */

#include "LocationEstimation.h"

const int LocationEstimation::MAX_WIDTH = 600;
const int LocationEstimation::MAX_HEIGHT = 400;
const int LocationEstimation::CURSOR_SIZE = 8;
const int LocationEstimation::RAY_DISTANCE = 150;
const int LocationEstimation::RAY_WIDTH = 2;

const bool LocationEstimation::ENABLE_NOISE = true;
const bool LocationEstimation::ENABLE_FAILURE = true;

const double LocationEstimation::RAY_FAILURE_RATE = 0.01;
const double LocationEstimation::RAY_MAX_NOISE = RAY_DISTANCE / 40;

cv::Mat LocationEstimation::map;

cv::Point LocationEstimation::pos = cv::Point(0, 0);

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

void LocationEstimation::start(const char* imagePath, double scale) {
	cv::namedWindow("display", CV_WINDOW_AUTOSIZE);

	std::cout << "Image path: " << imagePath << std::endl;

	// Load source image
	cv::Mat srcImg = cv::imread(imagePath, cv::IMREAD_UNCHANGED);
	while (srcImg.cols * scale * srcImg.rows * scale > 800 * 800) {
		scale /= 2;
		std::cout << "Image too large, decreasing scale by a factor of 2" << std::endl;
	}
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

	while (true) {

		cv::Mat display = map.clone();

		// Create the display image
		double scale = std::max((double) MAX_WIDTH / display.cols,
				(double) MAX_HEIGHT / display.rows);
		display = resizeDisplay(display, scale);

		int numRays = 8;
		for (int i = 0; i < numRays; i++) {
			double dist = rayCast(pos.x, pos.y, i * M_PI / numRays * 2 ,
					RAY_DISTANCE / scale, &display, scale);
			std::cout << dist << "\t";
		}
		std::cout << std::endl;

		// Display image
		cv::imshow("display", display);

		// Handle keyboard input
		int key = cv::waitKey(1);
		if (key > -1) {
			switch(key) {
			case 63232: moveUp(); break; // up
			case 63233: moveDown(); break; // down
			case 63234: moveLeft(); break; // left
			case 63235: moveRight(); break; // right
			case 27: exit(1); break; // esc
			default: std::cout << "Key pressed: " << key << std::endl;
			}
		}
	}
}

double LocationEstimation::rayCast(int x, int y, double angle, double maxDist) {
	return rayCast(x, y, angle, maxDist, nullptr, 1);
}

double LocationEstimation::rayCast(int x, int y, double angle, double maxDist,
		cv::Mat* display, double scale) {
	double stepSize = std::min(1 / scale, 0.05);
	std::vector<std::pair<double, double> > rayList;
	bool failed = false;
	for (int i = 0; i * stepSize < maxDist; i++) {
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
			if (ENABLE_FAILURE && rand() / (RAND_MAX + 1.0) < RAY_FAILURE_RATE) {
				failed = true;
				continue;
			}
			for (int j = 0; j < rayList.size(); j++) {
				cv::rectangle(*display, cv::Rect(rayList[j].first, rayList[j].second, RAY_WIDTH, RAY_WIDTH), cv::Scalar(100, 100, 255), -1);
			}
			double noisyDistance = dist + (ENABLE_NOISE ? 1 : 0)
					* rand() / (RAND_MAX + 1.0) * RAY_MAX_NOISE / scale;
			double hitX = (x + 0.5 + std::cos(angle) * noisyDistance) * scale;
			double hitY = (y + 0.5 + std::sin(angle) * noisyDistance) * scale;
			cv::circle(*display, cv::Point(hitX, hitY), RAY_WIDTH * 2, cv::Scalar(0, 0, 255), -1);
			return noisyDistance;
		}
	}
	int n = rayList.size();
	if (ENABLE_FAILURE && rand() / (RAND_MAX + 1.0) < RAY_FAILURE_RATE) {
		n = rand() / (RAND_MAX + 1.0) * rayList.size();
	}
	for (int j = 0; j < n; j++) {
		cv::rectangle(*display, cv::Rect(rayList[j].first, rayList[j].second, RAY_WIDTH, RAY_WIDTH), cv::Scalar(100, 100, 255), -1);
	}
	if (n == rayList.size()) {
		return -1;
	}
	else {
		cv::circle(*display, cv::Point(rayList[n].first, rayList[n].second), RAY_WIDTH * 2, cv::Scalar(0, 0, 255), -1);
		return n * stepSize;
	}
}

void LocationEstimation::moveUp() {
	if (canMove(pos.x, pos.y - 1)) {
		pos = cv::Point(pos.x, pos.y - 1);
	}
}

void LocationEstimation::moveDown() {
	if (canMove(pos.x, pos.y + 1)) {
		pos = cv::Point(pos.x, pos.y + 1);
	}
}

void LocationEstimation::moveLeft() {
	if (canMove(pos.x - 1, pos.y)) {
		pos = cv::Point(pos.x - 1, pos.y);
	}
}

void LocationEstimation::moveRight() {
	if (canMove(pos.x + 1, pos.y)) {
		pos = cv::Point(pos.x + 1, pos.y);
	}
}

bool LocationEstimation::canMove(int x, int y) {
	if (x < 0 || x >= map.cols || y < 0 || y >= map.rows) {
		return false;
	}
	return map.at<cv::Vec3b>(cv::Point(x, y))[0] > 0;
}

bool LocationEstimation::isBarrier(int x, int y) {
	if (x < 0 || x >= map.cols || y < 0 || y >= map.rows) {
		return false;
	}
	return map.at<cv::Vec3b>(cv::Point(x, y))[0] == 0;
}

// Resize to fit the MAX_WIDTH and MAX_HEIGHT, but maintain aspect ratio
cv::Mat LocationEstimation::resizeDisplay(cv::Mat img, double scale) {
	cv::circle(img, pos, CURSOR_SIZE / scale, cv::Scalar(255, 0, 0), -1);
	cv::resize(img, img, cv::Size(img.cols * scale, img.rows * scale),
			0, 0, cv::INTER_NEAREST);
	return img;
}

int main(int argc, const char *argv[]) {

	// Check arguments
	double scale = 1;
	if (argc < 2) {
		std::cerr << "ERROR No input image provided" << std::endl;
		std::cout << "Usage: ./main <image path> [scale]" << std::endl;
		exit(1);
	}
	else if (argc >= 3) {
		scale = std::stod(argv[2]);
	}

	// Get image path
	const char* imagePath = argv[1];
	LocationEstimation::start(imagePath, scale);
}
