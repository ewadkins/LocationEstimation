/*
 * LocationEstimation.cpp
 *
 *  Created on: Jun 24, 2016
 *      Author: ericwadkins
 */

#include "LocationEstimation.h"

const int LocationEstimation::MAX_WIDTH = 600;
const int LocationEstimation::MAX_HEIGHT = 400;

cv::Mat LocationEstimation::map;

cv::Point LocationEstimation::pos = cv::Point(0, 0);

cv::Rect getMaxRectSingleContour(const cv::Mat1b& src) {
    cv::Mat1f width(src.rows, src.cols, float(0));
    cv::Mat1f height(src.rows, src.cols, float(0));

    cv::Rect maxRect(0, 0, 0, 0);
    float maxArea = 0;

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

void LocationEstimation::start(const char* imagePath) {
	cv::namedWindow("display", CV_WINDOW_AUTOSIZE);

	std::cout << "Image path: " << imagePath << std::endl;

	// Load source image
	cv::Mat srcImg = cv::imread(imagePath, cv::IMREAD_UNCHANGED);
	std::cout << srcImg.cols << " x " << srcImg.rows << std::endl;

	// Create grayscale and then black and white image
	cv::Mat grayImg;
	cv::cvtColor(srcImg, grayImg, cv::COLOR_BGR2GRAY);
	cv::Mat binaryImg = grayImg > 128;

	cv::cvtColor(binaryImg, map, cv::COLOR_GRAY2BGR);

	cv::Rect maxRect = getMaxRect(binaryImg);
	pos = cv::Point(maxRect.x + (maxRect.width - 1) / 2,
			maxRect.y + (maxRect.height - 1) / 2);

	while (true) {
		cv::Mat display = map.clone();
		//cv::rectangle(display, maxRect, cv::Scalar(200, 255, 200), -1);
		cv::circle(display, pos, std::max(display.cols, display.rows) / 200,
				cv::Scalar(0, 0, 255), -1);
		cv::Mat resized = resizeImage(display);

		// Display image
		cv::imshow("display", resized);

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
	return map.at<cv::Vec3b>(cv::Point(x, y))[0] > 0;
}

// Resize to fit the MAX_WIDTH and MAX_HEIGHT, but maintain aspect ratio
cv::Mat LocationEstimation::resizeImage(cv::Mat img) {
	cv::Mat resizedImg;
	double scale = std::max((float) MAX_WIDTH / img.cols,
			(float) MAX_HEIGHT / img.rows);
	cv::resize(img, resizedImg, cv::Size(img.cols * scale, img.rows * scale),
			0, 0, cv::INTER_NEAREST);
	return resizedImg;
}

int main(int argc, const char *argv[]) {

	// Check arguments
	if (argc < 2) {
		std::cerr << "ERROR No input image provided" << std::endl;
		std::cout << "Usage: ./main <image path>" << std::endl;
		exit(1);
	}

	// Get image path
	const char* imagePath = argv[1];
	LocationEstimation::start(imagePath);
}
