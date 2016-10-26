/*
LINE DETECTION IN EMPTY + NON EMPTY -> 2 IMAGE VERSION
*/

#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <cmath>

const cv::Scalar GREEN = cv::Scalar(0, 255, 0);
const cv::Scalar RED = cv::Scalar(0, 0, 255);
const int CHUNK_HEIGHT = 40;
const int CHUNK_WIDTH = 25;
const int CUTOFF = 50;
const int DISTANCE = 5;
int JUMP = 3;
const int EROSION_SIZE = 1;
const int EROSION_TYPE = cv::MORPH_ELLIPSE;
const int DILATION_SIZE = 1;
const int DILATION_TYPE = cv::MORPH_ELLIPSE;
const int THRESHOLD = 125;

double distance(cv::Point2f p0, cv::Point2f p1)
{
	double dX0 = p0.x, dY0 = p0.y, dX1 = p1.x, dY1 = p1.y;
	return std::sqrt((dX1 - dX0)*(dX1 - dX0) + (dY1 - dY0)*(dY1 - dY0));
}

class PLApplication
{
public:
	cv::Mat image; //Loaded Image
	cv::Mat image_empty; // Empty parking lot
	cv::Mat roi; // Region of Interest for Homography
	cv::Mat roi_gray;
	cv::Mat roi_empty;
	cv::Mat roi_empty_gray;
	
	std::vector<cv::Point2f> corners;
	std::vector<cv::Point2f> spots;
	std::vector<cv::Point2f> parking_lines;
	std::vector<cv::Rect> parking_spots;

	void reset();
	void init(cv::Mat _image, cv::Mat _image_empty);
	void countCars();
	int getTotalCars();
private:
	int total_cars = 0;
	bool can_count_line;
	void detectLines();
	void filterLines();
	void createSpots();
	void filterSpots();
	float averageSpotWidth();
	float averageLineDistance();
	void analyzeSpot(cv::Rect rect);
};

PLApplication plapp;

void selectCorners(int event, int x, int y, int flags, void* param)
{
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		circle(plapp.image, cv::Point(x, y), 3, RED, 5, CV_AA);
		cv::imshow("PARKING LOT", plapp.image);
		if (plapp.corners.size() < 4)
		{
			plapp.corners.push_back(cv::Point2f(x, y));
		}
	}
}

void selectSpots(int event, int x, int y, int flags, void* param)
{
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		circle(plapp.roi, cv::Point(x, y), 3, RED, 5, CV_AA);
		cv::imshow("RESULT", plapp.roi);
		plapp.spots.push_back(cv::Point2f(x, y));
	}
}

void PLApplication::init(cv::Mat _image, cv::Mat _image_empty)
{
	_image.copyTo(image);
	_image_empty.copyTo(image_empty);
	can_count_line = true;
}

int PLApplication::getTotalCars()
{
	return total_cars;
}

float PLApplication::averageSpotWidth()
{
	float sum = 0;
	for (int i = 0; i < parking_spots.size(); i++)
	{
		sum += parking_spots[i].width;
	}
	std::cout << "SUM: " << sum << std::endl;
	std::cout << "SPOTS " << parking_spots.size() << std::endl;
	return sum / parking_spots.size();
}

float PLApplication::averageLineDistance()
{
	float sum = 0;
	int n_of_spots = 0;

	for (int i = 0; i < spots.size(); i++)
	{
		for (int k = 0; k < parking_lines.size() - 1; k++)
		{
			if (parking_lines[i].x == parking_lines[i + 1].x)
			{
				sum += distance(parking_lines[i], parking_lines[i + 1]);
				n_of_spots++;
			}
			else
			{
				break;
			}
		}
	}

	float average = sum / n_of_spots;
	std::cout << "NUMBER OF SPOTS:" << n_of_spots << std::endl;
	std::cout << "AVERAGE:" << n_of_spots << std::endl;
	return average;
}

void PLApplication::detectLines()
{
	std::cout << "DETECTING LINES" << std::endl;
	for (int i = 0; i < spots.size(); i++)
	{
		cv::Point2f point = spots[i];
		cv::Rect rect_mat(0, 0, roi.cols, roi.rows);
		cv::Rect rectLine;
		for (;;)
		{
			rectLine = cv::Rect(point.x + JUMP, point.y - (CHUNK_HEIGHT / 2.0), JUMP, CHUNK_HEIGHT);
			if ((rectLine & rect_mat) == rectLine)
			{
				cv::Mat line = roi_empty_gray(rectLine);
				if (cv::mean(line)[0] > 0 && can_count_line) // line
				{
					can_count_line = false;
					circle(roi, point, 2, RED, 2, CV_AA);
					cv::imshow("RESULT", roi);
					circle(roi_empty, point, 2, RED, 2, CV_AA);
					cv::imshow("RESULT__", roi_empty);
					parking_lines.push_back(cv::Point2f(point.x + 5, point.y));
				}
				else if (cv::mean(line)[0] == 0)
				{
					can_count_line = true;
				}
				point = cv::Point2f(point.x + JUMP, point.y);
			}
			else
			{
				break;
			}
		}
	}
}



void PLApplication::filterLines()
{
	float average = averageLineDistance();
}

void PLApplication::createSpots()
{
	cv::Rect rect_mat(0, 0, roi.cols, roi.rows);
	for (int i = 0; i < parking_lines.size() - 1; i++)
	{
		cv::Point2f p1 = parking_lines[i];
		cv::Point2f p2 = parking_lines[i + 1];
		cv::Rect parking_spot = cv::Rect(p1.x, p1.y - CHUNK_HEIGHT / 2.0, distance(p1, p2), CHUNK_HEIGHT);
		if ((parking_spot & rect_mat) == parking_spot)
		{
			std::cout << "ADDING A SPOT" << std::endl;
			parking_spots.push_back(parking_spot);
		}
	}
}

void PLApplication::filterSpots()
{
	std::cout << "FILTETING SPOTS" << std::endl;
	std::vector<cv::Rect> new_parking_spots;
	float average = averageSpotWidth();
	std::cout << "AVERAGE: " << average << std::endl;
	for (int i = 0; i < parking_spots.size(); i++)
	{
		if (parking_spots[i].width > (average / 2.0))
		{
			new_parking_spots.push_back(parking_spots[i]);
		}
	}
	parking_spots = new_parking_spots;
}

void PLApplication::countCars()
{
	std::cout << "COUNTING CARS" << std::endl;
	
	detectLines();
	//filterLines();
	createSpots();
	filterSpots();

	cv::Rect rect_mat(0, 0, roi.cols, roi.rows);

	for (int i = 0; i < parking_spots.size(); i++)
	{
		analyzeSpot(parking_spots[i]);
	}
}

void PLApplication::analyzeSpot(cv::Rect rect)
{
	cv::Mat parking_spot = roi_gray(rect);
	cv::Scalar _mean = mean(parking_spot);
	std::cout << "MEAN :" << _mean << std::endl;
	if (_mean[0] < 95 || _mean[0] > 120)
	{
		cv::rectangle(roi, rect, GREEN, 2);
		imshow("RESULT", roi);
		total_cars++;
	}
}

int main(int argc, char** argv)
{
	cv::Mat image = cv::imread(argv[1]);
	cv::Mat image_empty = cv::imread("lot_empty.jpg");
	
	plapp.init(image, image_empty);
	
	std::cout << "Click on four corners -- top left first and bottom left last -- and then hit ENTER" << std::endl;

	// Show image and wait for 4 clicks. 
	cv::imshow("PARKING LOT", plapp.image);
	// Set the callback function for any mouse event
	cv::setMouseCallback("PARKING LOT", selectCorners, 0);
	cv::waitKey(0);
	cvDestroyWindow("PARKING LOT");

	float sizeW = distance(plapp.corners[0], plapp.corners[1]);
	float sizeH = distance(plapp.corners[1], plapp.corners[2]);

	cv::Size size(sizeW, sizeH);
	cv::Mat dst = cv::Mat::zeros(size, CV_8UC3);
	cv::Mat dst_empty = cv::Mat::zeros(size, CV_8UC3);

	// Create a vector of destination points
	std::vector<cv::Point2f> cornersDst;

	cornersDst.push_back(cv::Point2f(0, 0));
	cornersDst.push_back(cv::Point2f(size.width - 1, 0));
	cornersDst.push_back(cv::Point2f(size.width - 1, size.height - 1));
	cornersDst.push_back(cv::Point2f(0, size.height - 1));

	// Calculate the homography
	cv::Mat h = cv::findHomography(plapp.corners, cornersDst);

	// Warp source image to destination
	warpPerspective(image, dst, h, size);
	warpPerspective(image_empty, dst_empty, h, size);

	dst.copyTo(plapp.roi);
	dst_empty.copyTo(plapp.roi_empty);
	// Show image
	cv::imshow("RESULT", plapp.roi);
	//GrayScale for thresholding
	cv::cvtColor(plapp.roi_empty, plapp.roi_empty_gray, CV_BGR2GRAY);
	cv::cvtColor(plapp.roi, plapp.roi_gray, CV_BGR2GRAY);
	cv::threshold(plapp.roi_empty_gray, plapp.roi_empty_gray, THRESHOLD, 255, cv::THRESH_BINARY);
	
	//cv::Mat erosion_element = cv::getStructuringElement(EROSION_TYPE, cv::Size(2 * EROSION_SIZE + 1, 2 * EROSION_SIZE + 1), cv::Point(EROSION_SIZE, EROSION_SIZE));
	//cv::erode(plapp.roi_empty_gray, plapp.roi_empty_gray, erosion_element);
	cv::Mat dilation_element = cv::getStructuringElement(DILATION_TYPE, cv::Size(2 * DILATION_SIZE + 1, 2 * DILATION_SIZE + 1), cv::Point(DILATION_SIZE, DILATION_SIZE));
	cv::dilate(plapp.roi_empty_gray, plapp.roi_empty_gray, dilation_element);

	imshow("GRAY", plapp.roi_empty_gray);
	imshow("GRAY ROI", plapp.roi_gray);

	cv::setMouseCallback("RESULT", selectSpots, 0);
	cv::waitKey(0);

	plapp.countCars();
	std::cout << "FOUND " << plapp.getTotalCars() << std::endl;

	cv::waitKey(0);
	return 0;
}