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

const int SPOT_HEIGHT = 40;
const int SPOT_WIDTH = 25;
const int CUTOFF = 50;
const int DISTANCE = 5;
int JUMP = 3;
const int EROSION_SIZE = 1;
const int EROSION_TYPE = cv::MORPH_ELLIPSE;
const int DILATION_SIZE = 1;
const int DILATION_TYPE = cv::MORPH_ELLIPSE;
const int THRESHOLD = 125;
const float LINE_DISTANCE_THRES = 0.6;
const float SATURATION_THRESHOLD = 55;

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
	cv::Mat roi_hsv;
	cv::Mat roi_empty;
	cv::Mat roi_empty_gray;
	
	std::vector<cv::Point2f> corners;
	std::vector<cv::Point2f> queues;
	std::vector<std::vector<cv::Point2f>> parking_lines;
	std::vector<cv::Rect> parking_spots;

	void reset();
	void init(cv::Mat _image, cv::Mat _image_empty);
	void countCars();
	int getTotalCars();
private:
	int total_cars = 0;
	void detectLines();
	void filterLines();
	void createSpots();
	float averageLineDistancePerQueue(int i);
	float averageLineDistance();
	void paintPoint(cv::Point2f point, cv::Scalar color);
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
		plapp.queues.push_back(cv::Point2f(x, y));
	}
}

void PLApplication::init(cv::Mat _image, cv::Mat _image_empty)
{
	_image.copyTo(image);
	_image_empty.copyTo(image_empty);
}

int PLApplication::getTotalCars()
{
	return total_cars;
}

void PLApplication::paintPoint(cv::Point2f point, cv::Scalar color)
{
	circle(roi, point, 2, color, 2, CV_AA);
	cv::imshow("RESULT", roi);
	circle(roi_empty, point, 2, color, 2, CV_AA);
	cv::imshow("RESULT__", roi_empty);
}

float PLApplication::averageLineDistancePerQueue(int i)
{
	float sum = 0;
	float number_of_spots = 0;

	for (int k = 0; k < parking_lines[i].size() - 1; k++)
	{
		cv::Point2f p1 = parking_lines[i][k];
		cv::Point2f p2 = parking_lines[i][k+1];
		sum += distance(p1, p2);
		number_of_spots += 1.0;
	}

	return sum / number_of_spots;
}

float PLApplication::averageLineDistance()
{
	float sum = 0;
	float number_of_spots = 0;
	for (int i = 0; i < parking_lines.size(); i++)
	{
		for (int k = 0; k < parking_lines[i].size() - 1; k++)
		{
			cv::Point2f p1 = parking_lines[i][k];
			cv::Point2f p2 = parking_lines[i][k + 1];
			sum += distance(p1, p2);
			number_of_spots += 1.0;
		}
	}

	return sum / number_of_spots;
}



void PLApplication::detectLines()
{
	cv::Point2f fp, sp;
	for (int i = 0; i < queues.size(); i++)
	{
		parking_lines.push_back(std::vector<cv::Point2f>());
		cv::Point2f point = queues[i]; // user point
		parking_lines[i].push_back(point);
		cv::Rect rect_mat(0, 0, roi.cols, roi.rows);
		cv::Rect rectLine;
		for (;;)
		{
			rectLine = cv::Rect(point.x + JUMP, point.y - (SPOT_HEIGHT / 2.0), JUMP, SPOT_HEIGHT);
			if ((rectLine & rect_mat) == rectLine) // rect is inside image
			{
				cv::Mat line = roi_empty_gray(rectLine);
				if (cv::mean(line)[0] > 0) // found start of line
				{
					fp = point;
					for (;;)
					{
						rectLine = cv::Rect(point.x + JUMP, point.y - (SPOT_HEIGHT / 2.0), JUMP, SPOT_HEIGHT);
						if ((rectLine & rect_mat) == rectLine)
						{
							cv::Mat line = roi_empty_gray(rectLine);
							if (cv::mean(line)[0] == 0) // found end of line
							{
								sp = point;
								cv::Point2f middle_point = cv::Point2f((fp.x + sp.x) / 2.0, (fp.y + sp.y) / 2.0);
								paintPoint(middle_point, RED);
								parking_lines[i].push_back(middle_point);
								break;
							}
							else
							{
								point = cv::Point2f(point.x + JUMP, point.y);
							}
						}
					}
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
	std::vector<std::vector<cv::Point2f>> new_parking_lines;
	//float average = averageLineDistance();

	for (int i = 0; i < parking_lines.size(); i++)
	{
		float average = averageLineDistancePerQueue(i);
		float average_threshold = average * LINE_DISTANCE_THRES;
		new_parking_lines.push_back(std::vector<cv::Point2f>());
		
		paintPoint(parking_lines[i][0], GREEN);
		
		new_parking_lines[i].push_back(parking_lines[i][0]);

		for (int k = 0; k < parking_lines[i].size() - 1; k++)
		{
			//cv::Point2f p0 = parking_lines[i][k];
			cv::Point2f p0 = new_parking_lines[i][new_parking_lines[i].size() - 1]; // last valid point added
			cv::Point2f p1 = parking_lines[i][k + 1];
			float point_distance = distance(p0, p1);
			if (point_distance > average_threshold)
			{
				paintPoint(p1, GREEN);
				new_parking_lines[i].push_back(p1);
			}
		}
	}

	parking_lines = new_parking_lines;
}

void PLApplication::createSpots()
{
	cv::Rect rect_mat(0, 0, roi.cols, roi.rows);
	for (int i = 0; i < parking_lines.size(); i++)
	{
		for (int k = 0; k < parking_lines[i].size() - 1; k++)
		{
			cv::Point2f p1 = parking_lines[i][k];
			cv::Point2f p2 = parking_lines[i][k + 1];
			cv::Rect parking_spot = cv::Rect(p1.x, p1.y - SPOT_HEIGHT / 2.0, distance(p1, p2), SPOT_HEIGHT);
			if ((parking_spot & rect_mat) == parking_spot)
			{
				std::cout << "ADDING A SPOT" << std::endl;
				parking_spots.push_back(parking_spot);
			}
		}
	}
}

void PLApplication::countCars()
{
	std::cout << "COUNTING CARS" << std::endl;
	
	detectLines();
	filterLines();
	createSpots();

	cv::Rect rect_mat(0, 0, roi.cols, roi.rows);

	for (int i = 0; i < parking_spots.size(); i++)
	{
		analyzeSpot(parking_spots[i]);
	}
}

void PLApplication::analyzeSpot(cv::Rect rect)
{
	cv::Mat parking_spot = roi_hsv(rect);
	cv::Scalar _mean = mean(parking_spot);
	std::cout << "MEAN :" << _mean << std::endl;
	float saturation = _mean[1];
	if (saturation > SATURATION_THRESHOLD)
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
	cv::cvtColor(plapp.roi, plapp.roi_hsv, CV_BGR2HSV);
	cv::threshold(plapp.roi_empty_gray, plapp.roi_empty_gray, THRESHOLD, 255, cv::THRESH_BINARY);
	
	cv::Mat dilation_element = cv::getStructuringElement(DILATION_TYPE, cv::Size(2 * DILATION_SIZE + 1, 2 * DILATION_SIZE + 1), cv::Point(DILATION_SIZE, DILATION_SIZE));
	cv::dilate(plapp.roi_empty_gray, plapp.roi_empty_gray, dilation_element);

	imshow("GRAY", plapp.roi_empty_gray);

	cv::setMouseCallback("RESULT", selectSpots, 0);
	cv::waitKey(0);

	plapp.countCars();
	std::cout << "FOUND " << plapp.getTotalCars() << std::endl;

	cv::waitKey(0);
	return 0;
}