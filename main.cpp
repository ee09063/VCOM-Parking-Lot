/*
* Program Use: <executable> <image>
*/
#include <opencv2/opencv.hpp>
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "Constants.h"
#include "Blob.h"
#include <iostream>
#include <cmath>


int JUMP = 3;


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
	cv::Mat roi_final;

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
	void addLastLine();
	float averageLineDistancePerQueue(int i);
	float averageLineDistance();
	void paintPoint(cv::Point2f point, cv::Scalar color);
	void analyzeSpot(cv::Rect rect);
};

PLApplication plapp;

//Mouse callback to select the 4 initial corners
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

//Mouse callback to select the queues
void selectQueues(int event, int x, int y, int flags, void* param)
{
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		circle(plapp.roi, cv::Point(x, y), 3, RED, 5, CV_AA);
		cv::imshow("REGION OF INTEREST", plapp.roi);
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
	cv::imshow("REGION OF INTEREST", roi);
	circle(roi_empty, point, 2, color, 2, CV_AA);
	cv::imshow("EMPTY REGION OF INTEREST", roi_empty);
}

//Average distance between each dividing line, per queue
float PLApplication::averageLineDistancePerQueue(int i)
{
	float sum = 0;
	float number_of_spots = 0;

	for (int k = 0; k < parking_lines[i].size() - 1; k++)
	{
		cv::Point2f p1 = parking_lines[i][k];
		cv::Point2f p2 = parking_lines[i][k + 1];
		sum += distance(p1, p2);
		number_of_spots += 1.0;
	}

	return sum / number_of_spots;
}

//Average distance between each dividing line
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

//Executes an horizontal sweep to detect the lines
void PLApplication::detectLines()
{
	cv::Point2f fp, sp;
	for (int i = 0; i < queues.size(); i++)
	{
		parking_lines.push_back(std::vector<cv::Point2f>());
		// The queue point the user selected
		cv::Point2f point = queues[i];
		// This point is always valid 
		parking_lines[i].push_back(point);
		cv::Rect rect_mat(0, 0, roi.cols, roi.rows);
		cv::Rect rectLine;
		for (;;)
		{
			rectLine = cv::Rect(point.x + JUMP, point.y - (SPOT_HEIGHT / 2.0), JUMP, SPOT_HEIGHT);
			// Check if rectLine is inside the image
			if ((rectLine & rect_mat) == rectLine)
			{
				cv::Mat line = roi_empty_gray(rectLine);
				// Check for start of line
				if (cv::mean(line)[0] > 0)
				{
					fp = point;
					for (;;)
					{
						rectLine = cv::Rect(point.x + JUMP, point.y - (SPOT_HEIGHT / 2.0), JUMP, SPOT_HEIGHT);
						if ((rectLine & rect_mat) == rectLine)
						{
							cv::Mat line = roi_empty_gray(rectLine);
							// Check for end of line
							if (cv::mean(line)[0] == 0)
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

//Filters the lines to remove false positive points
//by comparing the distance of consecutive points
void PLApplication::filterLines()
{
	std::vector<std::vector<cv::Point2f>> new_parking_lines;

	for (int i = 0; i < parking_lines.size(); i++)
	{
		float average = averageLineDistancePerQueue(i);
		float average_threshold = average * LINE_DISTANCE_THRES;
		new_parking_lines.push_back(std::vector<cv::Point2f>());

		paintPoint(parking_lines[i][0], GREEN);

		//The user point is always valid
		new_parking_lines[i].push_back(parking_lines[i][0]);

		for (int k = 0; k < parking_lines[i].size() - 1; k++)
		{
			// Compare with the last valid point added
			cv::Point2f p0 = new_parking_lines[i][new_parking_lines[i].size() - 1];
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

// Create the parking spots based on the parking lines
void PLApplication::createSpots()
{
	cv::Rect rect_mat(0, 0, roi.cols, roi.rows);
	for (int i = 0; i < parking_lines.size(); i++)
	{
		for (int k = 0; k < parking_lines[i].size() - 1; k++)
		{
			cv::Point2f p1 = parking_lines[i][k];
			cv::Point2f p2 = parking_lines[i][k + 1];
			cv::Rect parking_spot = cv::Rect(p1.x, p1.y - SPOT_HEIGHT / 2.0, abs(p2.x - p1.x), SPOT_HEIGHT);
			if ((parking_spot & rect_mat) == parking_spot)
			{
				parking_spots.push_back(parking_spot);
			}
		}
	}
}

//The last parking spot of the queues doesn't 
//have a white line on the right side, so add it
void PLApplication::addLastLine()
{
	for (int i = 0; i < parking_lines.size(); i++)
	{
		float average_distance = averageLineDistancePerQueue(i);
		cv::Point last_line = parking_lines[i][parking_lines[i].size() - 1];
		cv::Point2f last_point = cv::Point2f(last_line.x + average_distance, last_line.y);
		paintPoint(last_point, GREEN);
		parking_lines[i].push_back(last_point);
	}
}

void PLApplication::countCars()
{
	std::cout << "COUNTING CARS" << std::endl;

	detectLines();
	filterLines();
	addLastLine();
	createSpots();

	cvDestroyWindow("REGION OF INTEREST");

	cv::Rect rect_mat(0, 0, roi.cols, roi.rows);

	for (int i = 0; i < parking_spots.size(); i++)
	{
		analyzeSpot(parking_spots[i]);
	}
}

// Check the parking spots and mark them 
void PLApplication::analyzeSpot(cv::Rect rect)
{
	cv::Mat parking_spot = roi_hsv(rect);
	cv::Scalar _mean = mean(parking_spot);
	float hue = _mean[0];
	float saturation = _mean[1];
	float value = _mean[2];

	std::cout << "HUE | SATURATION | VALUE: " << _mean << std::endl;

	if (saturation > SATURATION_THRESHOLD || hue > HUE_THESHOLD)
	{
		//Mark a filled spot
		cv::rectangle(roi_final, rect, GREEN, 2);
		imshow("FINAL REGION OF INTEREST", roi_final);
		total_cars++;
	}
	else
	{
		//Mark an empty spot
		cv::rectangle(roi_final, rect, RED, 2);
		imshow("FINAL REGION OF INTEREST", roi_final);
	}
}

int main(int argc, char** argv)
{
	cv::Mat image = cv::imread("C:/Dev/lot2.jpg");
	//The empty image
	cv::Mat image_empty = cv::imread("C:/Dev/parking_empty.jpg");

	// Check images
	if (!image.data)
	{
		std::cout << "Could not open or find image 1" << std::endl;
		return -1;
	}
	if (!image_empty.data)
	{
		std::cout << "Could not open or find image 2" << std::endl;
		return -1;
	}

	plapp.init(image, image_empty);

	std::cout << "Click on four corners -- top left first and bottom left last -- and then hit any Key" << std::endl;

	// Show image and wait for 4 clicks. 
	cv::imshow("PARKING LOT", plapp.image);
	// Set the callback function for any mouse event
	cv::setMouseCallback("PARKING LOT", selectCorners, 0);
	cv::waitKey(0);
	cvDestroyWindow("PARKING LOT");

	//Set the size for the region of interest
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

	// call blob counting function
	Blob::countCars_blobs(dst_empty, dst);

	/*

	dst.copyTo(plapp.roi);
	dst_empty.copyTo(plapp.roi_empty);
	dst.copyTo(plapp.roi_final);
	// Show REGION OF INTEREST
	cv::imshow("REGION OF INTEREST", plapp.roi);
	//GrayScale the empty parking lot for thresholding
	cv::cvtColor(plapp.roi_empty, plapp.roi_empty_gray, CV_BGR2GRAY);
	//Convert the non empty region of interest to HSV to detect cars
	cv::cvtColor(plapp.roi, plapp.roi_hsv, CV_BGR2HSV);
	cv::threshold(plapp.roi_empty_gray, plapp.roi_empty_gray, THRESHOLD, 255, cv::THRESH_BINARY);

	cv::imshow("GRAY THRESH", plapp.roi_empty_gray);

	cv::Mat dilation_element = cv::getStructuringElement(DILATION_TYPE, cv::Size(2 * DILATION_SIZE + 1, 2 * DILATION_SIZE + 1), cv::Point(DILATION_SIZE, DILATION_SIZE));
	cv::dilate(plapp.roi_empty_gray, plapp.roi_empty_gray, dilation_element);

	std::cout << "Click on each queue -- and then hit any Key" << std::endl;

	cv::setMouseCallback("REGION OF INTEREST", selectQueues, 0);
	cv::waitKey(0);

	plapp.countCars();
	std::cout << "FOUND " << plapp.getTotalCars() << std::endl;

	*/

	cv::waitKey(0);
	return 0;
}