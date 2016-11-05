// Blob.cpp

#include "Blob.h"
#include <iostream>
#include <string.h>
#include <sstream>

using namespace cv;

///////////////////////////////////////////////////////////////////////////////////////////////////
Blob::Blob(std::vector<cv::Point> _contour) {

	contour = _contour;

	boundingRect = cv::boundingRect(contour);

	centerPosition.x = (boundingRect.x + boundingRect.x + boundingRect.width) / 2;
	centerPosition.y = (boundingRect.y + boundingRect.y + boundingRect.height) / 2;

	dblDiagonalSize = sqrt(pow(boundingRect.width, 2) + pow(boundingRect.height, 2));

	dblAspectRatio = (float) boundingRect.width / (float) boundingRect.height;

}

void Blob::countCars_blobs(Mat emptyParking, Mat fullParking) {
	unsigned int nCars = 0;

	cv::Mat imgDifference;
	cv::Mat imgThreshold;

	Mat fullCopy = fullParking.clone();

	std::vector<Blob> blobVec;

	// convert images to grayscale
	cv::cvtColor(emptyParking, emptyParking, CV_BGR2GRAY);
	cv::cvtColor(fullParking, fullParking, CV_BGR2GRAY);

	// apply gaussian blur
	cv::GaussianBlur(emptyParking, emptyParking, cv::Size(blurSize, blurSize), 0);
	cv::GaussianBlur(fullParking, fullParking, cv::Size(blurSize, blurSize), 0);

	// compute difference between pictures
	cv::absdiff(emptyParking, fullParking, imgDifference);

	// apply threshold
	cv::threshold(imgDifference, imgThreshold, 80, 255.0, CV_THRESH_BINARY);
	cv::imshow("imgThreshold", imgThreshold);

	// get, dilate and erode structuring element
	cv::Mat structElement = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(structuringRectSize, structuringRectSize));
	cv::dilate(imgThreshold, imgThreshold, structElement);
	cv::dilate(imgThreshold, imgThreshold, structElement);
	cv::erode(imgThreshold, imgThreshold, structElement);

	// find & draw contours
	cv::Mat imgThresholdCopy = imgThreshold.clone();
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(imgThresholdCopy, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	cv::Mat imgContours(imgThreshold.size(), CV_8UC3, BLACK);
	cv::drawContours(imgContours, contours, -1, WHITE, -1);
	cv::imshow("imgContours", imgContours);

	// find convex hull for each contour

	vector<vector<Point>> hulls(contours.size());

	for (unsigned int i = 0; i < contours.size(); i++)
	{
		cv::convexHull(contours[i], hulls[i]);
	}

	for (unsigned int i = 0; i < hulls.size(); i++)
	{
		Blob possibleBlob(hulls[i]);

		if (possibleBlob.boundingRect.area() > blobRectAreaLowerLimit &&
			possibleBlob.dblAspectRatio >= blobAspectRatioLowerLimit &&
			possibleBlob.dblAspectRatio <= blobAspectRatioUpperLimit &&
			possibleBlob.boundingRect.width > blobRectMinWidth &&
			possibleBlob.boundingRect.height > blobRectMinHeight)
			blobVec.push_back(possibleBlob);
	}

	cv::Mat convexHulls(imgThreshold.size(), CV_8UC3, BLACK);

	hulls.clear();

	for (auto &blob : blobVec) {
		hulls.push_back(blob.contour);
	}

	cv::drawContours(convexHulls, hulls, -1, WHITE, -1);

	cv::imshow("convexHulls", convexHulls);

	for (auto &blob : blobVec) {
		cv::rectangle(fullCopy, blob.boundingRect, GREEN, 2);
		nCars++;
	}

	std::ostringstream nCars_string;
	nCars_string << nCars;
	string outStr = "Final Blob Counting - " + (string)nCars_string.str() + " cars";
	std::cout << outStr << std::endl;

	cv::imshow(outStr, fullCopy);
}