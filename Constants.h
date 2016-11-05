// Constants.h
#include<opencv2/core/core.hpp>
#include "opencv2/imgproc.hpp" // MORPH_ELLIPSE


#if !defined(MYLIB_CONSTANTS_H)
#define MYLIB_CONSTANTS_H 1

	const cv::Scalar BLACK = cv::Scalar(0, 0, 0);
	const cv::Scalar WHITE = cv::Scalar(255, 255, 255);
	const cv::Scalar GREEN = cv::Scalar(0, 255, 0);
	const cv::Scalar RED = cv::Scalar(0, 0, 255);

	const int SPOT_HEIGHT = 40;
	const int SPOT_WIDTH = 25;
	const int DILATION_SIZE = 2;
	const int DILATION_TYPE = cv::MORPH_ELLIPSE;
	const int THRESHOLD = 190;
	const float LINE_DISTANCE_THRES = 0.6;
	const float HUE_THESHOLD = 50;
	const float SATURATION_THRESHOLD = 50;

	// Blob:
	const int structuringRectSize = 5; // 3, 5, 7
	const int blurSize = 5;

	const int blobRectAreaLowerLimit = 80;
	const int blobAspectRatioLowerLimit = 0.15;
	const int blobAspectRatioUpperLimit = 1.25;
	const int blobRectMinWidth = 15;
	const int blobRectMinHeight = 15;

#endif