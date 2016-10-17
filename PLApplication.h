#ifndef PLAPPLICATION_H
#define PLAPPLICATION_H

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

using namespace std;
using namespace cv;

const Scalar RED = Scalar(0, 0, 255);
const Scalar PINK = Scalar(230, 130, 255);
const Scalar BLUE = Scalar(255, 0, 0);
const Scalar LIGHTBLUE = Scalar(255, 255, 160);
const Scalar GREEN = Scalar(0, 255, 0);

class PLApplication
{
public:
	enum{ NOT_SET = 0, IN_PROCESS = 1, SET = 2 };
	Mat image;
	Mat roi;

	void reset();
	void setImageAndWinName(Mat _image, const string& _winName);
	void showImage() const;
	void mouseClick(int event, int x, int y, int flags, void* param);
	void computeHistogram(Mat image, bool color);
	void computeColorHistogram(Mat image);
	void computeGrayHistogram(Mat image);
private:
	void setRectInMask();

	const string* winName;

	uchar rectState;
	bool isInitialized;

	Rect rect;
};

#endif