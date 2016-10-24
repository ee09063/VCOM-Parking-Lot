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
const int EROSION_SIZE = 5;
const int DILATION_SIZE = 5;
const int EROSION_TYPE = cv::MORPH_ELLIPSE;
const int DILATION_TYPE = cv::MORPH_ELLIPSE;
const int THRESHOLD = 125;
const int THRESHHOLD_INV = 50;

class PLApplication
{
public:
	enum{ NOT_SET = 0, IN_PROCESS = 1, SET = 2 };
	cv::Mat image; //Loaded Image
	cv::Mat roi; // Region of Interest for Homography
	cv::Mat gray_roi; // GrayScale ROI for Thresholding

	std::vector<cv::Point2f> corners;
	std::vector<cv::Point2f> spots;

	void reset();
	void init(cv::Mat _image);
	void countCars();
	int getTotalCars();
private:
	bool can_count;
	int total_cars = 0;
	void analyzeSpot(cv::Point2f point);
	bool hasCar(cv::Scalar _mean);

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

void PLApplication::init(cv::Mat _image)
{
	_image.copyTo(image);
	can_count = true;
}

void PLApplication::countCars()
{
	std::cout << "COUNTING CARS" << std::endl;
	
	for (int i = 0; i < spots.size(); i++)
	{
		cv::Point2f point = spots[i];
		cv::Rect rect_mat(0, 0, roi.cols, roi.rows);
		cv::Rect rectLine;
		cv::Rect rectSquare;
		for (;;)
		{
			rectSquare = cv::Rect(point.x /*- (CHUNK_WIDTH / 2.0)*/, point.y - (CHUNK_HEIGHT / 2.0), CHUNK_WIDTH, CHUNK_HEIGHT);
			rectLine = cv::Rect(point.x + JUMP, point.y - (CHUNK_HEIGHT / 2.0), JUMP, CHUNK_HEIGHT);
			if ((rectLine & rect_mat) == rectLine && (rectSquare & rect_mat) == rectSquare)
			{
				cv::Mat line = gray_roi(rectLine);
				if (cv::mean(line)[0] > 0)
				{
					analyzeSpot(point);
				}
				else
				{
					can_count = true;
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

void PLApplication::analyzeSpot(cv::Point2f point)
{
	cv::Rect rect;
	rect.x = point.x;// -(CHUNK_WIDTH / 2.0);
	rect.y = point.y - (CHUNK_HEIGHT / 2.0);
	rect.width = CHUNK_WIDTH;
	rect.height = CHUNK_HEIGHT;
	
	cv::Mat sample_roi = plapp.gray_roi(rect);
	cv::Scalar roi_mean = mean(sample_roi);
	
	if (hasCar(roi_mean))
	{
		std::cout << "FOUND 1" << std::endl;
		total_cars++;
		cv::rectangle(roi, rect, GREEN, 2);
		JUMP = 3;
	}
	imshow("RESULT", roi);
}

bool PLApplication::hasCar(cv::Scalar _mean)
{
	std::cout << "MEAN: " << _mean << std::endl;
	
	if (_mean[0] > CUTOFF && can_count)
	{
		can_count = false;
		return true;
	}
	else if (_mean[0] < CUTOFF)
	{
		can_count = true;
		return false;
	}
	return false;
}

int PLApplication::getTotalCars()
{
	return total_cars;
}

double distance(cv::Point2f p0, cv::Point2f p1)
{
	double dX0 = p0.x, dY0 = p0.y, dX1 = p1.x, dY1 = p1.y;
	return std::sqrt((dX1 - dX0)*(dX1 - dX0) + (dY1 - dY0)*(dY1 - dY0));
}

int main(int argc, char** argv)
{
	cv::Mat src = cv::imread(argv[1]);
	
	plapp.init(src);
	
	std::cout << "Click on four corners -- top left first and bottom left last -- and then hit ENTER" << std::endl;

	// Show image and wait for 4 clicks. 
	cv::imshow("PARKING LOT", plapp.image);
	// Set the callback function for any mouse event
	cv::setMouseCallback("PARKING LOT", selectCorners, 0);
	cv::waitKey(0);
	cvDestroyWindow("PARKING_LOT");

	float sizeW = distance(plapp.corners[0], plapp.corners[1]);
	float sizeH = distance(plapp.corners[1], plapp.corners[2]);

	cv::Size size(sizeW, sizeH);
	cv::Mat dst = cv::Mat::zeros(size, CV_8UC3);
	// Create a vector of destination points
	std::vector<cv::Point2f> cornersDst;

	cornersDst.push_back(cv::Point2f(0, 0));
	cornersDst.push_back(cv::Point2f(size.width - 1, 0));
	cornersDst.push_back(cv::Point2f(size.width - 1, size.height - 1));
	cornersDst.push_back(cv::Point2f(0, size.height - 1));

	// Calculate the homography
	cv::Mat h = cv::findHomography(plapp.corners, cornersDst);

	// Warp source image to destination
	warpPerspective(src, dst, h, size);

	dst.copyTo(plapp.roi);
	// Show image
	cv::imshow("RESULT", dst);
	//GrayScale for thresholding
	cv::Mat thresh, thresh_inv;
	cv::cvtColor(dst, dst, CV_BGR2GRAY);
	cv::threshold(dst, thresh, THRESHOLD, 255, cv::THRESH_BINARY);
	cv::threshold(dst, thresh_inv, THRESHHOLD_INV, 255, cv::THRESH_BINARY_INV);
	
	cv::Mat erosion_element = cv::getStructuringElement(EROSION_TYPE, cv::Size(2 * EROSION_SIZE + 1, 2 * EROSION_SIZE + 1), cv::Point(EROSION_SIZE, EROSION_SIZE));
	cv::erode(thresh, thresh, erosion_element);
	cv::erode(thresh_inv, thresh_inv, erosion_element);
	cv::Mat dilation_element = cv::getStructuringElement(DILATION_TYPE, cv::Size(2 * DILATION_SIZE + 1, 2 * DILATION_SIZE + 1), cv::Point(DILATION_SIZE, DILATION_SIZE));
	cv::dilate(thresh, thresh, dilation_element);
	cv::erode(thresh_inv, thresh_inv, erosion_element);

	cv::Mat sum_mat = thresh; //+ thresh_inv;
	sum_mat.copyTo(plapp.gray_roi);

	//cv::imshow("THESH", thresh);
	//cv::imshow("THESH_INV", thresh_inv);
	cv::imshow("SUM_MAT", sum_mat);

	cv::setMouseCallback("RESULT", selectSpots, 0);
	cv::waitKey(0);

	plapp.countCars();
	std::cout << "FOUND " << plapp.getTotalCars() << std::endl;

	cv::waitKey(0);
	return 0;
}

/*
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
	Mat gray_roi;

	void reset();
	void setImageAndWinName(Mat _image, const string& _winName);
	void showImage() const;
	void showROI() const;
	void mouseClick(int event, int x, int y, int flags, void* param);
private:
	void setRectInMask();
	void setSampleRectInMask();
	void divideROIWithSample();
	void countCars(vector<Mat> spaces, Scalar mean);

	const string* winName;

	uchar rectState;
	uchar sampleState;
	bool isInitialized;

	Rect rect;
	Rect sampleRect;

	Scalar emptyMean;
};
PLApplication plapp;

static void on_mouse(int event, int x, int y, int flags, void* param)
{
	plapp.mouseClick(event, x, y, flags, param);
}

int main(int argc, char** argv)
{
	string filename = argv[1];
	if (filename.empty())
	{
		cout << "Empty filename" << endl;
		return 1;
	}

	Mat image = imread(filename, 1);

	if (image.empty())
	{
		cout << "Couldn't read image filename " << filename << endl;
		return 1;
	}

	const string winName = "PARKING LOT";
	namedWindow(winName, WINDOW_AUTOSIZE);
	setMouseCallback(winName, on_mouse, 0);

	plapp.setImageAndWinName(image, winName);
	plapp.showImage();

	waitKey(0);
	return 0;
}

void PLApplication::reset()
{
	isInitialized = false;
	rectState = NOT_SET;
	sampleState = NOT_SET;
}

void PLApplication::setImageAndWinName(Mat _image, const string& _winName)
{
	image = _image;
	winName = &_winName;
	reset();
}

void PLApplication::showImage() const
{
	if (image.empty() || winName->empty())
		return;

	Mat res;
	Mat binMask;
	if (!isInitialized)
		image.copyTo(res);
	else
	{
		image.copyTo(res, binMask);
	}

	if (rectState == IN_PROCESS || rectState == SET)
		rectangle(res, Point(rect.x, rect.y), Point(rect.x + rect.width, rect.y + rect.height), GREEN, 2);

	imshow("PARKING LOT", res);
}

void PLApplication::showROI() const
{
	if (roi.empty())
		return;

	Mat res;
	Mat binMask;
	if (!isInitialized)
		roi.copyTo(res);
	else
	{
		roi.copyTo(res, binMask);
	}

	if (rectState == SET && (sampleState == IN_PROCESS || sampleState == SET))
		rectangle(res, Point(sampleRect.x, sampleRect.y), Point(sampleRect.x + sampleRect.width, sampleRect.y + sampleRect.height), GREEN, 2);

	imshow("ROI", res);
}

void PLApplication::setRectInMask()
{
	rect.x = max(0, rect.x);
	rect.y = max(0, rect.y);
	rect.width = min(rect.width, image.cols - rect.x);
	rect.height = min(rect.height, image.rows - rect.y);

	//
	roi = image(rect);
	namedWindow("ROI", WINDOW_AUTOSIZE);
	imshow("ROI", roi);

	//computeHistogram(roi, true);
	cvtColor(roi, gray_roi, CV_BGR2GRAY);
	//computeHistogram(grayImage, false);

	Mat thres, invThresh;
	threshold(gray_roi, thres, 125, 255, THRESH_BINARY);
	threshold(gray_roi, invThresh, 75, 255, THRESH_BINARY_INV);

	imshow("THRESH", thres);
	imshow("THRESH_INV", invThresh);
	
	Mat sumMat = thres + invThresh;

	imshow("SUM", sumMat);

	int erosion_type = MORPH_ELLIPSE;
	int erosion_size = 2;

	Mat element = getStructuringElement(erosion_type, Size(2 * erosion_size + 1, 2 * erosion_size + 1), Point(erosion_size, erosion_size));

	/// Apply the erosion operation
	erode(sumMat, sumMat, element);

	cout << "MEAN OF ROI " << mean(gray_roi) << endl;
	
	//setMouseCallback("ROI", on_mouse, 0);
}

void PLApplication::setSampleRectInMask()
{
	sampleRect.x = max(0, sampleRect.x);
	sampleRect.y = max(0, sampleRect.y);
	sampleRect.width = min(sampleRect.width, roi.cols - sampleRect.x);
	sampleRect.height = min(sampleRect.height, roi.rows - sampleRect.y);

	//
	Mat newRoi = roi(sampleRect);
	imshow("ROI_OF_ROI", newRoi);

	Mat grayImage;
	cvtColor(newRoi, grayImage, CV_BGR2GRAY);

	cout << "MEAN OF ROI OF ROI: " << mean(grayImage) << endl;

	emptyMean = mean(grayImage);

	divideROIWithSample();
}

void PLApplication::mouseClick(int event, int x, int y, int flags, void*)
{
	switch (event)
	{
	case EVENT_LBUTTONDOWN:
		if (rectState == NOT_SET)
		{
			rectState = IN_PROCESS;
			rect = Rect(x, y, 1, 1);
		}
		else if (rectState == SET)
		{
			if (sampleState == NOT_SET)
			{
				sampleState = IN_PROCESS;
				sampleRect = Rect(x, y, 1, 1);
			}
		}
		break;
	case EVENT_LBUTTONUP:
		if (rectState == IN_PROCESS)
		{
			rect = Rect(Point(rect.x, rect.y), Point(x, y));
			rectState = SET;
			setRectInMask();
			showImage();
		}
		else if (rectState == SET)
		{
			if (sampleState == IN_PROCESS)
			{
				sampleRect = Rect(Point(sampleRect.x, sampleRect.y), Point(x, y));
				sampleState = SET;
				setSampleRectInMask();
				showROI();
			}
		}
		break;
	case EVENT_MOUSEMOVE:
		if (rectState == IN_PROCESS)
		{
			rect = Rect(Point(rect.x, rect.y), Point(x, y));
			showImage();
		}
		else if (rectState == SET)
		{
			if (sampleState == IN_PROCESS){
				sampleRect = Rect(Point(sampleRect.x, sampleRect.y), Point(x, y));
				showROI();
			}
		}
		break;
	}
}

void PLApplication::divideROIWithSample()
{
	vector<Mat> grid;
	Rect rect_mat(0, 0, roi.cols, roi.rows);

	for (int y = 0; y < roi.rows; y += sampleRect.height)
	{
		for (int x = 0; x < roi.cols; x += sampleRect.width)
		{
			Rect rect = Rect(x, y, sampleRect.width, sampleRect.height);
			if ((rect & rect_mat) == rect){
				rectangle(roi, Point(x, y), Point(x + rect.width, y + rect.height), GREEN, 2);
				grid.push_back(Mat(gray_roi, rect));
			}
		}
	}

	countCars(grid, emptyMean);
}

void PLApplication::countCars(vector<Mat> spaces, Scalar em)
{
	for each(Mat m in spaces)
	{
		Scalar spaceMean = mean(m);
		Scalar subMean = spaceMean - em;
		cout << "MEAN: " << subMean << endl;
	}
}
*/