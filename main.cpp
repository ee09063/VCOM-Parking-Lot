#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "PLApplication.h"

using namespace std;
using namespace cv;

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

	const string winName = "image";
	namedWindow(winName, WINDOW_AUTOSIZE);
	setMouseCallback(winName, on_mouse, 0);

	plapp.setImageAndWinName(image, winName);
	plapp.showImage();

	waitKey(0);
	return 0;
}

