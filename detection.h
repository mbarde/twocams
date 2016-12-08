#ifndef _DETECTION_H_
#define _DETECTION_H_

#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

using namespace cv;
using namespace std;

/**
* Detect ball in camera frame and write center position to center.
**/
void detectBall(Mat frame, Point2f& center, int mode);

#endif
