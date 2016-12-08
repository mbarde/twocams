#include "detection.h"

void detectBall(Mat frame, Point2f& center, int mode) {
    // By convention return x,y=0 if no ball is found
    center.x = 0;
    center.y = 0;

    Scalar hsv_min;
    Scalar hsv_max;
    if (mode == 0) {
      hsv_min = Scalar(75, 118, 131, 0);
      hsv_max = Scalar(85, 255, 188, 0);
    } else {
      hsv_min = Scalar(44, 102, 56, 0);
      hsv_max = Scalar(58, 164, 95, 0);
    }

    Mat hsv_frame, thresholded;

    // Covert color space to HSV as it is much easier to filter colors in the HSV color-space.
    cvtColor(frame, hsv_frame, CV_BGR2HSV);
    // Filter out colors which are out of range.
    inRange(hsv_frame, hsv_min, hsv_max, thresholded);

    // smooth it
    GaussianBlur( thresholded, thresholded, Size(9, 9), 2, 2 );

    Point min_loc, max_loc;
    double min, max;
    minMaxLoc(thresholded, &min, &max, &min_loc, &max_loc);
    center.x = max_loc.x;
    center.y = max_loc.y;
}
