#include "detection.h"

BallDetector::BallDetector(int keypointNoiseThreshold) {
	// Create Background Subtractor object
	pMOG2 = createBackgroundSubtractorMOG2();

	// Init blob detector with some parameters
	SimpleBlobDetector::Params params;

	// Filter white blobs
	params.filterByColor = 1;
	params.blobColor = 255;

	// Filter by Area
	params.filterByArea = true;
	params.minArea = keypointNoiseThreshold;

	detector = SimpleBlobDetector::create(params);
}

// Detect ball in camera frame and write position to corresponding argument
void BallDetector::detectionStep(Mat& frame, Point2f& position) {
	// By default return x,y = 0 if ball is not detected
	position.x = 0;
	position.y = 0;

	Mat hFrame;

	// Smooth image (to avoid noise)
	GaussianBlur(frame, hFrame, Size(5, 5), 2, 2 );

	// Update the background model and to background subtraction
	// Resulting mask will be written to fgMaskMOG2
	pMOG2->apply(hFrame, fgMaskMOG2);

	// Smooth mask (to avoid noise)
	GaussianBlur(fgMaskMOG2, fgMaskMOG2, Size(5, 5), 2, 2);

	// Detect blobs (regions) in fgMaskMOG2
	std::vector<KeyPoint> keypoints;
	detector->detect(fgMaskMOG2, keypoints);

	// TODO: Find out which keypoint actually corresponds to the ball
	// Ideally we only have one keypoint and there is nothing to do here
	int kpBallId = 0;
	if (keypoints.size() < 1) { kpBallId = -1; }

	if (kpBallId > -1) {
		// Mark keypoint in image and write position to corresponding argument
		int size_int = (int)keypoints[kpBallId].size;
		circle(frame, keypoints[kpBallId].pt ,size_int, Scalar(0,0,255), 3, 8, 0);
		position.x = keypoints[kpBallId].pt.x;
		position.y = keypoints[kpBallId].pt.y;
	}

	putText(frame, "KPs : " + SSTR(int(keypoints.size())), Point(100,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);
}
