#include "detection.h"

Detection::Detection(int keypointSizeThreshold) {
	this->keypointSizeThreshold = keypointSizeThreshold;

	// create Background Subtractor objects
	//pMOG2 = bgsegm::createBackgroundSubtractorMOG();
	pMOG2 = createBackgroundSubtractorMOG2();

	// init blob detector
	// Setup SimpleBlobDetector parameters.
	SimpleBlobDetector::Params params;

	// Fitler white blobs
	params.filterByColor = 1;
	params.blobColor = 255;

	// Filter by Area
	params.filterByArea = true;
	params.minArea = 10;

	detector = SimpleBlobDetector::create(params);
}

/**
* Detect ball in camera frame and write center position to center.
**/
void Detection::detectionStep(Mat& frame, Point2f& center) {
	center.x = 0;
	center.y = 0;

	// smooth it
	GaussianBlur(frame, frame, Size(5, 5), 2, 2 );

	//update the background model
	pMOG2->apply(frame, fgMaskMOG2);

	// smooth it
	GaussianBlur( fgMaskMOG2, fgMaskMOG2, Size(5, 5), 2, 2 );

	// Detect blobs in fgMaskMOG2
	std::vector<KeyPoint> keypoints;
	detector->detect(fgMaskMOG2, keypoints);

	float m_size = 0;
	int biggest_kp = -1;
	int countOfKPsBiggerThanThreshold = 0;
	for (int i = 0; i < keypoints.size(); i++) {
		// Only work with keypoints of certain size (remove noise)
		if (keypoints[i].size > keypointSizeThreshold) {
			countOfKPsBiggerThanThreshold++;
			if (m_size < keypoints[i].size) {
				biggest_kp = i;
				m_size = keypoints[i].size;
			}
		}
	}
	if (biggest_kp > -1) {
		int size_int = (int)keypoints[biggest_kp].size;
		circle(frame, keypoints[biggest_kp].pt ,size_int, Scalar(0,0,255), 3, 8, 0);
		center.x = keypoints[biggest_kp].pt.x;
		center.y = keypoints[biggest_kp].pt.y;
	}

	putText(frame, "KPs : " + SSTR(int(countOfKPsBiggerThanThreshold)), Point(100,50), FONT_HERSHEY_SIMPLEX, 0.75, Scalar(50,170,50), 2);
}
