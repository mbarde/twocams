#include "opencv2/opencv.hpp"
#include "detection.h"

using namespace cv;
using namespace std;

// Calculates corner positions of calibration board depending on the squareSize
static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners) {
    corners.clear();
    for( int i = 0; i < boardSize.height; ++i )
        for( int j = 0; j < boardSize.width; ++j )
            corners.push_back(Point3f(j*squareSize, i*squareSize, 0));
}

// Create projection matrix from extrinsic and intrinsic parameters of a camera
void createProjectionMatrix(Mat tvec, Mat rvec, Mat cameraMatrix, Mat& proj) {
  Mat rmat;
  Rodrigues(rvec, rmat); // create rotation matrix from rotation vector
  Mat Rt;
  hconcat(rmat, Mat(tvec), Rt); // concat rotation matrix and translation vector P = [R|t]
  proj = cameraMatrix * Rt;
}


// Does triangulation with one 2D point for each camera
Point3f doTriangulation(Mat proj0, Mat proj1, Point2f pt0, Point2f pt1) {
  vector<Point2f> points0, points1;
  points0.push_back(pt0);
  points1.push_back(pt1);

  Mat Thomogeneous;
  triangulatePoints(proj0, proj1, points0, points1, Thomogeneous);

  // We need a 4-channel matrix for convertPointsFromHomogeneous
  Mat points4d = Thomogeneous.reshape(4);

  Mat points3d;
  convertPointsFromHomogeneous(points4d, points3d);
  points3d = points3d/10;

  Point3f result(points3d.at<float>(0), points3d.at<float>(1), points3d.at<float>(2));
  return result;
}

// Markers can be set by mouse click for triangulation
Point2f cam0marker;
Point2f cam1marker;

// Mouse click callback to set marker for cam0
void mouseCam0Handler(int event, int x, int y, int flags, void* param)
{
    switch(event){
    case CV_EVENT_LBUTTONDOWN:
        cam0marker = Point2i(x,y);
        break;
    }
}

// Mouse click callback to set marker for cam1
void mouseCam1Handler(int event, int x, int y, int flags, void* param)
{
    switch(event){
    case CV_EVENT_LBUTTONDOWN:
        cam1marker = Point2i(x,y);
        break;
    }
}

// Runs extrinsic calibration (and creates projectionMatrix) for a camera based on:
// -> Calibration board parameters and corresponding objectPoints
// -> Frame of camera
bool doExtrinsicCalibration(	Size boardSize, int chessBoardFlags, vector<Point3f> objectPoints,
										Mat frame, Mat cameraMatrix, Mat distCoeffs,
										Mat& projectionMatrix) {
	vector<Point2f> foundBoardCorners;
	bool found = findChessboardCorners(frame, boardSize, foundBoardCorners, chessBoardFlags);

	if (found) {
	  	drawChessboardCorners( frame, boardSize, Mat(foundBoardCorners), found );

	  	Mat rvec, tvec;
	  	solvePnP(Mat(objectPoints), Mat(foundBoardCorners), cameraMatrix,
						distCoeffs, rvec, tvec, false);

	  	createProjectionMatrix(tvec, rvec, cameraMatrix, projectionMatrix);
  		return true;
  	}
  	return false;
}

int main(int, char**) {
	// Init camera input streams
	VideoCapture cap0(1);
	if(!cap0.isOpened())
  		return -1;

	VideoCapture cap1(2);
	if(!cap1.isOpened())
		return -1;

	// Init windows
	namedWindow("cam0", 1);
	namedWindow("cam1", 1);

	// Init mouse callbacks
	int mouseParam= CV_EVENT_FLAG_LBUTTON;
	setMouseCallback("cam0",mouseCam0Handler,&mouseParam);
	setMouseCallback("cam1",mouseCam1Handler,&mouseParam);

	// Init BallDetector for automatic goal detection mode
	BallDetector detector(100);
	bool goalDetectionMode = false;
	Point3f lastBallPosition(0, 0, -100);

	// Load camera 0 parameters and compute remap
	FileStorage fs("camera_wired.xml", FileStorage::READ);
	Mat cameraMatrix0, distCoeffs0;
	float img_width, img_height;
	fs["camera_matrix"] >> cameraMatrix0;
	fs["distortion_coefficients"] >> distCoeffs0;
	fs["image_width"] >> img_width;
	fs["image_height"] >> img_height;
	Size imageSize0(img_width, img_height);
	Mat cam0map1, cam0map2;
	initUndistortRectifyMap(
              cameraMatrix0, distCoeffs0, Mat(),
              getOptimalNewCameraMatrix(cameraMatrix0, distCoeffs0, imageSize0, 1, imageSize0, 0), imageSize0,
              CV_16SC2, cam0map1, cam0map2);

	// Load camera 1 parameters and compute remap
	fs = FileStorage("camera_wired.xml", FileStorage::READ);
	Mat cameraMatrix1, distCoeffs1;
	fs["camera_matrix"] >> cameraMatrix1;
	fs["distortion_coefficients"] >> distCoeffs1;
	fs["image_width"] >> img_width;
	fs["image_height"] >> img_height;
	Size imageSize1(img_width, img_height);
	Mat cam1map1, cam1map2;
	initUndistortRectifyMap(
	        cameraMatrix1, distCoeffs1, Mat(),
	        getOptimalNewCameraMatrix(cameraMatrix1, distCoeffs1, imageSize1, 1, imageSize1, 0), imageSize1,
	        CV_16SC2, cam1map1, cam1map2);

	// Calibration board size configuration
	Size boardSize(9, 6); // size in squares
	float squareSize = 25; // size in mm
	int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;

	// Object points are corner positions of calibration board
	vector<Point3f> objectPoints;
	calcBoardCornerPositions(boardSize, squareSize, objectPoints);

	Mat frame0, frame1; // current image of cam0 and cam1
	Mat proj0, proj1;	// projection matrices for cam0 and cam1

	bool calibrated0 = false;
	bool calibrated1 = false;
	bool start = false;

 	cout << "Press SPACE to start" << endl;

	for(;;) {
		// Capture camera frames and remap regarding intrinsic parameters
      cap0 >> frame0;
   	cap1 >> frame1;

		remap(frame0, frame0, cam0map1, cam0map2, INTER_LINEAR);
		remap(frame1, frame1, cam1map1, cam1map2, INTER_LINEAR);

      if (start && !calibrated0) {
			if ( doExtrinsicCalibration(boardSize, chessBoardFlags, objectPoints, frame0, cameraMatrix0, distCoeffs0, proj0) ) {
				calibrated0 = true;
         	cout << "Got cam0 projection matrix!" << endl;
			}
      }

      if (start && !calibrated1) {
			if ( doExtrinsicCalibration(boardSize, chessBoardFlags, objectPoints, frame1, cameraMatrix1, distCoeffs1, proj1) ) {
				calibrated1 = true;
         	cout << "Got cam1 projection matrix!" << endl;
			}
      }

      if (start && (!calibrated0 || !calibrated1)) {
        cout << "Chessboard has not been detected by both cameras! Press SPACE to try again!" << endl;
      }

      if (start && calibrated0 && calibrated1) {
        	cout << "Extrinsic calibration done!" << endl;
       	cout << "Click with your mouse into the images to set markers. Then press SPACE to triangulate." << endl;
			cout << "Press SPACE to start goal detection." << endl;
      }

      start = false;

		if (goalDetectionMode) {
			// Set markers to ball position
			detector.detectionStep(frame0, cam0marker);
			detector.detectionStep(frame1, cam1marker);

			if (cam0marker.x != 0 && cam1marker.x != 0) {
				// Do triangulation to get ball positon
				Point3f newBallPosition = doTriangulation(proj0, proj1, cam0marker, cam1marker);

				// If ball is behind z-Coordinate of goal line:
				// -> its a goal, if x and y coordinates are inside goal measurements
				// -> its a miss, else
				if (newBallPosition.z > -10) {
					if (newBallPosition.x > -5 && newBallPosition.x < 25
						&& newBallPosition.y > -7 && newBallPosition.y < 15) {
							cout << "GOAAAAAAL: " << newBallPosition.x << " : " << newBallPosition.y << endl;
					} else {
						cout << "MISS: " << newBallPosition.x << " : " << newBallPosition.y << endl;
					}
					goalDetectionMode = false;
				} else {
					lastBallPosition = newBallPosition;
				}
			}
		}

      // Draw markers
      if (cam0marker.x != 0) {
      	circle( frame0, cam0marker, 3, Scalar(0,255,0), -1, 8, 0 );
      }
      if (cam1marker.x != 0) {
        	circle( frame1, cam1marker, 3, Scalar(0,255,0), -1, 8, 0 );
      }

		// Show frames
		if (!frame0.empty()) {
      	imshow("cam0", frame0);
		}
		if (!frame1.empty()) {
      	imshow("cam1", frame1);
		}

		// Keyboard input
      char key = (char)waitKey(30);
		// Pressing space ...
      if( key  == 32 ) {
			// ... toggles calibration, if not done yet
        	if (!start && (!calibrated0 || !calibrated1)) {
          	start = true;
          	cout << "Try detecting" << endl;
        	}
		  	// ... toggles triangulation if markers are set (by mouse) and if calibration is done
        	if (calibrated0 && calibrated1) {
          	if (cam0marker.x != 0 && cam1marker.x != 0) {
				  	cout << "Start triangulation" << endl;
	            Point3f point = doTriangulation(proj0, proj1, cam0marker, cam1marker);
					cout << point << endl;
          	}
        	}
	  	} else if (key == 'g') {
			// Pressing 'G' starts goal-detection
		  	if (!goalDetectionMode && calibrated0 && calibrated1) {
			  	goalDetectionMode = true;
			  	lastBallPosition.z = -100;
			  	cout << "Goal detection mode started" << endl;
		  	}
	  	} else if (key > 0) {
			cout << key << endl;
  			break;
      }

	} // Outer for loop

 	return 0;
} // Main function
