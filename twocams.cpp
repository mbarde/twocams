#include "opencv2/opencv.hpp"
#include "detection.h"

using namespace cv;
using namespace std;

static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners)
{
    corners.clear();

    for( int i = 0; i < boardSize.height; ++i )
        for( int j = 0; j < boardSize.width; ++j )
            corners.push_back(Point3f(j*squareSize, i*squareSize, 0));
}

void createProjectionMatrix(Mat tvec, Mat rvec, Mat cameraMatrix, Mat& proj) {
  Mat rmat;
  Rodrigues(rvec, rmat); // create rotation matrix from rotation vector
  Mat Rt;
  hconcat(rmat, Mat(tvec), Rt); // concat rotation matrix and translation vector P = [R|t]
  proj = cameraMatrix * Rt;
}

/**
* Does triangulation with one 2D point for each camera
**/
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

Point2f cam0point;
Point2f cam1point;

void mouseCam0Handler(int event, int x, int y, int flags, void* param)
{
    switch(event){
    case CV_EVENT_LBUTTONDOWN:
        cam0point = Point2i(x,y);
        break;
    }
}
void mouseCam1Handler(int event, int x, int y, int flags, void* param)
{
    switch(event){
    case CV_EVENT_LBUTTONDOWN:
        cam1point = Point2i(x,y);
        break;
    }
}

int main(int, char**)
{
    VideoCapture cap0(1);
    if(!cap0.isOpened())
        return -1;

    VideoCapture cap1(2);
    if(!cap1.isOpened())
          return -1;

    Mat edges;
    namedWindow("cam0", 1);
    namedWindow("cam1", 1);

    int mouseParam= CV_EVENT_FLAG_LBUTTON;
    setMouseCallback("cam0",mouseCam0Handler,&mouseParam);
    setMouseCallback("cam1",mouseCam1Handler,&mouseParam);

	 Detection detection(20);

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

    Mat frame0, frame1, proj0, proj1;
    Size boardSize(9, 6);
    float squareSize = 25;

    vector<Point3f> objectPoints;
    calcBoardCornerPositions(boardSize, squareSize, objectPoints);

    int chessBoardFlags = CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE;

    bool calibrated0 = false;
    bool calibrated1 = false;
    bool start = false;

    cout << "Press SPACE to start" << endl;

    for(;;) {
      cap0 >> frame0;
      cap1 >> frame1;

      if (start && !calibrated0) {
        vector<Point2f> foundBoardCorners;
        bool found = findChessboardCorners( frame0, boardSize, foundBoardCorners, chessBoardFlags);

        if (found) {
          drawChessboardCorners( frame0, boardSize, Mat(foundBoardCorners), found );

          Mat rvec, tvec;
          solvePnP(Mat(objectPoints), Mat(foundBoardCorners), cameraMatrix0,
                       distCoeffs0, rvec, tvec, false);

          cout << norm(tvec) << endl;
          createProjectionMatrix(tvec, rvec, cameraMatrix0, proj0);
          calibrated0 = true;

          cout << "Got cam0 projection matrix!" << endl;
        }
      }

      if (start && !calibrated1) {
        vector<Point2f> foundBoardCorners;
        bool found = findChessboardCorners( frame1, boardSize, foundBoardCorners, chessBoardFlags);

        if (found) {
          drawChessboardCorners( frame1, boardSize, Mat(foundBoardCorners), found );

          Mat rvec, tvec;
          solvePnP(Mat(objectPoints), Mat(foundBoardCorners), cameraMatrix1,
                       distCoeffs1, rvec, tvec, false);

          cout << norm(tvec) << endl;
          createProjectionMatrix(tvec, rvec, cameraMatrix1, proj1);
          calibrated1 = true;
          cout << "Got cam1 projection matrix!" << endl;
        }

        if(waitKey(30) >= 0) break;
      }

      if (start && (!calibrated0 || !calibrated1)) {
        cout << "Chessboard has not been detected by both cameras! Press SPACE to try again!" << endl;
      }
      if (start && calibrated0 && calibrated1) {
        cout << "READY!" << endl;
      }
      start = false;

		if (goalDetectionMode) {
			detection.detectionStep(frame0, cam0point);
			detection.detectionStep(frame1, cam1point);

			if (cam0point.x != 0 && cam1point.x != 0) {
				Point3f newBallPosition = doTriangulation(proj0, proj1, cam0point, cam1point);

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

      // draw markers
      if (cam0point.x != 0) {
        circle( frame0, cam0point, 3, Scalar(0,255,0), -1, 8, 0 );
      }
      if (cam1point.x != 0) {
        circle( frame1, cam1point, 3, Scalar(0,255,0), -1, 8, 0 );
      }

      //Mat frame0mapped, frame1mapped;
      //remap(frame0, frame0mapped, cam0map1, cam0map2, INTER_LINEAR);
      //remap(frame1, frame1mapped, cam1map1, cam1map2, INTER_LINEAR);
		if (!frame0.empty()) {
      	imshow("cam0", frame0);
		}

		if (!frame1.empty()) {
      	imshow("cam1", frame1);
		}

      char key = (char)waitKey(30);
      if( key  == 32 ) {
        if (!start && (!calibrated0 || !calibrated1)) {
          	start = true;
          	cout << "Try detecting" << endl;
        }
        	if (calibrated0 && calibrated1) {
          	if (cam0point.x != 0 && cam1point.x != 0) {
				  	cout << "Start triangulation" << endl;
	            Point3f point = doTriangulation(proj0, proj1, cam0point, cam1point);
					cout << point << endl;
          	}
        	}
	  	} else if (key == 'g') {
		  	if (!goalDetectionMode && calibrated0 && calibrated1) {
			  goalDetectionMode = true;
			  lastBallPosition.z = -100;
			  cout << "Goal detection mode started" << endl;
		  	}
	  	} else if (key > 0) {
			cout << key << endl;
     		break;
      }

    } // outer for loop

    return 0;
}
