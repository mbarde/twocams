#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

static void calcBoardCornerPositions(Size boardSize, float squareSize, vector<Point3f>& corners)
{
    corners.clear();

    for( int i = 0; i < boardSize.height; ++i )
        for( int j = 0; j < boardSize.width; ++j )
            corners.push_back(Point3f(j*squareSize, i*squareSize, 0));
}

void createProjectionMatrix(Mat tvec, Mat rvec, Mat& proj) {
  Mat rmat;
  Rodrigues(rvec, rmat); // create rotation matrix from rotation vector
  hconcat(rmat, Mat(tvec), proj); // concat rotation matrix and translation vector P = [R|t]
}

Point2i cam0point;
Point2i cam1point;

void mouseCam0Handler(int event, int x, int y, int flags, void* param)
{
    switch(event){
    case CV_EVENT_LBUTTONDOWN:
        cam0point = Point2i(x,y);
        cout << cam0point << endl;
        break;
    }
}
void mouseCam1Handler(int event, int x, int y, int flags, void* param)
{
    switch(event){
    case CV_EVENT_LBUTTONDOWN:
        cam1point = Point2i(x,y);
        cout << cam1point << endl;
        break;
    }
}


int main(int, char**)
{
    VideoCapture cap0(0);
    if(!cap0.isOpened())
        return -1;

    VideoCapture cap1(1);
    if(!cap1.isOpened())
          return -1;

    Mat edges;
    namedWindow("cam0", 1);
    namedWindow("cam1", 1);

    int mouseParam= CV_EVENT_FLAG_LBUTTON;
    setMouseCallback("cam0",mouseCam0Handler,&mouseParam);
    setMouseCallback("cam1",mouseCam1Handler,&mouseParam);

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

          // Load camera parameters
          FileStorage fs("camera_data_0.xml", FileStorage::READ);
          Mat cameraMatrix, distCoeffs;
          fs["camera_matrix"] >> cameraMatrix;
          fs["distortion_coefficients"] >> distCoeffs;

          Mat rvec, tvec;
          solvePnP(Mat(objectPoints), Mat(foundBoardCorners), cameraMatrix,
                       distCoeffs, rvec, tvec, false);

          cout << norm(tvec) << endl;
          createProjectionMatrix(tvec, rvec, proj0);
          calibrated0 = true;

          cout << "Got cam0 projection matrix!" << endl;
          cout << "Trying to get cam1 pos and rot" << endl;
        }
      }

      if (calibrated0 && !calibrated1) {
        vector<Point2f> foundBoardCorners;
        bool found = findChessboardCorners( frame1, boardSize, foundBoardCorners, chessBoardFlags);

        if (found) {
          drawChessboardCorners( frame1, boardSize, Mat(foundBoardCorners), found );

          // Load camera parameters
          FileStorage fs("camera_data_1.xml", FileStorage::READ);
          Mat cameraMatrix, distCoeffs;
          fs["camera_matrix"] >> cameraMatrix;
          fs["distortion_coefficients"] >> distCoeffs;

          Mat rvec, tvec;
          solvePnP(Mat(objectPoints), Mat(foundBoardCorners), cameraMatrix,
                       distCoeffs, rvec, tvec, false);

          cout << norm(tvec) << endl;
          createProjectionMatrix(tvec, rvec, proj1);
          calibrated1 = true;
          cout << "Got cam1 projection matrix!" << endl;
          break;
        }

        if(waitKey(30) >= 0) break;
      }

      // draw markers
      if (cam0point.x != 0) {
        circle( frame0, cam0point, 3, Scalar(0,255,0), -1, 8, 0 );
      }
      if (cam1point.x != 0) {
        circle( frame1, cam1point, 3, Scalar(0,255,0), -1, 8, 0 );
      }

      imshow("cam0", frame0);
      imshow("cam1", frame1);

      char key = (char)waitKey(30);
      if( key  == 32 ) {
        if (!start) start = true;
      } else if (key > 0) {
        break;
      }

    } // outer for loop

    return 0;
}
