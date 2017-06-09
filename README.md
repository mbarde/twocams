# twocams
Using OpenCV and two cameras to detect 3D position of object.

Requirements: OpenCV (http://opencv.org/)

Build & Run:
```
mkdir build
cd build
cmake .. && make
./twocams
```

Required configurations files:

* ```config.xml```: Define camera indexes, calibration files and parameters for automatic goal detection (like goal size and offsets)
* Intrinsic calibration files: One file per camera required, can be created using the program provided by this OpenCV-Tutorial: http://docs.opencv.org/master/d4/d94/tutorial_camera_calibration.html

How to use:

1. Start program
2. Place chessboard as your "goal" (both cameras must see it completely)
3. Press SPACE
4. If chessboard has not been detected properly there will be an error message and you have to try it a again (go to 2.)
5. Extrinsic calibration has been sucessfully done, projection matrices of cameras have been calculated

Now you have several options:

* Mark a position in both views (by clickin in them)
* Press SPACE to get 3D position of this point

OR

* Press 'g' to start goal detection mode
* Now the ball will be detected automatically and marked in the views by a red circle
* When it crosses the Z-coordinate of the goal line (see ```config.xml```) a goal or a miss will be detected (depending of the X and Y position of the ball)

OR

* Press 't' to start deviation test
* Chessboard corners will be detected automatically, its 3D points will be calculated and compared to the actual positions of this edges
* Output is the deviation in cm for each point and in average
