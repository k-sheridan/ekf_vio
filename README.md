# invio
## Indirect Visual Inertial Odometry algorithm.
Developed by Kevin Sheridan(@Trexter), Purdue University.

## Purpose and Intended Use
invio was developed for ROS and uses a few ROS tools like tf and the rosconsole. In addition to standard ROS libraries, this algorithm extensively uses OpenCV 3.0/2.0, Sophus and Eigen.

This algorithm is intended to be used with a downward facing, **high framerate (>30fps)**, **high horizontal field of view (>90 degrees)**, **global shutter** camera, but it should still work with most other configurations. It is important that the camera is facing a textured well lit planar surface when initializing.

An IMU is not required, but an IMU is highly reccomended. The IMU is used to predict how the camera traveled between frames. In the future I would like to use the IMU to estimate the metric scale of our initial motion so invio can be initialized from any situation if a velocity estimate is initially given to it.

## Performance
to be evaluated.

## Quick ROS installation guide

1. >cd ~/catkin_ws/src
2. >git clone https://github.com/pauvsi/invio
3. >sudo apt-get install ros-{your distribution}-sophus
4. >cd ..
5. >catkin_make

this should compile the entire package

## Development Status

- [x] initialize with guessed uniform depth estimate of all features and estimate camera motion
- [ ] optimize depths of new points when they are added
- [ ] integrate imu readings
- [ ] add realtime reinitialization feature
- [ ] add more initialization methods

## Closed Source Version

if you would like a closed source version with more sensor integration please contact Kevin Sheridan @Trexter.
