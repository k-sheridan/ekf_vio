# invio
## Indirect Visual Inertial Odometry algorithm.
Developed by Kevin Sheridan, Purdue University.

## Purpose and Intended Use
invio was developed for ROS and uses a few ROS tools like tf and the rosconsole. In addition to standard ROS libraries, this algorithm extensively uses OpenCV 3.0/2.0, Sophus and Eigen.

This algorithm is intended to be used with a downward facing, **high framerate (>30fps)**, **high horizontal field of view (>90 degrees)**, **global shutter** camera, but it should still work with most other configurations. It is important that the camera is facing a textured well lit planar surface when initializing. I have been trying to get good results with forward facing results, and in some cases it is working well.

The best scenario for invio is the bottom facing camera scenario. In this scenario there is very little occlusions and a lot of parallax for depth estimation. The forward facing scenario is the most difficult one because there are many occlusions and sometimes very little parallax to determine the depth of pixels accurately. Currently the forward facing scenario is working with most but not all of my datasets.

An IMU is not required, but an IMU is highly reccomended. The IMU's body frame linear accelerations and angular velocities are fused into a large state vector bewteen frames essentially predicting the pose for the next frame.

## Performance

![Small Scale Results](/images/invio1.png)

I am currently hevily working on this algorithm at the moment.

The speed of the program per frame on a laptop (2015 Macbook Pro in my case) 
### runtimes per frame:
Feature Extraction: 0.1-0.3ms (I will attempt to decrese this later)

KLT Feature Tracking for 200 features: 0.8-3ms (decreases once IMU is fused)

Motion Estimation with 100-300 features: 0.03 - 0.2ms 

Depth Measurement and Update for 200 features: 0.5ms

###Total: 1.43 - 5.43ms 

I run invio on 1 thread only currently so that it consumes as little CPU as possible. For my application this algorithm must be running along side much hungrier programs.


## Quick ROS installation guide

(developed using ROS Kinetic)

1. >cd ~/catkin_ws/src
2. >git clone https://github.com/pauvsi/invio
3. >sudo apt-get install ros-{your distribution}-sophus
4. >cd ..
5. >catkin_make

this should compile the entire package. If there is an issue please create an issue on this repo!

## Development Status

- [x] initialize with guessed uniform depth estimate of all features and estimate camera motion
- [x] optimize depths of new points when they are added
- [x] publish odometry message
- [x] remove old frames from the buffer safely preventing the algorithm from using an extremely high amount of memory
- [ ] integrate imu readings
- [ ] determine fast and accurate params and increase speed
- [ ] add third party initialization through rosservice
- [ ] add realtime reinitialization feature
- [ ] add more initialization methods
