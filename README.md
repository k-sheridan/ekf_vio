# invio
## Indirect Visual Inertial Odometry algorithm.
Developed by Kevin Sheridan(@Trexter), Purdue University.

## Purpose and Intended Use
invio was developed for ROS and uses a few ROS tools like tf and the rosconsole. In addition to standard ROS libraries, this algorithm extensively uses OpenCV 3.0/2.0, Sophus and Eigen.

This algorithm is intended to be used with a downward facing, **high framerate (>30fps)**, **high horizontal field of view (>90 degrees)**, **global shutter** camera, but it should still work with most other configurations. It is important that the camera is facing a textured well lit planar surface when initializing.

The best scenario for invio is the bottom facing camera scenario. In this scenario there is very little occlusions and a lot of parallax for depth estimation. The forward facing scenario is the most difficult one because there are many occlusions and sometimes very little parallax to determine the depth of pixels accurately. Currently the forward facing scenario is working with most but not all of my datasets.

An IMU is not required, but an IMU is highly reccomended. The IMU is used during the prediction step to help predict how the camera has rotated between frames. In the future I would like to use the IMU to estimate the metric scale of our initial motion so invio can be initialized from any situation if a velocity estimate is initially given to it.

## Performance

![Small Scale Results](/images/invio1.png)

I am constantly improving the performance of the algorithm while decreasing the runtime. Most recently I added occlusion detection based on the range of depth estimates. I also now compute the variance of depth measurements based on both the measured depth and its parallax.

The speed of the program per frame on a laptop (2015 Macbook Pro in my case) 
### runtimes per frame:
Feature Extraction: 0.1-0.3ms

KLT Feature Tracking for 200 features: 0.8-3ms (will be decreased once IMU is fully fused)

Motion Estimation with 20 features: 1-3ms (currently for debugging purposes I am using all valid features ~150 at 30ms)

Depth Measurement and Update for 200 features: 5ms (in the future I will update only ~20 features per frame for speed improvement)

Total: 6.9-8.3ms 

I run invio on 1 thread only currently so that it consumes as little CPU as possible. For my application this algorithm must be running along side much hungrier programs.


## Quick ROS installation guide

1. >cd ~/catkin_ws/src
2. >git clone https://github.com/pauvsi/invio
3. >sudo apt-get install ros-{your distribution}-sophus
4. >cd ..
5. >catkin_make

this should compile the entire package

## Development Status

- [x] initialize with guessed uniform depth estimate of all features and estimate camera motion
- [x] optimize depths of new points when they are added
- [x] publish odometry message
- [ ] remove old frames from the buffer safely preventing the algorithm from using an extremely high amount of memory
- [ ] integrate imu readings
- [ ] determine fast and accurate params and increase speed
- [ ] add third party initialization through rosservice
- [ ] add realtime reinitialization feature
- [ ] add more initialization methods

## Closed Source Version

if you would like a closed source version with more sensor integration please contact Kevin Sheridan @Trexter.
