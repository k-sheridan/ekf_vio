# invio
## Indirect Visual Inertial Odometry algorithm.
Developed by Kevin Sheridan(@Trexter), Purdue University.

## Purpose and Intended Use
invio was developed for ROS and uses a few ROS tools like tf and the rosconsole. In addition to standard ROS libraries, this algorithm extensively uses OpenCV 3.0/2.0, Sophus and Eigen.

This algorithm is intended to be used with a downward facing, **high framerate (>30fps)**, **high horizontal field of view (>90 degrees)**, **global shutter** camera, but it should still work with most other configurations. It is important that the camera is facing a textured well lit planar surface when initializing.

An IMU is not required, but an IMU is highly reccomended. The IMU is used during the prediction step to help predict how the camera has rotated between frames. In the future I would like to use the IMU to estimate the metric scale of our initial motion so invio can be initialized from any situation if a velocity estimate is initially given to it.

## Performance

![Small Scale Results](/images/invio1.png)

from initial tests the algorithm works well when estimating feature depths and motion. Fast rotations currently cause a tracking loss from both lack of depth estimates and "slipping" of feature tracking due to occlusions. The feature slipping should be fixed by integrating linear and angular velocity measurements/estimates into the KLT feature tracker. 

The speed of the program per frame on a laptop (2015 Macbook Pro in my case) is very variant because I wrote the program to run using only one thread. When feature positions are ready to be optimized there is a significant slow down of about 3X depending on how many feature positions are being optimized at that time.

### runtimes per frame:
typical: 5-10ms 
peak: 30ms

I would like to keep Invio running on only one thread, so I plan to address the periodic slow downs by more evenly distributing the feature position optimizations through out the frames. I will investigate the extent to which my use of STL Lists is impacting the performance during a search/insert/delete operation.


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
