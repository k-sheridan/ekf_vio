^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package invio
^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Update package.xml
* Update README.md
* very good performance with simulated bottom facing dataset. need to be less strict with keyframe count
* works well with sim bag file
* added simulated odometry launch
* fixed inverse where it should not have been. performs as expected now
* tuning. getting more robust results and plotted depth with rainbow
* attempting to visualize the depth of pixels
* integrated optimized points into moba
* successful converge
* the performance should be good as long as points converge. need more features though
* added mature init and kill box (untested)
* added simplest method for depth init
* tweaking the high level program flow
* changed name
* Update package.xml
* Update CMakeLists.txt
* change package name
* Update README.md
* Merge pull request `#5 <https://github.com/pauvsi/invio/issues/5>`_ from pauvsi/add-license-1
  Create LICENSE.md
* Create LICENSE.md
* Update README.md
* Create README.md
* i need to start odometry with enough features and assume them to be mature immediately after initilization. this won't allow newly added features to affect the pose estimate
* needs more tweaking
* found bug which should'nt be occuring when optimizing pose.
* publishing odometry
* added robot description
* got base to camera transform
* fixed initial pos problem
* bug during moba
* actually implemented moba
* added motion only bundle adjstment
* fixed feature clumping
* need to fix the duplicate feature problem i think I know why it is happening
* added insight publisher
* seemed to have fixed memory bug, but i aslo may have slowed the program by using lists
* switching to stl lists so that pointers are not invalidated
* I will write a function which checks that a feature's point's first observation points to the correct memory location of the feature
* can't write feature inside safe point delete
* found special case memory bug unaccounted for with assertions
* flowing features but issue exists in the observation deque
* fixed feature extraction bug
* started testing. its good i added a lot of assertions
* starting to add topics like camera image then imu then more
* fully compiled
* got frame to compile
* got feature to compile
* got point to compile
* fixed sophus include again now need to fix many bugs and cleanup the frame buffer safely so that there are no memory issues
* added xyz uv reproj jacobian for gauss newton
* added new feature
* computed point position. later i will initialize with point cloud
* setting new point's depth with avg scene depth
* made depth avg most efficient
* computed avg feature depth nearly as efficient as possible
* added efficient pose estimate setting
* flowed features and set pointers
* rewriting vio moving over previous work
* updating state
* transform SE3 back to the base position
* added a custom motion only bundle adjustment method
* added sophus data type
* tinkering with g2o
* its pretty fast except when the structure is optimized
* made it easy to compile out all the debug text. speeds up program drastically
* Merge branch 'master' into fixing_crashes
* added back motion only BA
* per point SBA was a success
* I think that i removed all dangling pointers
* removed most dangling pointers
* added the per point SBA algorithm untested
* added depth initialization
* fixed the draw function
* still tweaking keyframe update
* adding keyframe update method
* hacky fix for deeeeeeply rooted null pointer bug. May cause issues in future
* added safe delete structure
* fixed the g2o include issue
* set the g2o root
* making it official
* weird crashing bug memory?
* fixed bundle adjustment
* deleting points on the fly
* Merge branch 'master' of https://github.com/pauvsi/pauvsi_vio
* something is slowing the program down
* Create README.md
* set up the structure only BA
* got transformed states
* got g2o baseline set up
* added sba
* fixed the g2o include errors
* Fixed random bug
* New Sort
* found g2o a nonlinear optimization library
* fixed the pointer issue fully
* used hacky fix to fix the pointer issue
* asserting that the frame pointers are still valid
* really deep issue with the pointers
* current features seem good but old features are still in question
* compiles however there may be many pointer based issues
* nearly fixed compilation issues
* cleaned but bugs persist
* cleaning up but found issue
* updating using 2d 2d motion estimation
* fixed hsv to bgr conv
* added experiments
* have the old triangulate still
* Merge branch 'master' of https://github.com/pauvsi/pauvsi_vio
* added triangulation methods
* updating depths of features
* added a depth filter to each 2s feature
* epilines are sometimes ok
* sorted the active features by variance for easy selection
* fixed the tf tree for the new camera rig
* added gauss newton optimization
* feature kf
* Merge branch 'master' of https://github.com/pauvsi/pauvsi_vio
* added a draw feature
* added a vo_fast launch file
* found keyframes
* finding 4 keyframes
* commit before changing to keyframe based motion estimation
* debugging pnp issue
* have gotten reprojection error down to 0.0001
* tweaking traigulation
* triangulation successful
* triangulating with fundam mat
* added an ineffiecient frame buffer search but the error has become nearly zero with the triangulation
* removed the current and last frame and simply kept the frame buffer
* first version working
* triangulation successful ish
* fixing triangulation and increasing the camera travel
* used custom linear least squares triangulation method. have initializing step
* publishing 3d features
* nearly done publishing 3d points
* added conversion from state to rt matrix
* removed factor of 2 in the delta quaternion calculation
* fixed deep bug in state transition
* fixing transition
* converting angular velocity to radians with ros parameters
* fixed delta quaternion bug
* debugging overflow error
* propagating error with each prediction
* implemented the state transition jacobian function
* Merge branch 'master' of https://github.com/pauvsi/pauvsi_vio
* missing substitution 36 from jacobian
* finished the jacobean and state transition generation
* normalized the final quat
* fixed case where omega values are zero
* added state transition test.
* simplified and optimized jacobean
* Added matlab code which generates the state prediction and jacobean generation code
* fixed state error
* implemented 1d kalman filter for each 3d feature
* started publishing active features
* finished state transition function. need to propagate error.
* started state transition
* Added pauvsi_trajectory
* begin state transition function
* fixed the recalibration
* visual measurment fix
* Fixed Acceleration and gyro correction
* set up template for the EKF
* overloaded the << operator for the state.
* upgraded to tf2 and fixed eigen issue
* added the state vector class
* added ekf
* moved inertial motion estimator to ekf
* state transition function works only for certain parameters
* created experimental matlab code
* Just comments
* Fixed recalibration to calculate weighted sum for acceleration and gyroBias seperately.
* Incorrect weighted average
* ego
* begun ego motion estimate
* Recalibration
* added a compute error function for ego motion estimates. fixed the ego motion estimator.
* now added it oops
* added the feature tracker
* fixed all bugs in inertial algorithm
* angle now transformed correctly
* modified input args
* fixed the time issues with the motion estimator.
* there is a problem with the time that the image arrives, but it runs
* starting to remove gravity
* centripetal acceleration was calculated
* transformations are working as expected.
* Partial Commit
* imu thing
* added a big chunk:
* started the inertial motion estimator.
* added a parallel transform listener.
* simplified the main function. added callbacks to the VIO class
* reading IMU messages.
* added a remove by feature redundancy function. The algorithm now tries to keep n points.
* added an obnoxious amount of stuff:
  new calibration for scaled image
  new feature in driver to trasmit new calibration for scaled image
  improved the rank function to run faster, and overloaded it
  now using chierlality check to recover pose from essential matrix
  added number of features parameter.
  and more!
* VIOLine added
* Added transforms between IMU - base - Camera
* Data collection and added serial to vo.launch
* Merge remote-tracking branch 'origin/master'
* Added comments to ranking
* the motion estimation appears to work but the calibration is off right now.
* Merge remote-tracking branch 'origin/master'
* Added Ranking Features Function
* estimating motion. unit vectors only
* added a parameter for kill by similarity feature.
* Merge remote-tracking branch 'origin/master'
* Get response value
* fixed compare function
* Merge remote-tracking branch 'origin/master'
* still debuging
* changed launch file
* debug stuff
* Merge remote-tracking branch 'origin/master'
* Killed dissimilar features from optical flow
* Fixed Compare Descriptor
* function that describes a feature vector
* Getting feature similarity threshold
* Added compare descriptors.
* added stuff
* added a camera subscriber to get K and D matrices from driver
* current run time is 5ms per frame
* detected and flowed features are now first killed by their radius.
* mmorpg and description compare function.
* debug output for lost features. gotta take a test...
* optical flow is running. It can track 100 features for over 2000 frames after only running fast once!
* now each 2d feature gets a unique id untill the id's overflow back to zero. the overflow is controlled.
* added the VIOLine object. This will store a projected line on which a 2d feature lies.
* added the 3d features active and inactive
* coved where the computation/analysis of images is done
* restructuring vio algorithm
* added the vio feature classes
* features are now matched. motion estimation will come next
* describing and detecting features in 5 milliseconds.
* tinkering with feature descriptors
* now detecting FAST features and displaying
* added the flycap library. added vo.launch
* added pauvsi_vio files from original package and fixed cmakelists.txt to compile vio libs
* Restructuring layout of pauvsi_m7. Each section will now get its own package. I have also started rewriting the driver and calibrator.
* Contributors: Kevin Sheridan, Trexter, isakamot, kbravo, killerbee4992
