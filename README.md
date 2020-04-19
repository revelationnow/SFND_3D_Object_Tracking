# SFND 3D Object Tracking

Welcome to the final project of the camera course. By completing all the lessons, you now have a solid understanding of keypoint detectors, descriptors, and methods to match them between successive images. Also, you know how to detect objects in an image using the YOLO deep-learning framework. And finally, you know how to associate regions in a camera image with Lidar points in 3D space. Let's take a look at our program schematic to see what we already have accomplished and what's still missing.

<img src="images/course_code_structure.png" width="779" height="414" />

In this final project, you will implement the missing parts in the schematic. To do this, you will complete four major tasks: 
1. First, you will develop a way to match 3D objects over time by using keypoint correspondences. 
2. Second, you will compute the TTC based on Lidar measurements. 
3. You will then proceed to do the same using the camera, which requires to first associate keypoint matches to regions of interest and then to compute the TTC based on those matches. 
4. And lastly, you will conduct various tests with the framework. Your goal is to identify the most suitable detector/descriptor combination for TTC estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor. In the last course of this Nanodegree, you will learn about the Kalman filter, which is a great way to combine the two independent TTC measurements into an improved version which is much more reliable than a single sensor alone can be. But before we think about such things, let us focus on your final project in the camera course. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* Git LFS
  * Weight files are handled using [LFS](https://git-lfs.github.com/)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.

# Rubric Details

## FP.1 : Match Bounding Boxes

The code is implemented in src/camFusion_Student.cpp
The logic used here is as follows:

1. Loop through all matches
.. For each of the two keypoints in a match, identify which bounding box it is from (one is from the previous frame and one is from the current frame)
.. Insert the pair of bounding box IDs for the match in a map stucture, where the key is the pair of bounding boxes and the value is the number of times this pair was seen in the matches

2. After having inserted all the bounding box pairs in the map structure, we find the pairs with the maximum occurrences to determine correspondances.
.. This is done in a local greedy mechanism rather than a global greedy mechanism
.. The map of bounding box pair frequencies is flipped into a multimap (which is initialized to store elements in descending key order) where the key is the frequency and the value is the pair of boxes
.. Then the multimap is iterated through to find the bounding box pairs with maximum correspondance. Every time a pair of bounding boxes are identified, that pair is no longer considered in the rest of the iterations

## FP.2 : Compute LIDAR Based TTC

The code is implemented in src/camFusion_Student.cpp

## FP.3 : Associate Keypoint correspondances with Bounding Boxes

The code is implemented in src/camFusion_Student.cpp

## FP.4 : Compute Camera-based TTC

The code is implemented in src/camFusion_Student.cpp

## FP.5 : Performance evaluation of LIDAR

![](lidar_ttc_error_001.png?raw=true)
![](lidar_ttc_error_002.png?raw=true)
![](lidar_ttc_error_003.png?raw=true)

Above are a few examples of a few poorly detected TTC values.
The first one shows an error due to some new points that are detected which were not present in the image just before that.
This leads to a shorter than expected TTC to be computed.

The second image shows an extremely large TTC computed, this happens because of the reverse case, where some points which were really close disappear making the new distance appear larger than it would have been otherwise.

The third image shows an abnormally small TTC. This happens as a result of a combination of the new closer points detected and also a limitation of the Constant Velocity model. Here we are not really filtering TTC values and taking instantaneous reads to estimate TTC. This makes the glitches in relative velocities of the Ego car and target car become a lot more prominent. Note that a Constant Acceleration model wouldn't solve this particular problem either.
Something along the lines of IIR filtering of the TTC values would help reduce the jitter in the values. Another mechanism could be to use some control theory guided estimation of acceleration and velocity of the target vehicle to get a more accurate estimate of time to collision.


## FP.6 : Performance evaluation of Camera

![](ttc_all.svg?raw=true)

The above graph compares all the different descriptor and detector combinations for the camera based TTC estimates. We see that in general other than Harris and ORB, the other detectors have a fairly tight bound on the TTC estimates across different image pairs. Focussing closer on Shi-Tomasi, SIFT and AKAZE detectors, we see that surprisingly Shi-Tomasi appears to have a somewhat tighter bound on interquartile range, however the one off outliers seem further away than in the other two cases. Between SIFT and AKAZE, SIFT with BRIEF or SIFT descriptors appears to be similar or better than AKAZE, but only slightly and this might change if tried on a different test set.
The image below shows the comparison between the 3 detectors mentioned.

![](detector_sift_shitomasi_akaze.png?raw=true)

Subsequently, comparing the SIFT descriptor across different detectors with AKAZE gives the below result. We see that SIFT with Shitomasi, FAST and SIFT appear to be superior to other detectors using SIFT and AKAZE appears tobe comparable to SIFT with FAST or SIFT.

![](descriptor_sift_akaze.png?raw=true)

