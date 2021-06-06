# **Advanced Lane Finding**

The goal of this project is to identify road lanes in a video stream of a forward facing camera mounted c on a moving vehicle.

We'll be using image manipulation techniques to extract enough information from each frame of the video and identify the lane lines, the radius of curvature and the distance from the camera to the center line of the road.

## **Project Structure**

  * **camera_calibration/** Directory with calibration images and camera parameters
  * **test_images/** Directory with test images
  * **videos/** Directory with input and output videos
  * **Advanced_lane_detector/** Directory with the project code ALF.ipynb
  * **Readme.md** Project Write up

## **Project Overview**

In order to detect the lane lines in a video stream we must accomplish the following:

* **Camera Calibration** - Calibrate the camera to correct for image distortions. For this we use a set of chessboard images, knowing the distance and angles between common features like corners, we can calculate the transformation functions and apply them to the video frames.

  * **Class Calibration** is used for achieving this goal.This class has the following member functions
    * **calibrate()** - To calibrate and return camera intrensic matrix and distortion parameters
    * **undistort()** - To undistort an input image using camera matrix and distortion parameters
    * **camsave()** - To save the camera matrix and distortion parameters as a pickle file so that that can be reused by loading the pickle file.


* **Image Transform** - We use a set of image manipulation techniques to accentuate certain features like lane lines. We use color space transformations, like from RGB to HLS, channel separation, like separating the S channel from the HLS image and image gradient to allow us to identify the desired lines.We apply a "bird’s-eye view transform" that let's us view a lane from above and thus identify the lane lines, measure its curvature and respective radius.

  * **Class Transformer** is used for this purpose.This class has the following member functions
    * **sobel_xy()** - To perform basic sobel operation on x and y to be used on magnitude and direction functions
    * **sobel_thresholded_abs()** - To perform sobel operator  on x direction and y direction separately with threshold.
    * **magnitude_threshold()** - To calculate the gradient magnitude
    * **direction_threshold()** - To calculate the direction of gradient based on the threshold. **gradient_threshold()** - to apply all the threshold together on an image
    * **color_transform()** - To transform to RGB to HLS and then to separate S channel.
    * **combine_image()** - To combine gradient and color transformed binary test_images
    * **birdseyeview()** - To apply perspective transform on an image
    * **run()** - Pipeline function for run the above functions


* **Lane Detection** -We then analyze the transformed image and try to detect the lane pixels. We use a series of windows and identify the lane lines by finding the peeks in a histogram of each window's.

  * **Class LaneFinder** is used for achieving this goal.This class has the following member functions

    * **histogram()** - To find the histogram of the images and return the two peaks where the potential lane lines can be found
    * **lane_pixel_finder()** - To return the x and y pixels that belongs to the left lane and right lane.This is achieved using sliding window approach.
    * **line_fit()** - To fit a line of second degree using the detected x ,y pixels of left lane and right lane.
    * **check_poly()** - To identify the margin that line can be found using previously fitted lines without using sliding window approach. This will increase the program speed.
    * **curvature()** - To calculate the radius of the curve
    * **fill_poly()** - To fill the identified lane with a color for visualization.
    * **vehicle_position()** - To calculate the vehicle position from the center
    * **run()** -Pipeline function for run the above functions


* **Loading the Video Stream** -  We then load the camera matrix images and videos.

  * **Class Loader** - is used for achieving this goal.This class has the following member functions

    * **load_Camera()** - To calibrate the camera if it is not calibrated before or load the camera matrix if it is already calibrated and saved the parameters locally,
    * **video_loader()** - To load the image frame by frame.
    * **image_loader()** - To load the images for building and debugging purpose
    * **process_frame()** - To run the pipeline of the code.It expects an image as input and returns an image

## **Discussion**

The image and video results can be found in the corresponding directories. The program can successfully detect the lane in road curve scenarios. Also it can calculate the radius of the curvature and vehicle position related to the center.The detected lane and values is visualized properly.However further tuning is needed for steep curves and road with banking.
