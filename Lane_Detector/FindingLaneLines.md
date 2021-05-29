# **Finding Lane Lines on the Road**

## **The Goals of this project are the following**

* Make a pipeline that finds lanes on the road
* Test the lane detection pipeline on video clips

### Buliding the pipeline

Pipe line is built using the following 2 Classes:
* ProcessFrame
* DetectLane

### ProcessFrame - Processing the frames

The processFrame class has 4 member functions to process each frame from a videoclip.Those are listed below with a brief description.
* **grayscale()** - Converting the RGB image to grayscale
* **canny()** - Detecting the edges of the input grayscaled images
* **gaussian_blur()** - Performing gaussian blurring on the grayscaled image. This is used as a preparatory step for canny function
* **region_of_interest()** - Mask all other regions outside the area outside the decided region.The decide region is selected by cv2.fillPoly function and the appropriate vertices is passed as parameters.

### DetectLane - Detecting the lanes

The Detectlane class has 7 member functions to perform the lane detection.Those are listed below with a brief description.
* **calc_slope()** - Calculating the slope of the lines from the points(x1,y1,x2,y2)
* **calc_theta()** - Converting the slopes to degrees
* **segment_lane()** - Segmenting the lines into two categories based on their slopes which is compared against fixed angles.
* **decide_lane()** - Deciding the lines into two categories based on their on calculated slopes.
* **draw_lines()** - Extrapolating drawing the left and right lanes based on the detected lines using cv2.polyfit,cv2.poly1d,cv2.line.
* **draw_extended()** - Extending the lines till bottom side of the image using the slope of the intercept of the extrapolated lines.
* **process_image()** -  Detecting the lanes by using the pipeline.

### **Input_function()**
The method to input the video clips.

## **Testing the function**
The program is tested using two video clips.As of now the program can detect only straight lanes.

## **Conclusion**
The program can successfully detect both standard vertical solid lines and standard vertical skip lines of any color.Future enhancements are still needed for making it robust for several other scenarios.

## **Possible Future enhancements**
Make the program more robust so that it can detect the lanes with curvature.
