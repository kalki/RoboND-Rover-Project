# Project Report: Search and Sample Return

## Summary ##

* All rubric poins and required steps has been done.
* Notebook has been completed and all functions has benn implemented. 
* Notebook has been tested over provided test data and my own recorded data. Video is generated base on my own data.
* Code for automatically navigate and map has been implemented.
* Rover simlutator with code has been tested. Usually rover can get 80% map with 90% fidelity and pick several 
rocks and go back to middle of map in 10 minutes.
* Code for rover can not handle rocks very good and some times it is stucked by rock. It will effect final result.   
* Codes and output files has been uploaded to [my github repository](https://github.com/kalki/RoboND-Rover-Project)

## Notebook Analysis ##

[//]: # (Image References)
[n_image1]: ./misc/warp_output.png
[n_image2]: ./misc/warp_output_modified.png
[n_image3]: ./misc/color_threshold_output.png
[n_image4]: ./misc/polar_output.png
[n_image5]: ./misc/video_output.png

#### Notice ####
*   Several variables are defined in first cell for all methods and code in following cells, like:
 
    `RECORDED_PATH`: to specify image path.
    
    `VIEW_WINDOW_DISTANCE`: only pxiel close to rover will be used to create map, here is distance threshold to rover. 
     
    `NAVIGATABLE_THRESHOLD`: Color definition for navigatable threshold.
    
    `ROCK_THRESHOLD`: Color definition for rock threshold, first value is for red/green channel threshold, second
    value is for multiple threshold between sum of red and green channel and blue channel.
     
     `BORDER_MASK`: Define pure blue color for pixel in warp image which is not in rover's sight. Choosing blue for 
     such pixel, is to different from dark pixels in rover's sight which will be consider as obstables, and eventually
     get better estimation about map.
*   Perspective transform matrix is change to `[8, 145], [311, 145], [201, 97], [119, 97]`, it is base on image on my
    own machine, it should be same as original matrix.
*   Image output of some cells are changed to make more detail output, like in cell for **Color Thresholding**, it 
    outputs 4 images instead of 1.

### Notebook Result ###

The notebook, a markdown file with images exported from notebook, and output video have been uploaded to GIT. It is run 
over my own recorded data.

Check [notebook](https://github.com/kalki/RoboND-Rover-Project/blob/master/code/Rover_Project_Test_Notebook.ipynb)
for detail code and output image.

Here is [markdown version](https://github.com/kalki/RoboND-Rover-Project/blob/master/code/notebook_output/Rover_Project_Test_Notebook.md).

Download [video file](https://github.com/kalki/RoboND-Rover-Project/raw/master/output/test_mapping.mp4) here.
 
   


### Point 1: Obstacle and rock sample identification ###
> Run the functions provided in the notebook on test images (first with the test data provided, next on data you have 
recorded). Add/modify functions to allow for color selection of obstacles and rock samples.

#### Method ####

Navigatables, obstables and rocks are handled in differen way.

*   Navigatables: any pixel which has RGB value above `NAVIGATABLE_THRESHOLD` will be considerdd as navigatables in 
    vision image. `NAVIGATABLE_THRESHOLD` choose default value (160, 160, 160).
*   Obstacles: any pixel which is no navigatables, and not pure blue (0, 0, 255), will be considered as obstacles.
    Reason explained below.
*   Rocks: any pixel that looks yellow will be considered as rock. 

    Detail methos is choose pixel which has both red and green channel value above 120 (first element of 
    `ROCK_THRESHOLD`), and sum of red and green channel is 3 (second element of `ROCK_THRESHOLD`) times larger than 
    blue channel.
    
##### Reason to exclude blue in obstacles thresholding #####

After perspective transform, the pixel in warpped image at left bottom and right bottom cornor is black, and it is no
from real vision. Considering these as obstacles provides following steps inaccurate information and cause inaccurate 
mapping. By setting `borderValue` parameter to blue in `cv2.warpPerspective()`, code will generate blue border in
perspective transform image, then it is easy to eliminate these in thresholding.


| Original | Blue border |
| --- | ---|
|![Original perspective transformed image][n_image1]  | ![Blue borded perspective transformed image][n_image2] |
  

##### Result #####

Here is a result of color threshold, right top is navigatables, left bottom is obstacles, right bottom is rock.

![Color thresholding output][n_image3]

##### Code #####

Implementation in notebook located in method `color_thresh()` in cell **Color Thresholding**.
 
It returns 3 ndarray for navigatables, obstacles and rocks. 

#### Point 2: Created a worldmap ####
> Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable 
terrain, obstacles and rock samples into a worldmap. Run `process_image()` on your test data using the moviepy 
functions provided to create video output of your result.

##### Method #####

##### Result #####

##### Code #####



## Autonomous Navigation / Mapping ##

