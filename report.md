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


### Notice ###
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


### Result ###

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
  

#### Sample output of color thresholding ####

![Color thresholding output][n_image3]

Here is a result of color threshold, right top is navigatables, left bottom is obstacles, right bottom is rock.

#### Code Inspect ####

Implementation in notebook located in method `color_thresh()` in cell **Color Thresholding**.

There is nothing to say for navigatable.

Obstacles is non navigatable minus pure blue.

```python
    __obstacles = ~__navigatable & ~((img[:,:,0] == border_mask[0]) \
                                     & (img[:,:,1] == border_mask[1]) \
                                     & (img[:,:,2] == border_mask[2]))
```

Rock include sum of red and green channle devide by blue channel. Before that, blue channel is clip to 1 to 255 to
prevent devide by 0 error.

```python
    __blue = np.clip(img[:,:,2], 1, 255)
    __rock = (img[:,:,0] > rock_thresh[0]) \
                & (img[:,:,1] > rock_thresh[0]) \
                & (((img[:,:,0].astype(np.int32) + img[:,:,1].astype(np.int32)) / __blue) > rock_thresh[1])
```

It returns 3 ndarray for navigatables, obstacles and rocks. 


### Point 2: Created a worldmap ###
> Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable 
terrain, obstacles and rock samples into a worldmap. Run `process_image()` on your test data using the moviepy 
functions provided to create video output of your result.

#### Method ####

To create world map, it processes every vision image and do following operations:

1. Perspective transform
2. Color threshold to identify navigatables, obstacles, rocks
3. Convert thresholded pixel values to rover-centric coords
4. Filter rover-centric coords pixels by viewing distance, only pixel close to rover will be used in following steps
5. Convert filtered rover-centric pixel values to world coords
6. Update word coords pixel to world map 

##### Reason to filter pixel by viewing distance #####

It is realted to how world map is built. World map build navigatable area base on comaprison of how many times a pixel 
counted as obstacle or navigatable. The pixel at far side in perspective transformed rover-centric coord image, is 
interpolated from very small amount pixel and it is no accurate. Use these pixels cause uncertainty of world map.

The more pixel close to rover, the more information is accurate. 

Now viewing distance is set to 40. I tried several distance setting: no limited,75, 50, 40, 30. It seems 40 got enough 
accuracy but did not lose too many mapping ability.
 
##### How world map is build in detail #####

The final world map is actually a binary image. It has only 2 values, 0 and 255. One reason is to display image 
correctly. Another reason is to make path selection logic easier.
 
But a binary image is not useful during creating world map. So another temp world map is created to keep number how many
times a pixel is considered as obstacle or navigatable. The temp world map has 3 channel, 0 for number of obstacle, 1
for navigatable, last channel for navigatable seem within distance 10. The last channel will be considered as high 
priority information and will directly be used as navigatable in final world map. It is very useful when navigatable 
terrain is very narrow.

The brief step to create world map after pixel is transformed into world coords:
1.  If any pixel is considered as navigatable within distance 10, it would be navigatable in final world map.
2.  If any pixel is considered as navigatable and never considered as obstacle, it would be navigatable.
3.  If any pixel is considered as obstacles and never considered as navigatable, it would be obstacle.
4.  If any pixel is considered as obstacles and as navigatable, it depends on how many times considered as obstacle or
    navigatable in a visoin image. In note book, number of obstacles > 66% of number of navigatable, means obstacle. Or
    it would be navigatable.
5.  Rocks will be set to world map without any processing. 

#### Sample output of world map ####

![World map output][n_image5]

Here is a screen capture of final output video. The image at left bottom is the ground truth of the world. The image 
at right side is the world map, red means obstacles, blue means navigatables, yellow means rocks. The rock images is 
processed so it is tiny yellow spot on the map.

The text at the top of the image has mapped percentage and fidelity. The images used to build video walk throught the
entire map. The screen capture is capture near the end of video, the number of mapped and fidelity is the final value.

Mapped is 87% and it is not close to 100%. There are 2 reasons:
1.  Current viewing distance is 40, it is very close to rover and it did not use information at far side. Rover only
    follow the map edge during recording, it did not cross open ground. That mean center of open ground will not be
    used in mapping, like 2 black areas in the middle and right of world map image.
2.  For any particular world map pixel, it will be considered as obstacles if its numbers counted as obstables is larger
    than 66% of its numbers counted as navigatables.

#### Code Inspect ####

Realted code are in 2 cells. Major process is defined in function `process_image` in cell **Write a function to process 
stored images**. All used functions are defined in cell **Coordinate Transformations**. A `temp_worldmap` property is
added to data structure `Databucket` in cell **Read in saved data and ground truth map of the world**.

##### Main process #####

`process_image` in cell **Write a function to process stored images**

The process at begin is exactly same as class there is no much to said. Only differece are:
1. Perspective transform matrix use my own version
```python
    __source = np.float32([[8, 145], [311, 145], [201, 97], [119, 97]])
```
2. Color threshold return 3 ndarrays.
```python
    __navigatable, __obstacles, __rock  = color_thresh(__warped, ground_thresh=NAVIGATABLE_THRESHOLD, 
                                                       rock_thresh=ROCK_THRESHOLD, border_mask=BORDER_MASK)
```
The preprocess to build world map included 4 steps for each kind of pixel, following is code for obstacles. Second line
is viewing distance filter, forth line will save output to a temp world map added in `Databucket`, no the final world 
map.

```python
    __xpix_o, __ypix_o = rover_coords(__obstacles)
    __xpix_o_ns, __ypix_o_ns = filter_by_polar_distance(__xpix_o, __ypix_o, VIEW_WINDOW_DISTANCE)
    __xwd_o_ns, __ywd_o_ns = pix_to_world(__xpix_o_ns, __ypix_o_ns, data.xpos[data.count], data.ypos[data.count], 
                                    data.yaw[data.count], __world_size, __scale)
    data.temp_worldmap[__ywd_o_ns, __xwd_o_ns, 0] += 1
```

The last step to create world map will draw conclusion from temp world, It get obstacles, navigatable, close navigatable
from temp world map, then calculate world map.   
 
 ```python
    oa_sel = data.temp_worldmap[:, :, 0] > 0
    na_sel = data.temp_worldmap[:, :, 1] > 0
    na2_sel = data.temp_worldmap[:, :, 2] > 0
    oo_sel = data.temp_worldmap[:, :, 0] * 3 > data.temp_worldmap[:, :, 1] * 2
    data.worldmap[:, :, 0] = 0
    data.worldmap[oa_sel, 0] = 255
    data.worldmap[na_sel, 0] = 0
    data.worldmap[oo_sel, 0] = 255
    data.worldmap[na2_sel, 0] = 0
    data.worldmap[:, :, 2] = 0
    data.worldmap[na_sel, 2] = 255
    data.worldmap[oo_sel, 2] = 0
    data.worldmap[na2_sel, 2] = 255
```

##### Used functions #####

Functions in cell **Write a function to process stored images** has no change from class. Only one method 
`filter_by_polar_distance` is added to do view distance filter.

```python
def filter_by_polar_distance(x_pixel, y_pixel, distance):
    __dist = np.sqrt(x_pixel ** 2 + y_pixel ** 2)
    __angles = np.arctan2(y_pixel, x_pixel)
    __angles = __angles[__dist < distance]
    __dist = __dist[__dist < distance]
    __x = __dist * np.cos(__angles)
    __y = __dist * np.sin(__angles)
    return __x, __y
```

## Autonomous Navigation / Mapping ##

[//]: # (Image References)
[a_image1]: ./misc/rover_sim_finish.png
[a_image2]: ./misc/rover_sim_stuck.png
[a_image3]: ./misc/rover_sim_vision.png
[a_image4]: ./misc/rover_sim_rock.png
[a_image5]: ./misc/rover_sim_rock_large.png

### Notice ###

1.  Rover simulator is run in 800 x 600 resolution with good quality. FPS is about 30.
2.  No ground truth data used in any decision. 
3.  Original code including `driver_rover.py`, `supproting_functions.py` and `preception.py` are reformated to remove
    warning message in PyCharm IDE.
4.  All `print()` are commeted out to reduce output in console. Code only print mode siwtch information in console. 
5.  `close()` method for PIL is added into `driver_rover.py` and `supproting_functions.py` to prevent resource leak.
6.  `temp_worldmap` is added in `RoverState` in `driver_rover.py` to keep raw data of world map.
    

### Result ###

Current code usually can explorer entire map by following the edge. Final score is usualy mapped 80% with 90% fidelity.
Rover usually can pick up at least hald rocks and more. After map explored, rover will go back to map coords (100, 90).

Code has been tested and uploaded to git hub and can be found 
[here](https://github.com/kalki/RoboND-Rover-Project/tree/master/code). 

Original files:
*   `decision.py`: update control loop
*   `drive_rover.py`: minor change to re-format, close resource, add temp world map
*   `supporting_functions.py`: minot change to re-format, close resource
*   `perception.py`: updated to d vision transform, color threshold, world coords transform and update world map

New files:
*   `rover_controller.py`: to define behavior of rover control in functions, to reuse them in control loop.
*   `rover_status.py`: to define logic to detect rover status, like moving status, vision.
*   `path_finder.py`: logic to find path on a navigatable map fro rover to a point.
*   `rock_locator.py`: to define logic to detect rock from rover vision.
*   `navigatable_area.py`: to define areas in rover vision. Use to decide direction and brake. 
*   `rectangle.py`: utility class
 
A video file recorded a single round pf Rover Simulator test has been uploaded to git hhub. This round took about 13 
minutes and pick up 3 rock (5 rocks is found).

![End of video][a_image1]
  
[Download video file here](https://github.com/kalki/RoboND-Rover-Project/raw/master/misc/rover_sim_output.mp4)

#### Open Issues ####

1.  Sample rock is ignored some times.Rover follow left edge of ground, so it did not pick up rock at right side to 
    prevent break current path. 
2.  Sometime sample rock picking is interrupt. It is because visual lose due to large pitch change cause by brake.
3.  Some sample rocks can not be found because: 1) rover will no go into very small cornor to prevent stuck in it. 2)
    viewing range is only 40.
4.  Rover can not handle large rocks in the ground very well. Sometimes rover is stuck because rock is actually not in
    camera, it just stuck the rover. But usually rover will figure a way to get out. In 4:30 of provided video, rover 
    is stuck for about 20 seconds.
    ![Rover is stuck][a_image2]


### Point 1: Perception and decision ###
> Fill in the perception_step() (at the bottom of the perception.py script) and decision_step() (in decision.py) 
> functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these 
> functions were modified as they were.

Point 1 will be discussed in 2 parts:  perception and decision


### Point 1: Perception Part ###

Preception part is almost same as what is done in notebook. Please refer to notebook part about color thresholding and
world mapping. There are 3 things to be mentioned as followed.

####  Render rock as a dot ####

Rock is rendered as a yellow dot in vision image. 

Left one is a standard screen shot when rock is in sight, yellow dot is very hard to see but it is in there. Right
one is a modified version (change rock width to 5 pixel in code), there is a bright dot in the vision image.
    
| Standard output | Code is modified with rock width in 5 pixels |
| --- | ---|
|![Standard output][a_image4]  | ![Modified output][a_image5] |

##### Reason #####

Reason is to simplify rock detect and pick logic in decision step. Green channel of vision image is used to detect and 
decide rover direction when picking rock. A single dot is more accurate and easy to calculate then many dots.
    
##### Code Inspect #####

**`preception.py`**

Code start at line 183 in `preception.py` by call `render_rock()` and put rock dot into green channel of vision 
image,   

```python
    ro.render_rock(rover.vision_image[:, :, 1], __rock, 255, 0, 255)
```

**`rock_locator.render_rock()`**

The rock processing is done by `render_rock()` method in `rock_locator.py`. The method will increase value of rock
pixel in green channle of input vision image, rock pixel is found from second input which is rock output of color 
threshold. The method first use `is_rock_insight()` method to scan `rock_img` input to decide if there is rock appears 
in . Then it use `get_closest_rock()` method to get pixel of closest rock in `rock_ime`. Then it call `__render_image` 
method to increase pixel value in `img` and clip value between 0 - 255.

**`rock_locator.is_rock_insight()`**

The `is_rock_insight()` method uses 3 predefined rectangle area, and see if there is nonzero pixel from `rock_img` in 
them. These 3 area are in left side of rover. That means rock at right side of rover will be ignore. It is to keep rover
following left edge.

**`rock_locator.get_closest_rock()`**

The `get_closest_rock()` method first translate `rock_img` into rover-centric corrds, then translate it into polar 
coords, and get closest one by sorting them by distance. Before return, closest polor coords is translate back to 
rectanglar coords and camera corrds.

####  Difference in identifing obstacles ####

In rover code, pixel with obstacle count > navigatable count will be considered as obstacles. It is different from 
obstacle count > 60% * navigatable count in notebook. To do this is to make more navigatable pixel to make path find 
easier.

Code is at line 215 of `perception.py` 

```python
    __oo_sel = rover.temp_worldmap[:, :, 0] > rover.temp_worldmap[:, :, 1]
```

#### Areas highlight in vision image ####

Beside color threshold result updated to vision image, some blocks in vision image are highlighted. These blocks are 
areas use to detect obstacles ahead of rover to control the rover.

##### Reason #####
The reason to highlight is to make problem solving easier when watching the rover running.  
    
##### Code Inspect #####
Code is at line 180 of `perception.py`

```python
    nav.render_image(rover.vision_image[:, :, 0], 127, 0, 255)
```

These highlight will not effect decision since hightlight part only modify red channel (for obstacles), and 
`decision.py` use blue channel (for navigatable) for decision.


#### Point 1: Decision Part ####

Decision is seperated into 2 parts: 
1. expolor map and pick rock sample
2. decide when to return and return to middle of map

##### To be detailed #####

#### Output ####

#### Code Inspect ####


### Point 2: Rover autonomous mode test ###

> Launching in autonomous mode your rover can navigate and map autonomously. Explain your results and how you might 
> improve them in your writeup.

#### Method ####

##### To be detailed #####

#### Output ####

#### Code Inspect ####


### Further improvement ###

