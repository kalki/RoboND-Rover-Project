# Project Report: Search and Sample Return



## Summary ##

* All rubric points and required steps has been done.
* Notebook has been completed and all functions has benn implemented. 
* Notebook has been tested over provided test data and my own recorded data. Video is generated base on my own data.
* Code for automatically navigate and map has been implemented.
* Rover simulator with code has been tested. Usually rover can get 80% map with 90% fidelity and pick several 
rocks and go back to middle of map in 15 minutes.
* Code for rover can not handle rocks very good and some times it is stuck by rock. It will effect final result.   
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
    
    `VIEW_WINDOW_DISTANCE`: only pixel close to rover will be used to create map, here is distance threshold to rover. 
     
    `NAVIGATABLE_THRESHOLD`: Color definition for navigable threshold.
    
    `ROCK_THRESHOLD`: Color definition for rock threshold, first value is for red/green channel threshold, second
    value is for multiple threshold between sum of red and green channel and blue channel.
     
     `BORDER_MASK`: Define pure blue color for pixel in warp image which is not in rover's sight. Choosing blue for 
     such pixel, is to different from dark pixels in rover's sight which will be consider as obstacles, and eventually
     get better estimation about map.
*   Perspective transform matrix is change to `[8, 145], [311, 145], [201, 97], [119, 97]`, it is base on image on my
    own machine, it should be same as original matrix.
*   Image output of some cells are changed to make more detail output, like in cell for **Color Thresholding**, it 
    outputs 4 images instead of 1.


### Result ###

The notebook, a markdown file with images exported from notebook, and output video have been uploaded to github. It is
run over my own recorded data.

Check [notebook](https://github.com/kalki/RoboND-Rover-Project/blob/master/code/Rover_Project_Test_Notebook.ipynb)
for detail code and output image.

Here is [markdown version](https://github.com/kalki/RoboND-Rover-Project/blob/master/code/notebook_output/Rover_Project_Test_Notebook.md).

Download [video file](https://github.com/kalki/RoboND-Rover-Project/raw/master/output/test_mapping.mp4) here.
 
   
## Notebook Point 1: Obstacle and rock sample identification ##
> Run the functions provided in the notebook on test images (first with the test data provided, next on data you have 
recorded). Add/modify functions to allow for color selection of obstacles and rock samples.

### Method ###

Navigable, obstacles and rocks are handled in different way.

*   Navigable: any pixel which has RGB value above `NAVIGATABLE_THRESHOLD` will be considered as navigable in 
    vision image. `NAVIGATABLE_THRESHOLD` choose default value (160, 160, 160).
*   Obstacles: any pixel which is no navigable, and not pure blue (0, 0, 255), will be considered as obstacles.
    Reason explained below.
*   Rocks: any pixel that looks yellow will be considered as rock. 

    Detail methods is choose pixel which has both red and green channel value above 120 (first element of 
    `ROCK_THRESHOLD`), and sum of red and green channel is 3 (second element of `ROCK_THRESHOLD`) times larger than 
    blue channel.
    
#### Reason to exclude blue in obstacles threshold ####

After perspective transform, the pixel in warped image at left bottom and right bottom corner is black, and it is no
from real vision. Considering these as obstacles provides following steps inaccurate information and cause inaccurate 
mapping. By setting `borderValue` parameter to blue in `cv2.warpPerspective()`, code will generate blue border in
perspective transform image, then it is easy to eliminate these in threshold.

| Original | Blue border |
| --- | ---|
|![Original perspective transformed image][n_image1]  | ![Blue border perspective transformed image][n_image2] |
  

### Sample output of color threshold ###

![Color threshold output][n_image3]

Here is a result of color threshold, right top is navigable, left bottom is obstacles, right bottom is rock.

### Code Inspect ###

Implementation in notebook located in method `color_thresh()` in cell **Color Thresholding**.

There is nothing to say for navigable.

Obstacles is non navigable minus pure blue.

```python
    __obstacles = ~__navigatable & ~((img[:,:,0] == border_mask[0]) \
                                     & (img[:,:,1] == border_mask[1]) \
                                     & (img[:,:,2] == border_mask[2]))
```

Rock include sum of red and green channel divide by blue channel. Before that, blue channel is clip to 1 to 255 to
prevent divide by 0 error.

```python
    __blue = np.clip(img[:,:,2], 1, 255)
    __rock = (img[:,:,0] > rock_thresh[0]) \
                & (img[:,:,1] > rock_thresh[0]) \
                & (((img[:,:,0].astype(np.int32) + img[:,:,1].astype(np.int32)) / __blue) > rock_thresh[1])
```

It returns 3 ndarray for navigable, obstacles and rocks. 


## Notebook Point 2: Created a world map ##
> Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable 
terrain, obstacles and rock samples into a world map. Run `process_image()` on your test data using the moviepy 
functions provided to create video output of your result.

### Method ###

To create world map, it processes every vision image and do following operations:

1. Perspective transform
2. Color threshold to identify navigable, obstacles, rocks
3. Convert threshold pixel values to rover-centric coord
4. Filter rover-centric coord pixels by viewing distance, only pixel close to rover will be used in following steps
5. Convert filtered rover-centric pixel values to world coord
6. Update word coord pixel to world map 

#### Reason to filter pixel by viewing distance ####

It is related to how world map is built. World map build navigable area base on comparison of how many times a pixel 
counted as obstacle or navigable. The pixel at far side in perspective transformed rover-centric coord image, is 
interpolated from very small amount pixel and it is no accurate. Use these pixels cause uncertainty of world map.

The more pixel close to rover, the more information is accurate. 

Now viewing distance is set to 40. I tried several distance setting: no limited,75, 50, 40, 30. It seems 40 got enough 
accuracy but did not lose too many mapping ability.
 
#### How world map is build in detail ####

The final world map is actually a binary image. It has only 2 values, 0 and 255. One reason is to display image 
correctly. Another reason is to make path selection logic easier.
 
But a binary image is not useful during creating world map. So another temp world map is created to keep number how many
times a pixel is considered as obstacle or navigable. The temp world map has 3 channel, 0 for number of obstacle, 1
for navigable, last channel for navigable seem within distance 10. The last channel will be considered as high 
priority information and will directly be used as navigable in final world map. It is very useful when navigable 
terrain is very narrow.

The brief step to create world map after pixel is transformed into world coord:
1.  If any pixel is considered as navigable within distance 10, it would be navigable in final world map.
2.  If any pixel is considered as navigable and never considered as obstacle, it would be navigable.
3.  If any pixel is considered as obstacles and never considered as navigable, it would be obstacle.
4.  If any pixel is considered as obstacles and as navigable, it depends on how many times considered as obstacle or
    navigable in a vision image. In note book, number of obstacles > 66% of number of navigable, means obstacle. Or
    it would be navigable.
5.  Rocks will be set to world map without any processing. 

### Sample output of world map ###

![World map output][n_image5]

Here is a screen capture of final output video. The image at left bottom is the ground truth of the world. The image 
at right side is the world map, red means obstacles, blue means navigable, yellow means rocks. The rock images is 
processed so it is tiny yellow spot on the map.

The text at the top of the image has mapped percentage and fidelity. The images used to build video walk through the
entire map. The screen capture is capture near the end of video, the number of mapped and fidelity is the final value.

Mapped is 87% and it is not close to 100%. There are 2 reasons:
1.  Current viewing distance is 40, it is very close to rover and it did not use information at far side. Rover only
    follow the map edge during recording, it did not cross open ground. That mean center of open ground will not be
    used in mapping, like 2 black areas in the middle and right of world map image.
2.  For any particular world map pixel, it will be considered as obstacles if its numbers counted as obstacles is larger
    than 66% of its numbers counted as navigable.

### Code Inspect ###

Related code are in 2 cells. Major process is defined in function `process_image` in cell **Write a function to process 
stored images**. All used functions are defined in cell **Coordinate Transformations**. A `temp_worldmap` property is
added to data structure `Databucket` in cell **Read in saved data and ground truth map of the world**.

#### Main process ####

`process_image` in cell **Write a function to process stored images**

The process at begin is exactly same as class there is no much to said. Only difference is perspective transform matrix 
use my own version:

```python
    __source = np.float32([[8, 145], [311, 145], [201, 97], [119, 97]])
```
The pre-process to build world map included 4 steps for each kind of pixel, following is code for obstacles. Second line
is viewing distance filter, forth line will save output to a temp world map added in `Databucket`, no the final world 
map.

```python
    __xpix_o, __ypix_o = rover_coords(__obstacles)
    __xpix_o_ns, __ypix_o_ns = filter_by_polar_distance(__xpix_o, __ypix_o, VIEW_WINDOW_DISTANCE)
    __xwd_o_ns, __ywd_o_ns = pix_to_world(__xpix_o_ns, __ypix_o_ns, data.xpos[data.count], data.ypos[data.count], 
                                    data.yaw[data.count], __world_size, __scale)
    data.temp_worldmap[__ywd_o_ns, __xwd_o_ns, 0] += 1
```

The last step to create world map will draw conclusion from temp world, It get obstacles, navigable, close navigable
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

#### Used functions ####

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
[a_image6]: ./misc/rover_sim_vision_areas.png
[a_image7]: ./misc/rover_sim_vision_areas_name.png
[a_image8]: ./misc/rover_sim_rock_areas.png

### Notice ###

1.  Rover simulator is run in 800 x 600 resolution with good quality. FPS is about 30.
2.  No ground truth data used in any decision. 
3.  Original code including `driver_rover.py`, `supproting_functions.py` and `preception.py` are re-formatted to remove
    warning message in PyCharm IDE.
4.  All `print()` are commented out to reduce output in console. Code only print mode switch information in console. 
5.  `close()` method for PIL is added into `driver_rover.py` and `supproting_functions.py` to prevent resource leak.
6.  `temp_worldmap` is added in `RoverState` in `driver_rover.py` to keep raw data of world map.
    

### Result ###

Current code usually can explorer entire map by following the edge. Final score is usually mapped 80% with 90% fidelity.
Rover usually can pick up at least half rocks and more. After map explored, rover will go back to map coord (100, 90).

Code has been tested and uploaded to github and can be found 
[here](https://github.com/kalki/RoboND-Rover-Project/tree/master/code). 

Original files:
*   `decision.py`: update control loop
*   `drive_rover.py`: minor change to re-format, close resource, add temp world map
*   `supporting_functions.py`: minot change to re-format, close resource
*   `perception.py`: updated to d vision transform, color threshold, world coord transform and update world map

New files:
*   `rover_controller.py`: to define behavior of rover control in functions, to reuse them in control loop.
*   `rover_status.py`: to define logic to detect rover status, like moving status, vision.
*   `path_finder.py`: logic to find path on a navigable map fro rover to a point.
*   `rock_locator.py`: to define logic to detect rock from rover vision.
*   `navigatable_area.py`: to define areas in rover vision. Use to decide direction and brake. 
*   `rectangle.py`: utility class
 
A video file recorded a single round of Rover Simulator test has been uploaded to github. This round took about 13 
minutes and pick up 3 rock (5 rocks is found).

![End of video][a_image1]
  
[Download video file here](https://github.com/kalki/RoboND-Rover-Project/raw/master/misc/rover_sim_output.mp4)

### Open Issues ###

1.  Sample rock is ignored some times.Rover follow left edge of ground, so it did not pick up rock at right side to 
    prevent break current path. 
2.  Sometime sample rock picking is interrupt. It is because visual lose due to large pitch change cause by brake.
3.  Some sample rocks can not be found because: 1) rover will no go into very small corner to prevent stuck in it. 2)
    viewing range is only 40.
4.  Rover can not handle large rocks in the ground very well. Sometimes rover is stuck because rock is actually not in
    camera, it just stuck the rover. But usually rover will figure a way to get out. In 4:30 of provided video, rover 
    is stuck for about 20 seconds.
    
    ![Rover is stuck][a_image2]


## Rover Sim Point 1: Perception ###
> Fill in the perception_step() (at the bottom of the perception.py script) and decision_step() (in decision.py) 
> functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these 
> functions were modified as they were.

Point 1 will be discussed in 2 parts:  perception and decision. This section is only for perception. next section will
discuss decision.

Perception part is almost same as what is done in notebook. Please refer to notebook part about color threshold and
world mapping. There are 3 things to be mentioned as followed.

###  Render rock as a dot ###

Rock is rendered as a yellow dot in vision image. 

Left one is a standard screen shot when rock is in sight, yellow dot is very hard to see but it is in there. Right
one is a modified version (change rock width to 5 pixel in code), there is a bright dot in the vision image.
    
| Standard output | Code is modified with rock width in 5 pixels |
| --- | ---|
|![Standard output][a_image4]  | ![Modified output][a_image5] |

#### Why just 1 pixel for rock ####

Reason is to simplify rock detect and pick logic in decision step. Green channel of vision image is used to detect and 
decide rover direction when picking rock. A single dot is more accurate and easy to calculate then many dots.
    
#### Code Inspect ####

**`preception.py`**

Code start at line 183 in `preception.py` by call `render_rock()` and put rock dot into green channel of vision 
image,   

```python
    ro.render_rock(rover.vision_image[:, :, 1], __rock, 255, 0, 255)
```

**`rock_locator.render_rock()`**

The rock processing is done by `render_rock()` method in `rock_locator.py`. The method will increase value of rock
pixel in green channel of input vision image, rock pixel is found from second input which is rock output of color 
threshold. The method first use `is_rock_insight()` method to scan `rock_img` input to decide if there is rock appears 
in . Then it use `get_closest_rock()` method to get pixel of closest rock in `rock_ime`. Then it call `__render_image` 
method to increase pixel value in `img` and clip value between 0 - 255.

**`rock_locator.is_rock_insight()`**

The `is_rock_insight()` method uses 3 predefined rectangle area, and see if there is nonzero pixel from `rock_img` in 
them. These 3 area are in left side of rover. That means rock at right side of rover will be ignore. It is to keep rover
following left edge.

**`rock_locator.get_closest_rock()`**

The `get_closest_rock()` method first translate `rock_img` into rover-centric coord, then translate it into polar 
coord, and get closest one by sorting them by distance. Before return, closest polar coord is translate back to 
rectangular coord and camera coord.

###  Difference in identifying obstacles ###

In rover code, pixel with obstacle count > navigable count will be considered as obstacles. It is different from 
obstacle count > 60% * navigable count in notebook. To do this is to make more navigable pixel to make path find 
easier.

Code is at line 215 of `perception.py` 

```python
    __oo_sel = rover.temp_worldmap[:, :, 0] > rover.temp_worldmap[:, :, 1]
```

### Areas highlight in vision image ###

Beside color threshold result updated to vision image, some blocks in vision image are highlighted. These blocks are 
areas use to detect obstacles ahead of rover to control the rover.

![Highlight areas][a_image3]

#### Why highlight ####
The reason to highlight is to make problem solving easier when watching the rover running.  
    
#### Code Inspect ####

Code is at line 180 of `perception.py`

```python
    nav.render_image(rover.vision_image[:, :, 0], 127, 0, 255)
```

These highlight will not effect decision since highlight part only modify red channel (for obstacles), and 
`decision.py` use blue channel (for navigable) for decision.


## Rover Sim Point 1: Decision Part ##

Decision is separated into several actions. In normal case, rover decision will perform following task 
1.  Move forward to explore map with default behavior provided in original code
2.  If rover find there is not navigable edge at left side, it start to follow that edge.
3.  Decide when to return by check if it has been current position with same angle before.
4.  When it decide to return, it use navigable map to calculate a path for move from current position to predefined
    return point (100, 90). And then follow the way point to move to return point
5.  During exploring the map in move forward or follow edge mode, if any rock is found, it will initiate pick mode
    to pick rock sample
6.  When rover is stuck for several seconds in move forward or follow edge mode, it will initiate turn right 15 degree
    and try again to move forward.
7.  When rover has no enough vision to navigate in move forward or follow edge mode, it will initiateStop and turn 
    right until there are enough vision and try again to move forward.
8.  When rover is following left edge and suddenly edge is missing in vision, it will turn left 60 degree to see if 
    there is edge to follow, or switch to normal move forward mode.
9.  In general, if rover is not stable (mean pitch or roll is greater than threshold), rover will do nothing. It is 
    because any decision is heavily depends on camera input and input image will not be useful if rover has large pitch 
    or roll. Many actions cause such problem like brake, stuck or hit by rock. 


### Vision detection ###

Many decision are depends on whether vision image are navigable. Image is divided into several areas to simplify the 
detection.

Here are current defined areas in real size in vision image. 

![Vision image areas][a_image6]

Here are areas enlarged with name. Real name will be prefix with side like `L_` for left side and `R_` for right side, 
like L_FRONT_CLOSE.

![Vision image areas name][a_image7]

Be noticed that left impact, close front, far front are 1 pixel narrow than right side area. It is to make rover closer 
to edge. 

#### Why introduce area ####

Create these areas and calculate percentage of navigable in area seems be most simple and efficient way to decide 
situation of rover. By defined a threshold for avoid or go, it is easy to define a rule like:

* It there are 70% not navigable in left front far area, rover needs to turn right
* It there are 70% not navigable in left front close and right front close area, rover needs to brake
 
The decision tree in `decision.py` is a list the condition of area navigable percentage and corresponding action.
 
#### Code Inspect ####

Area detection are implemented in 2 files.

**`navigatable_area.py`**

It defined area and the coord of each area, like `L_IMPACT`, and defined name for area like `L_IMPACT_NAME`. It 
defined a list `AREAS` for all defined areas.

It also define the threshold of percentage to decide if area is clear (navigable > 95%), open (navigable > 70%), 
normal (70% > navigable > 30%), blocked (30% > navigable). It also defined a class `NavigationArea` to provides 
methods to check area, like `is_area_clear()`.
 
**`rover_status.py`**
 
`rover_status.py` defined class `RoverStatus` and provide a bunch of method like ` is_both_edge_blocked()` to simplify
and reuse area detection logic, and also make decision tree in `decision.py` more readable. 


### Rover abnormal detection ###

Sometimes rover get stuck by rock and not moving. In move forward, follow edge and navigate mode, it use position 
history to detect stuck. In rock picking mode, if rover stay in mode for 20 seconds, rover will abandon and move forward.

Rover will record it world map position and yaw every second. If it almost no change in last 8 seconds, it means rover
is stuck and will trigger action.

To decide if there is change, we use value in last 8 seconds with max difference with 8 seconds average minus the 
average. If pos change is small than 0.2 and yaw change is small than 2, it is no change. 

#### Why needs abnormal detection ####

When rover get stuck by rock, sometime there is no rock visual on vision image. In below image, rover is stuck but
vision actually looks normal narrow path. The only way to detect is use rover movement itself.
  
![Rover is stuck][a_image2]

The threshold like 0.2 and 2, it just value base on many test.

#### Code Inspect ####

Position recoding is implemented in `update_status()` method in `rover_status.py`. It compare time with last update, if 
it is more than one second, it update current status to class variables like `x_pos` defined at the begin of class.
  
Not moving judgement is implemented in `is_not_moving()` method in `rover_status.py`.

Moving judgement is used in brake situation so it needs to be quick and simple, it is implemented in `is_moving()` 
method in `rover_status.py` and only check velocity.


### When to return ###

Now it is base on if rover pass same area with same yaw. If rover is stay in such repeat mode for long time, that means
it is travel on a path it traveled before and it will trigger return operation.

Rover will keep it position and yaw every second, but position is divided by 4 and round, yaw is divided by 30 and 
round. Current position and yaw after divided and round will be checked, it is happens before, it enter same way mode. 
If it stay in this mode for 20 seconds, it trigger return.
 
To prevent some stuck trigger return, only action happens before 60 seconds will be used in check. 

#### Why use repeat action to trigger return ####

Without ground truth information, it is difficult to decide if mapping is done. One method is check if current 
navigable map are closed by obstacle map, if any navigable pixel connected to unknown pixel, that means mapping is
not finish. 

But this method need to handle navigation very well and need to decide forward angle after reach unmap position. It 
brings too many uncertainty so a simply method is choose.
  
Rover is always follow left edge, if it start repeat its path, its position and yaw should be similar. Current maximum 
speed of rover is 2m/s, so position / 4 is enough to group similar movement together. 

#### Code Inspect ####

Action recoding is implemented in `update_status()` method in `rover_status.py`. It build action by divide and round 
position and yaw, then buffer these actions in class variable Queue `action_buffer` defined at the begin of class. It 
will pop action from queue when queue contains 60 elements, and put pop action into class variable set 
`old_action_history` for further checking.
  
Action checking is implemented in `is_on_same_way_too_long()` method in `rover_status.py`. It just build current action
and see if it is in the set `old_action_history`.
 

### Follow edge mode ###

In most time, rover is in follow mode. It follow left edge and try to keep a fix distance to edge. 

#### Why follow edge ####

Follow edge is simple way to explore a unknown map when ground is closed and edge in the middle is very small. Combine
with original forward mode, it works fine for current map. And for more complex map with large edge in the middle of
map, we can introduce more logic to head to unclosed area when entire edge are explored.

#### Code Inspect ####

Follow edge mode is actually a decision tree build base on experiment, it decide when to left, when to right and when to
straight base on situation of predefined area at left side of rover.

Decision tree is implemented from line 219 in `decision.py`. Area detection logic is defined in methods in 
`rover_status.py`, and control logic like turn left, turn right is defined in methods in `rover_control.py`


### Move forward mode ###

Move forward mode is basically same as provided in original code with extra abnormal detection. And it also detect if 
there is edge at left guide area, it will switch to follow mode if detected.

Move forward mode is kept since is is useful as start at the begin and as start after recover from abnormal.  

#### Code Inspect ####

Code is start from line 174 in `decision.py`. It shared not moving handling, repeat action handling, block handling 
with follow edge mode. And it has its own simple decision tree, just decide when to switch to follow edge mode.  


### Rock pick ###

If rock is in predefined area at left side, rover will enter pick mode. Here is area range in visual image:  

![Rock detect areas][a_image8]

The area is not very large but rock at left side eventually should enter this area.
 
The reason ignore rock too far and rock at right side is to keep rover in correct track. Pick rock too far may ignore
unexplored area at closer left side. Pick rock at right side may cause rover ignore entire area ahead.
 
Current logic may ignore several rock but it is a trade off between ability and complexity of code.

#### Code Inspect ####

Code is start from line 76 in `decision.py`. It basically decide turing angle and speed base on distance and angle of
rock.


### Navigate mode ###

When rover decide to return start point, it set a goal and enter navigate mode. Navigate mode will build a path base on 
current navigable map, and set way point and make sure between 2 way points, rover can travel straight.  

#### Path search ####

Path is a A* search algorithm, with edge penalty to make path avoid edge. Now mapping fidelity is about 90% and some 
not passable area will be marked as navigable. These area will cause trouble when rover move straight through them.

Code is implemented in `search_path()` method in `path_finder.py`. It is basically a standard A* search algorithm.
  
#### Way point ####

Move one pixel by one pixel is low efficient, reduce path with connected pixel to way point will make rover move more
efficient.

Way point is build in `reduce_path` method in `path_finder.py`. It find farthest points in path which has straight 
navigable path to each other.

#### Navigate to way point ####

Basic idea is turn to right direction and set correct speed, when way point is closer, slow down or brake. When one way 
point is reached, move to next point.

Navigate code loop is start from line 119 in `decision.py`. It decide if goal is reached first, then get current way 
point. Then try to move to the point. If rover is stuck, it switch to forward mode and try to return later.
 
Control logic to move rover to way point is implemented in `head_to_waypoint()` method in `rover_controller.py`. It 
calculate angle difference and ask rover to turn, and start moving if angle difference is small enough. It also control 
speed base on distance and try not to use brake. It also try to avoid not navigable area.


### Find left edge and turn mode ###

Theses 2 modes are similar and they start from line 240 in `decision.py`. One is turn left for 60 degree, and the other
is turn right for 15 degree. 


## Rover Sim Point 2: Rover autonomous mode test ##

> Launching in autonomous mode your rover can navigate and map autonomously. Explain your results and how you might 
> improve them in your writeup.

### Test result ###

Code has been tested in autonomous mode several times, Usually rover can explorer entire map by following the edge. 

Average score is mapped 80% with 90% fidelity. But some times cause by rock blocking, rover may ignore the bottom path
in the map. 

Rover usually pick at least half rocks, depends on rock position, could be more. Rock in the middle of open ground and
in the deep corner will be ignore.

Usually rover is able to return to return point.

Entire process usually take more that 10 minutes.

Refer to 

### Further improvement ###

#### Mapping ####

1.  Current mapping logic by comparing count of obstacle and navigable is not good enough. Distance should be 
    consider more. 
2.  Use entire vision image is not correct, only edge has meaning. For example, a rock will block sight entire area,
    area behind rock will be consider as obstacles in single image processing, that increase the count of obstacle for
    those unseen area.
3.  Pitch and roll during movement could interfere mapping, image transform should consider more of these.
4.  Need a better way to identify rock in ground. These rock may not be seen by rover's camera when they stop 
    rover. Depends on rock's shape, it is possible small at the bottom.
    
#### Explore ####

1.  With more accurate navigable map, it could just if map is explored more accurate by check boundary of navigated.
2.  Rover's speed could be faster if it can identify rock better and avoid it.
3.  Rover need to handle hard corner better, by refine area definition and decision tree.
4.  Rover should introduce right edge follow mode to handle exception case better.

#### Rock ####

1. Consider rock at right side if rover can navigate in very small range.
2. Get better approaching method, avoid obstacles.
3. In rock picking, handle rock lost in visual better, tolerate short missing. Try to find rock if lost in visual.

#### Navigate ####

1. Handle close distance (with 2 pixel) better, to keep angle stable.
2. Avoid obstacle better by consider more vision image input.
3. Avoid rock better.

#### Exception ####

1. When rover get stuck, try to do opposite move.
2. Try to keep original direction if get out from stuck, also mark it on the map tp avoid it later.


