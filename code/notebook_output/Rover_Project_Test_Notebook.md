
## Rover Project Test Notebook
This notebook contains the functions from the lesson and provides the scaffolding you need to test out your mapping methods.  The steps you need to complete in this notebook for the project are the following:

* First just run each of the cells in the notebook, examine the code and the results of each.
* Run the simulator in "Training Mode" and record some data. Note: the simulator may crash if you try to record a large (longer than a few minutes) dataset, but you don't need a ton of data, just some example images to work with.   
* Change the data directory path (2 cells below) to be the directory where you saved data
* Test out the functions provided on your data
* Write new functions (or modify existing ones) to report and map out detections of obstacles and rock samples (yellow rocks)
* Populate the `process_image()` function with the appropriate steps/functions to go from a raw image to a worldmap.
* Run the cell that calls `process_image()` using `moviepy` functions to create video output
* Once you have mapping working, move on to modifying `perception.py` and `decision.py` to allow your rover to navigate and map in autonomous mode!

**Note: If, at any point, you encounter frozen display windows or other confounding issues, you can always start again with a clean slate by going to the "Kernel" menu above and selecting "Restart & Clear Output".**

**Run the next cell to get code highlighting in the markdown cells.**


```python
%%HTML
<style> code {background-color : orange !important;} </style>
```


<style> code {background-color : orange !important;} </style>



```python
%matplotlib inline
#%matplotlib qt # Choose %matplotlib qt to plot to an interactive window (note it may show up behind your browser)
# Make some of the relevant imports
import cv2 # OpenCV for perspective transform
import numpy as np
import matplotlib.image as mpimg
import matplotlib.pyplot as plt
import scipy.misc # For saving images as needed
import glob  # For reading in a list of images from a folder
from PIL import Image

# CHANGE
# Define several global variables
# RECORDED_PATH
#  for following image loding and mapping
# VIEW_WINDOW_DISTANCE 
#  for range of viewing window for mapping, pixel too far will not be consider in mapping
# NAVIGATABLE_THRESHOLD
#  for color threshold of navigatable, all channels greater than specified value will be considered as navigatable
# ROCK_THRESHOLD
#  for color threshold of rock, sum of red and green channel must greater than n times of blue channel.
RECORDED_PATH = '../../RoverSim/RecordedData/0615/'
VIEW_WINDOW_DISTANCE = 40
NAVIGATABLE_THRESHOLD = (160, 160, 160)
ROCK_THRESHOLD = (120, 3)
BORDER_MASK = (0, 0, 255)
```

## Quick Look at the Data
There's some example data provided in the `test_dataset` folder.  This basic dataset is enough to get you up and running but if you want to hone your methods more carefully you should record some data of your own to sample various scenarios in the simulator.  

Next, read in and display a random image from the `test_dataset` folder


```python
path = RECORDED_PATH + 'IMG/*'
img_list = glob.glob(path)
# Grab a random image and display it
idx = np.random.randint(0, len(img_list)-1)
image = mpimg.imread(img_list[idx])
plt.imshow(image)
```




    <matplotlib.image.AxesImage at 0x983b9e8>




![png](output_4_1.png)


## Calibration Data
Read in and display example grid and rock sample calibration images.  You'll use the grid for perspective transform and the rock image for creating a new color selection that identifies these samples of interest. 


```python
# In the simulator you can toggle on a grid on the ground for calibration
# You can also toggle on the rock samples with the 0 (zero) key.  
# Here's an example of the grid and one of the rocks
example_grid = '../calibration_images/example_grid1.jpg'
example_rock = '../calibration_images/example_rock1.jpg'
grid_img = mpimg.imread(example_grid)
rock_img = mpimg.imread(example_rock)

fig = plt.figure(figsize=(12,3))
plt.subplot(121)
plt.imshow(grid_img)
plt.subplot(122)
plt.imshow(rock_img)
```




    <matplotlib.image.AxesImage at 0x98fa908>




![png](output_6_1.png)


## Perspective Transform

Define the perspective transform function from the lesson and test it on an image.


```python
# Define a function to perform a perspective transform
# I've used the example grid image above to choose source points for the
# grid cell in front of the rover (each grid cell is 1 square meter in the sim)
# Define a function to perform a perspective transform
# CHANGE
# Define blue background color for pixel not in viewing range
def perspect_transform(img, src, dst, border_mask=(0, 0, 255)):
    __M = cv2.getPerspectiveTransform(src, dst)
    __warped = cv2.warpPerspective(img, __M, (img.shape[1], img.shape[0]), borderValue=border_mask)# keep same size as input image
    return __warped

# Define calibration box in source (actual) and destination (desired) coordinates
# These source and destination points are defined to warp the image
# to a grid where each 10x10 pixel square represents 1 square meter
# The destination box will be 2*dst_size on each side
dst_size = 5 

# Set a bottom offset to account for the fact that the bottom of the image 
# is not the position of the rover but a bit in front of it
# this is just a rough guess, feel free to change it!
bottom_offset = 6

# CHANGE
source = np.float32([[8, 145], [311, 145], [201, 97], [119, 97]])

destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                  [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                  ])
warpped_1 = perspect_transform(grid_img, source, destination, border_mask=BORDER_MASK)
plt.imshow(warpped_1)

# scipy.misc.imsave('../output/warped_example.jpg', warped)
```




    <matplotlib.image.AxesImage at 0x99eb7b8>




![png](output_8_1.png)


## Color Thresholding
Define the color thresholding function from the lesson and apply it to the warped image

**TODO:** Ultimately, you want your map to not just include navigable terrain but also obstacles and the positions of the rock samples you're searching for.  Modify this function or write a new function that returns the pixel locations of obstacles (areas below the threshold) and rock samples (yellow rocks in calibration images), such that you can map these areas into world coordinates as well.  
**Hints and Suggestion:** 
* For obstacles you can just invert your color selection that you used to detect ground pixels, i.e., if you've decided that everything above the threshold is navigable terrain, then everthing below the threshold must be an obstacle!


* For rocks, think about imposing a lower and upper boundary in your color selection to be more specific about choosing colors.  You can investigate the colors of the rocks (the RGB pixel values) in an interactive matplotlib window to get a feel for the appropriate threshold range (keep in mind you may want different ranges for each of R, G and B!).  Feel free to get creative and even bring in functions from other libraries.  Here's an example of [color selection](http://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html) using OpenCV.  

* **Beware However:** if you start manipulating images with OpenCV, keep in mind that it defaults to `BGR` instead of `RGB` color space when reading/writing images, so things can get confusing.


```python
# Identify pixels above the threshold
# Threshold of RGB > 160 does a nice job of identifying ground pixels only
def color_thresh(img, ground_thresh=(160, 160, 160), rock_thresh=(120, 3), border_mask=(0, 0, 255)):
    # Create an array of zeros same xy size as img, but single channel
    __navigatable_select = np.zeros_like(img[:,:,0])
    __obstacles_select = np.zeros_like(img[:,:,0])
    __rock_select = np.zeros_like(img[:,:,0])

    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    __navigatable = (img[:,:,0] > ground_thresh[0]) \
                & (img[:,:,1] > ground_thresh[1]) \
                & (img[:,:,2] > ground_thresh[2])

    __obstacles = ~__navigatable & ~((img[:,:,0] == border_mask[0]) \
                                     & (img[:,:,1] == border_mask[1]) \
                                     & (img[:,:,2] == border_mask[2]))
    
    __blue = np.clip(img[:,:,2], 1, 255)
    __rock = (img[:,:,0] > rock_thresh[0]) \
                & (img[:,:,1] > rock_thresh[0]) \
                & (((img[:,:,0].astype(np.int32) + img[:,:,1].astype(np.int32)) / __blue) > rock_thresh[1])

    # Index the array of zeros with the boolean array and set to 1
    __navigatable_select[__navigatable] = 1
    __obstacles_select[__obstacles] = 1
    __rock_select[__rock] = 1
    # Return the binary image for navigatable, obstacles and rock
    return __navigatable_select, __obstacles_select, __rock_select


idx = np.random.randint(0, len(img_list)-1)
image_2 = mpimg.imread(img_list[0])
warpped_2 = perspect_transform(image_2, source, destination)
threshed_navigatable, threshed_obstacles, threshed_rock = color_thresh( \
    warpped_2, \
    ground_thresh=NAVIGATABLE_THRESHOLD, \
    rock_thresh=ROCK_THRESHOLD, \
    border_mask=BORDER_MASK)
fig = plt.figure(dpi=153)
plt.subplot(221)
plt.imshow(warpped_2)
plt.subplot(222)
plt.imshow(threshed_navigatable, cmap='gray')
plt.subplot(223)
plt.imshow(threshed_obstacles, cmap='gray')
plt.subplot(224)
plt.imshow(threshed_rock, cmap='gray')
```




    <matplotlib.image.AxesImage at 0x9f3c978>




![png](output_10_1.png)


## Coordinate Transformations
Define the functions used to do coordinate transforms and apply them to an image.


```python
def rover_coords(binary_img):
    # Identify nonzero pixels
    __ypos, __xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    __x_pixel = -(__ypos - binary_img.shape[0]).astype(np.float)
    __y_pixel = -(__xpos - binary_img.shape[1]/2).astype(np.float)
    return __x_pixel, __y_pixel

# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    __dist = np.sqrt(x_pixel ** 2 + y_pixel ** 2)
    # Calculate angle away from vertical for each pixel
    __angles = np.arctan2(y_pixel, x_pixel)
    return __dist, __angles

# Define a function to convert from radial coords in rover space
def from_polar_coords(dists, angles):
    __x = dists * np.cos(angles)
    __y = dists * np.sin(angles)
    return __x, __y

# CHANGE
# Define a function to filter pixel by distance
def filter_by_polar_distance(x_pixel, y_pixel, distance):
    __dist = np.sqrt(x_pixel ** 2 + y_pixel ** 2)
    __angles = np.arctan2(y_pixel, x_pixel)
    __angles = __angles[__dist < distance]
    __dist = __dist[__dist < distance]
    __x = __dist * np.cos(__angles)
    __y = __dist * np.sin(__angles)
    return __x, __y

# Define a function to apply a rotation to pixel positions
def rotate_pix(xpix, ypix, yaw):
    # TODO: DONE
    # Convert yaw to radians
    # Apply a rotation
    __yaw_rad = yaw * np.pi / 180
    __xpix_rotated = xpix * np.cos(__yaw_rad) - ypix * np.sin(__yaw_rad)
    __ypix_rotated = xpix * np.sin(__yaw_rad) + ypix * np.cos(__yaw_rad)
    # Return the result  
    return __xpix_rotated, __ypix_rotated

# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # TODO: DONE
    # Apply a scaling and a translation
    __xpix_translated = xpos + (xpix_rot / scale)
    __ypix_translated = ypos + (ypix_rot / scale)
    # Return the result  
    return __xpix_translated, __ypix_translated

# Define a function to apply rotation and translation (and clipping)
# Once you define the two functions above this function should work
def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    __xpix_rot, __ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    __xpix_tran, __ypix_tran = translate_pix(__xpix_rot, __ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    __x_pix_world = np.clip(np.int_(__xpix_tran), 0, world_size - 1)
    __y_pix_world = np.clip(np.int_(__ypix_tran), 0, world_size - 1)
    # Return the result
    return __x_pix_world, __y_pix_world

# Grab another random image
idx = np.random.randint(0, len(img_list)-1)
image_3 = mpimg.imread(img_list[0])
warpped_3 = perspect_transform(image_3, source, destination)
threshed, _, _ = color_thresh(warpped_3, ground_thresh=NAVIGATABLE_THRESHOLD, \
                              rock_thresh=ROCK_THRESHOLD, border_mask=BORDER_MASK)

# Calculate pixel values in rover-centric coords and distance/angle to all pixels
xpix, ypix = rover_coords(threshed)
xpix, ypix = filter_by_polar_distance(xpix, ypix, VIEW_WINDOW_DISTANCE)
dist, angles = to_polar_coords(xpix, ypix)
mean_dir = np.mean(angles)

polar_image = np.zeros_like(image[:,:,0])


# Do some plotting
fig = plt.figure(dpi=153)
plt.subplot(221)
plt.imshow(image_3)
plt.subplot(222)
plt.imshow(warpped_3)
plt.subplot(223)
plt.imshow(threshed, cmap='gray')
plt.subplot(224)
plt.plot(xpix, ypix, '.')
plt.ylim(-160, 160)
plt.xlim(0, 160)
arrow_length = VIEW_WINDOW_DISTANCE
x_arrow = arrow_length * np.cos(mean_dir)
y_arrow = arrow_length * np.sin(mean_dir)
plt.arrow(0, 0, x_arrow, y_arrow, color='red', zorder=2, head_width=5, width=1)
```




    <matplotlib.patches.FancyArrow at 0xa2f0f60>




![png](output_12_1.png)


## Read in saved data and ground truth map of the world
The next cell is all setup to read your saved data into a `pandas` dataframe.  Here you'll also read in a "ground truth" map of the world, where white pixels (pixel value = 1) represent navigable terrain.  

After that, we'll define a class to store telemetry data and pathnames to images.  When you instantiate this class (`data = Databucket()`) you'll have a global variable called `data` that you can refer to for telemetry and map data within the `process_image()` function in the following cell.  



```python
# Import pandas and read in csv file as a dataframe
import pandas as pd
# Change the path below to your data directory
# If you are in a locale (e.g., Europe) that uses ',' as the decimal separator
# change the '.' to ','
df = pd.read_csv(RECORDED_PATH + 'robot_log.csv', delimiter=';', decimal='.')
csv_img_list = df["Path"].tolist() # Create list of image pathnames
# Read in ground truth map and create a 3-channel image with it
ground_truth = mpimg.imread('../calibration_images/map_bw.png')
ground_truth_3d = np.dstack((ground_truth*0, ground_truth*255, ground_truth*0)).astype(np.float)

# Creating a class to be the data container
# Will read in saved data from csv file and populate this object
# Worldmap is instantiated as 200 x 200 grids corresponding 
# to a 200m x 200m space (same size as the ground truth map: 200 x 200 pixels)
# This encompasses the full range of output position values in x and y from the sim
class Databucket():
    def __init__(self):
        self.images = csv_img_list  
        self.xpos = df["X_Position"].values
        self.ypos = df["Y_Position"].values
        self.yaw = df["Yaw"].values
        self.count = -1 # This will be a running index, setting to -1 is a hack
                        # because moviepy (below) seems to run one extra iteration
        self.worldmap = np.zeros((200, 200, 3)).astype(np.float)
        self.temp_worldmap = np.zeros((200, 200, 3)).astype(np.float)
        self.ground_truth = ground_truth_3d # Ground truth worldmap

# Instantiate a Databucket().. this will be a global variable/object
# that you can refer to in the process_image() function below
data = Databucket()
```

## Write a function to process stored images

Modify the `process_image()` function below by adding in the perception step processes (functions defined above) to perform image analysis and mapping.  The following cell is all set up to use this `process_image()` function in conjunction with the `moviepy` video processing package to create a video from the images you saved taking data in the simulator.  

In short, you will be passing individual images into `process_image()` and building up an image called `output_image` that will be stored as one frame of video.  You can make a mosaic of the various steps of your analysis process and add text as you like (example provided below).  



To start with, you can simply run the next three cells to see what happens, but then go ahead and modify them such that the output video demonstrates your mapping process.  Feel free to get creative!


```python
# Define a function to pass stored images to
# reading rover position and yaw angle from csv file
# This function will be used by moviepy to create an output video
def process_image(img):
    # Example of how to use the Databucket() object defined above
    # to print the current x, y and yaw values 
    # print(data.xpos[data.count], data.ypos[data.count], data.yaw[data.count])

    # TODO: 
    # 1) Define source and destination points for perspective transform
    # 2) Apply perspective transform
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    # 4) Convert thresholded image pixel values to rover-centric coords
    # 5) Convert rover-centric pixel values to world coords
    # 6) Update worldmap (to be displayed on right side of screen)
        # Example: data.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        #          data.worldmap[rock_y_world, rock_x_world, 1] += 1
        #          data.worldmap[navigable_y_world, navigable_x_world, 2] += 1

    # 7) Make a mosaic image, below is some example code

    # Initiate parameter
    __world_size = data.worldmap.shape[0]
    __scale = 10
    __dst_size = 5 
    __bottom_offset = 6

    __source = np.float32([[8, 145], [311, 145], [201, 97], [119, 97]])
    __destination = np.float32([[img.shape[1] / 2 - __dst_size, img.shape[0] - __bottom_offset],
                  [img.shape[1] / 2 + __dst_size, img.shape[0] - __bottom_offset],
                  [img.shape[1] / 2 + __dst_size, img.shape[0] - 2 * __dst_size - __bottom_offset], 
                  [img.shape[1] / 2 - __dst_size, img.shape[0] - 2 * __dst_size - __bottom_offset],
                  ])
    
    # First create a blank image (can be whatever shape you like)
    __output_image = np.zeros((img.shape[0] + data.worldmap.shape[0], img.shape[1]*2, 3))

    # Next you can populate regions of the image with various output
    # Here I'm putting the original image in the upper left hand corner
    __output_image[0:img.shape[0], 0:img.shape[1]] = img

    # Let's create more images to add to the mosaic, first a warped image
    __warped = perspect_transform(img, __source, __destination)
    # Add the warped image in the upper right hand corner
    __output_image[0:img.shape[0], img.shape[1]:] = __warped
    
    # Identify navigatable, obstacles and rock
    __navigatable, __obstacles, __rock  = color_thresh(__warped, ground_thresh=NAVIGATABLE_THRESHOLD, 
                                                       rock_thresh=ROCK_THRESHOLD, border_mask=BORDER_MASK)

    # Update obstacles to red channel of world map
    __xpix_o, __ypix_o = rover_coords(__obstacles)
    __xpix_o_ns, __ypix_o_ns = filter_by_polar_distance(__xpix_o, __ypix_o, VIEW_WINDOW_DISTANCE)
    __xwd_o_ns, __ywd_o_ns = pix_to_world(__xpix_o_ns, __ypix_o_ns, data.xpos[data.count], data.ypos[data.count], 
                                    data.yaw[data.count], __world_size, __scale)
    data.temp_worldmap[__ywd_o_ns, __xwd_o_ns, 0] += 1

    # Update navigatable to green channel of world map
    __xpix_n, __ypix_n = rover_coords(__navigatable)
    __xpix_n_ns, __ypix_n_ns = filter_by_polar_distance(__xpix_n, __ypix_n, VIEW_WINDOW_DISTANCE)
    __xpix_n_ns2, __ypix_n_ns2 = filter_by_polar_distance(__xpix_n, __ypix_n, VIEW_WINDOW_DISTANCE / 4)

    __xwd_n_ns, __ywd_n_ns = pix_to_world(__xpix_n_ns, __ypix_n_ns, data.xpos[data.count], data.ypos[data.count],
                                    data.yaw[data.count], __world_size, __scale)
    __xwd_n_ns2, __ywd_n_ns2 = pix_to_world(__xpix_n_ns2, __ypix_n_ns2, data.xpos[data.count], data.ypos[data.count],
                                    data.yaw[data.count], __world_size, __scale)
    data.temp_worldmap[__ywd_n_ns, __xwd_n_ns, 1] += 1
    data.temp_worldmap[__ywd_n_ns2, __xwd_n_ns2, 2] += 1

    # Update rock to blue channel of worlf map, and if any value in blue channel, reset red and green channel
    __xpix_r, __ypix_r = rover_coords(__rock)
    __xpix_r, __ypix_r = filter_by_polar_distance(__xpix_r, __ypix_r, VIEW_WINDOW_DISTANCE)
    __xwd_r, __ywd_r = pix_to_world(__xpix_r, __ypix_r, data.xpos[data.count], data.ypos[data.count], 
                                    data.yaw[data.count], __world_size, __scale)
    data.worldmap[__ywd_r, __xwd_r, 1] += 10
    
    # Clip value of 3 channels to prevent black area
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
    
    data.worldmap = np.clip(data.worldmap, 0, 255)

    # Overlay worldmap with ground truth map
    #map_add = cv2.addWeighted(data.worldmap, 1, data.ground_truth, 0.5, 0)
    # Flip map overlay so y-axis points upward and add to output_image 
    #output_image[img.shape[0]:, 0:data.worldmap.shape[1]] = np.flipud(map_add)
    __output_image[img.shape[0]:, 0:data.worldmap.shape[1]] = np.flipud(data.ground_truth)

    # Add world map to output image with flip
    __output_image[img.shape[0]:, img.shape[1]:img.shape[1]+data.worldmap.shape[1]] = np.flipud(data.worldmap)

    # Calculate some statistics on the map results
    # First get the total number of pixels in the navigable terrain map
    tot_nav_pix = np.float(len((data.worldmap[:,:,2].nonzero()[0])))
    # Next figure out how many of those correspond to ground truth pixels
    good_nav_pix = np.float(len(((data.worldmap[:,:,2] > 0) & (data.ground_truth[:,:,1] > 0)).nonzero()[0]))
    # Next find how many do not correspond to ground truth pixels
    bad_nav_pix = np.float(len(((data.worldmap[:,:,2] > 0) & (data.ground_truth[:,:,1] == 0)).nonzero()[0]))
    # Grab the total number of map pixels
    tot_map_pix = np.float(len((data.ground_truth[:,:,1].nonzero()[0])))
    # Calculate the percentage of ground truth map that has been successfully found
    perc_mapped = round(100*good_nav_pix/tot_map_pix, 1)
    # Calculate the number of good map pixel detections divided by total pixels 
    # found to be navigable terrain
    if tot_nav_pix > 0:
        fidelity = round(100*good_nav_pix/(tot_nav_pix), 1)
    else:
        fidelity = 0
    
    # Then putting axis and yaw text over the image
    if data.count < len(data.images):
        cv2.putText(__output_image,
                    "X:{:.2f},   Y:{:.2f},   Yaw:{:.4f},   Mapped:{:.2f}%,   Fidelity:{:.2f}%"
                    .format(data.xpos[data.count], data.ypos[data.count], data.yaw[data.count], perc_mapped, fidelity),
                    (20, 20), 
                    cv2.FONT_HERSHEY_COMPLEX, 0.4, (255, 255, 255), 1)

    # Keep track of the index in the Databucket()
    data.count += 1 
    
    return __output_image

```

## Make a video from processed image data
Use the [moviepy](https://zulko.github.io/moviepy/) library to process images and create a video.
  


```python
# Import everything needed to edit/save/watch video clips
from moviepy.editor import VideoFileClip
from moviepy.editor import ImageSequenceClip


# Define pathname to save the output video
output = '../output/test_mapping.mp4'
data = Databucket() # Re-initialize data in case you're running this cell multiple times
clip = ImageSequenceClip(data.images, fps=60) # Note: output video will be sped up because 
                                          # recording rate in simulator is fps=25
new_clip = clip.fl_image(process_image) #NOTE: this function expects color images!!
%time new_clip.write_videofile(output, audio=False)
```

    [MoviePy] >>>> Building video ../output/test_mapping.mp4
    [MoviePy] Writing video ../output/test_mapping.mp4
    

    100%|████████████████████████████████████████████████████████████████████████████████████████████████████████████| 3547/3547 [00:42<00:00, 83.15it/s]
    

    [MoviePy] Done.
    [MoviePy] >>>> Video ready: ../output/test_mapping.mp4 
    
    Wall time: 42.8 s
    

### This next cell should function as an inline video player
If this fails to render the video, try running the following cell (alternative video rendering method).  You can also simply have a look at the saved mp4 in your `/output` folder


```python
from IPython.display import HTML
HTML("""
<video width="960" height="540" controls>
  <source src="{0}">
</video>
""".format(output))
```





<video width="960" height="540" controls>
  <source src="../output/test_mapping.mp4">
</video>




### Below is an alternative way to create a video in case the above cell did not work.


```python
import io
import base64
video = io.open(output, 'r+b').read()
encoded_video = base64.b64encode(video)
HTML(data='''<video alt="test" controls>
                <source src="data:video/mp4;base64,{0}" type="video/mp4" />
             </video>'''.format(encoded_video.decode('ascii')))
```


```python

```
