import numpy as np
import cv2

import rock_locator as ro
import navigatable_area as nav

# For range of viewing window for mapping, pixel too far will not be consider in mapping
VIEW_WINDOW_DISTANCE = 40
# For color threshold of navigatable, all channels greater than specified value will be considered as navigatable
NAVIGATABLE_THRESHOLD = (160, 160, 160)
# For color threshold of rock, red and green channel must be larger than x,
# and sum of red and green channel must greater than y times of blue channel.
ROCK_THRESHOLD = (120, 3)
# Blue for points not in vision in warped image
BORDER_MASK = (0, 0, 255)
# Define image width and height and not calculate it
IMG_WIDTH = 320
IMG_HEIGHT = 160


# Predefined a transform matrix and do not calculate it every time
def get_perspect_transformer():
    __dst_size = 5
    __bottom_offset = 6
    __img_width = IMG_WIDTH
    __img_height = IMG_HEIGHT
    __source = np.float32([[8, 145], [311, 145], [201, 97], [119, 97]])
    __destination = np.float32([[__img_width / 2 - __dst_size, __img_height - __bottom_offset],
                                [__img_width / 2 + __dst_size, __img_height - __bottom_offset],
                                [__img_width / 2 + __dst_size, __img_height - 2 * __dst_size - __bottom_offset],
                                [__img_width / 2 - __dst_size, __img_height - 2 * __dst_size - __bottom_offset],
                                ])
    __M = cv2.getPerspectiveTransform(__source, __destination)
    return __M
TRANSFORM_MATRIX = get_perspect_transformer()


# Identify pixels of navigatable, obstacles and rocks
def color_thresh(img, nav_thresh=(160, 160, 160), rock_thresh=(120, 3), border_mask=(0, 0, 255)):
    # Create an array of zeros same xy size as img, but single channel
    __navigatable_select = np.zeros_like(img[:, :, 0])
    __obstacles_select = np.zeros_like(img[:, :, 0])
    __rock_select = np.zeros_like(img[:, :, 0])

    # Navigatable requires that each pixel be above all three threshold values in RGB
    __navigatable = (img[:, :, 0] > nav_thresh[0]) & (img[:, :, 1] > nav_thresh[1]) & (img[:, :, 2] > nav_thresh[2])

    # Obstacles is revert of navigatable, then remove pure blue point (consider as point not in vision)
    __obstacles = ~__navigatable & ~((img[:, :, 0] == border_mask[0])
                                     & (img[:, :, 1] == border_mask[1]) & (img[:, :, 2] == border_mask[2]))
    # Set all points in blue channel to at least 1 to prevent divide by zero
    __blue = np.clip(img[:, :, 2], 1, 255)
    # Rock requires red and green channel is greater that specified value
    # and sum of red and green and larger than blue in n times
    __rock = (img[:, :, 0] > rock_thresh[0]) & (img[:, :, 1] > rock_thresh[0])
    __rock = __rock & (((img[:, :, 0].astype(np.int32) + img[:, :, 1].astype(np.int32)) / __blue) > rock_thresh[1])

    # Index the array of zeros with the boolean array and set to 1
    __navigatable_select[__navigatable] = 1
    __obstacles_select[__obstacles] = 1
    __rock_select[__rock] = 1
    # Return the binary image for navigatable, obstacles and rock
    return __navigatable_select, __obstacles_select, __rock_select


# Define a function to convert to rover-centric coordinates
def rover_coords(binary_img):
    # Identify nonzero pixels
    __ypos, __xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    __x_pixel = -(__ypos - binary_img.shape[0]).astype(np.float)
    __y_pixel = -(__xpos - binary_img.shape[1] / 2).astype(np.float)
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
    # Convert yaw to radians
    # Apply a rotation
    __yaw_rad = yaw * np.pi / 180
    __xpix_rotated = xpix * np.cos(__yaw_rad) - ypix * np.sin(__yaw_rad)
    __ypix_rotated = xpix * np.sin(__yaw_rad) + ypix * np.cos(__yaw_rad)
    # Return the result  
    return __xpix_rotated, __ypix_rotated


# Define a function to perform a translation
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale):
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


# Define a function to perform a perspective transform
# Use blue for points not in vision
def perspect_transform(img, border_mask=(0, 0, 255)):
    __M = TRANSFORM_MATRIX
    __warped = cv2.warpPerspective(img, __M, (img.shape[1], img.shape[0]), borderValue=border_mask)
    return __warped


# Apply the above functions in succession and update the Rover state accordingly
def perception_step(rover):
    # Perform perception steps to update Rover()
    # 1) Get camera image is coming to you in Rover.img and initialize transform parameter
    # 2) Apply perspective transform
    # 3) Apply color threshold to identify navigable terrain/obstacles/rock samples
    # 4) Update Rover.vision_image, overlap with navigation area definition, rock image is processed
    #    and only closeset rock position is kept
    # 5) Convert map image pixel values to rover-centric coords
    # 6) Filter rover-centric pixel by distance to rover and convert filtered to world coordinates
    # 7) Update Rover worldmap, it is to be displayed on right side of screen and also use for path find in
    #    navigation mode.
    #    Red channel is obstacle, any non zero point counted as obstacles, if a point count as obstacles and
    #    navigatable,if count of obstacles is larger, and it is not counted as closer navigatable
    #    Green channel is rock with not special logic
    #    Blue channel is navigatble, any non obstacle point is navigatable, any point counted as closed navigatable
    #    is always navigatable
    # 8) Convert rover-centric distance filtered navigatable pixel positions to polar coordinates and update rover
    #    pixel distances and angles

    # STEP 1. Get image
    __img = rover.img
    assert __img.shape[0] == IMG_HEIGHT, "Image height is unexpected, that will break perspect transform"
    assert __img.shape[1] == IMG_WIDTH, "Image width is unexpected, that will break perspect transform"
    __x = rover.pos[0]
    __y = rover.pos[1]
    __yaw = rover.yaw
    __world_size = rover.worldmap.shape[0]
    __scale = 10

    # STEP 2. Apply perspective transform
    __warped = perspect_transform(__img)

    # STEP 3. Apply color threshold
    __navigatable, __obstacles, __rock = color_thresh(__warped, nav_thresh=NAVIGATABLE_THRESHOLD,
                                                      rock_thresh=ROCK_THRESHOLD, border_mask=BORDER_MASK)
    # STEP 4. Update Rover.vision_image
    rover.vision_image[:, :, 0] = __obstacles * 255
    # Overlap navigation area definition to vision image
    nav.render_image(rover.vision_image[:, :, 0], 127, 0, 255)
    rover.vision_image[:, :, 1] = np.zeros_like(__rock)
    # Set processed rock image instead of raw rock image
    ro.render_rock(rover.vision_image[:, :, 1], __rock, 255, 0, 255)
    # Set navigatable image
    rover.vision_image[:, :, 2] = __navigatable * 255

    # STEP 5. To rover-centric coords
    __xpix_o, __ypix_o = rover_coords(__obstacles)
    __xpix_n, __ypix_n = rover_coords(__navigatable)
    __xpix_r, __ypix_r = rover_coords(__rock)

    # STEP 6. Filter by distance and to world coordinates
    # Filter by distance to rover, for navigatable, also get a set of closer points
    __xpix_o_ns, __ypix_o_ns = filter_by_polar_distance(__xpix_o, __ypix_o, VIEW_WINDOW_DISTANCE)
    __xpix_n_ns, __ypix_n_ns = filter_by_polar_distance(__xpix_n, __ypix_n, VIEW_WINDOW_DISTANCE)
    __xpix_n_ns2, __ypix_n_ns2 = filter_by_polar_distance(__xpix_n, __ypix_n, VIEW_WINDOW_DISTANCE / 4)
    __xpix_r, __ypix_r = filter_by_polar_distance(__xpix_r, __ypix_r, VIEW_WINDOW_DISTANCE)
    # To world coord.
    __xwd_o_ns, __ywd_o_ns = pix_to_world(__xpix_o_ns, __ypix_o_ns, __x, __y, __yaw, __world_size, __scale)
    __xwd_n_ns, __ywd_n_ns = pix_to_world(__xpix_n_ns, __ypix_n_ns, __x, __y, __yaw, __world_size, __scale)
    __xwd_n_ns2, __ywd_n_ns2 = pix_to_world(__xpix_n_ns2, __ypix_n_ns2, __x, __y, __yaw, __world_size, __scale)
    __xwd_r, __ywd_r = pix_to_world(__xpix_r, __ypix_r, __x, __y, __yaw, __world_size, __scale)

    # STEP 7. Update Rover worldmap
    # Count obstacles, navigatable and closer navigatable in temp worldmap channel 0, 1, 2
    rover.temp_worldmap[__ywd_o_ns, __xwd_o_ns, 0] += 1
    rover.temp_worldmap[__ywd_n_ns, __xwd_n_ns, 1] += 1
    rover.temp_worldmap[__ywd_n_ns2, __xwd_n_ns2, 2] += 1
    # Render rock to world map green channel
    rover.worldmap[__ywd_r, __xwd_r, 1] += 10
    # Render obstacles to world map red channel
    __oa_sel = rover.temp_worldmap[:, :, 0] > 0
    __na_sel = rover.temp_worldmap[:, :, 1] > 0
    __na2_sel = rover.temp_worldmap[:, :, 2] > 0
    __oo_sel = rover.temp_worldmap[:, :, 0] > rover.temp_worldmap[:, :, 1]
    rover.worldmap[:, :, 0] = 0
    # Light up all obstacles
    rover.worldmap[__oa_sel, 0] = 255
    # Black out all navigatable
    rover.worldmap[__na_sel, 0] = 0
    # Light up obstacles count larger than navigatable
    rover.worldmap[__oo_sel, 0] = 255
    # Black out closed navigatable
    rover.worldmap[__na2_sel, 0] = 0
    # Render navigatable to workld map blue channel
    rover.worldmap[:, :, 2] = 0
    rover.worldmap[__na_sel, 2] = 255
    rover.worldmap[__oo_sel, 2] = 0
    rover.worldmap[__na2_sel, 2] = 255
    # Clip to 0 - 255
    rover.worldmap = np.clip(rover.worldmap, 0, 255)

    # STEP 8. navigatables to polar and update to rover
    __dist, __angles = to_polar_coords(__xpix_n_ns, __ypix_n_ns)
    rover.nav_dists = __dist
    rover.nav_angles = __angles

    return rover
