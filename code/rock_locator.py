import numpy as np

from rectangle import Rectangle

# Area definition to discover rock
# Area is only at left side of rover, to prevent pick rock at right side and brake current forward path
# Area is only close area and will not detect rock too far
# Some rock will be missed if it is in the middle of open ground, because rover is always follow left edge
L_FRONT = Rectangle(150, 135, 164, 159)
L_EDGE_CLOSE = Rectangle(140, 130, 149, 159)
L_GUIDE = Rectangle(130, 125, 139, 159)


# Functions to work with rock input image


# Function to put closet rock in rock image on image, with specified rock width.
# Rock in rock image usually looks like a line, and only closest pixel will be consider as rock position
def render_rock(img, rock_img, max_increment, min_value=0, max_value=255, rock_size=1):
    __inc = max_increment
    __width = int(rock_size / 2)

    if is_rock_insight(rock_img):
        __x, __y = get_closest_rock(rock_img)
        __rock = Rectangle(__x - __width, __y - __width, __x + __width, __y + __width)
        __render_image(img, __rock, __inc)

    if min_value is not None and max_value is not None:
        np.clip(img, min_value, max_value, out=img)

    elif min_value is not None:
        __max = np.amax(img)
        np.clip(img, min_value, __max, out=img)

    elif max_value is not None:
        __min = np.amin(img)
        np.clip(img, __min, max_value, out=img)


# Render area definition on image, just for visualize area definition
def render_image(img, max_increment, min_value=0, max_value=255):
    __inc = max_increment

    __render_image(img, L_FRONT, __inc)
    __render_image(img, L_EDGE_CLOSE, __inc)
    __render_image(img, L_GUIDE, __inc)

    if min_value is not None and max_value is not None:
        np.clip(img, min_value, max_value, out=img)

    elif min_value is not None:
        __max = np.amax(img)
        np.clip(img, min_value, __max, out=img)

    elif max_value is not None:
        __min = np.amin(img)
        np.clip(img, __min, max_value, out=img)


def __render_image(img, area, increment):
    __area = area
    if not __area.is_contained(img.shape):
        return -1
    img[int(__area.y1):int(__area.y2+1), int(__area.x1):int(__area.x2+1)] += increment
    return 0


def __to_polar_coords(x_pixel, y_pixel):
    __dist = np.sqrt(x_pixel ** 2 + y_pixel ** 2)
    __angles = np.arctan2(y_pixel, x_pixel)
    return __dist, __angles


def __from_polar_coords(dist, angle):
    __x = dist * np.cos(angle)
    __y = dist * np.sin(angle)
    return __x, __y


# Get all non zero pixels in defined area in input imege
def __get_rock_pixel(img):
    __y1, __x1 = img[L_FRONT.y1:L_FRONT.y2 + 1, L_FRONT.x1:L_FRONT.x2 + 1].nonzero()
    __y1 += L_FRONT.y1
    __x1 += L_FRONT.x1
    __y2, __x2 = img[L_EDGE_CLOSE.y1:L_EDGE_CLOSE.y2 + 1, L_EDGE_CLOSE.x1:L_EDGE_CLOSE.x2 + 1].nonzero()
    __y2 += L_EDGE_CLOSE.y1
    __x2 += L_EDGE_CLOSE.x1
    __y3, __x3 = img[L_GUIDE.y1:L_GUIDE.y2 + 1, L_GUIDE.x1:L_GUIDE.x2 + 1].nonzero()
    __y3 += L_GUIDE.y1
    __x3 += L_GUIDE.x1
    __x = np.concatenate((__x1, __x2, __x3))
    __y = np.concatenate((__y1, __y2, __y3))
    return __x, __y


# Check if there is rock in defined area in input image
def is_rock_insight(img):
    __x, __y = __get_rock_pixel(img)

    if len(__x) > 0:
        return True
    else:
        return False


# Get distance and angle of closest rock pixel to rover in rock image
def get_closest_rock_polar(img):
    __x, __y = __get_rock_pixel(img)
    if len(__x) > 0:
        __x_pixel = -(__y - img.shape[0]).astype(np.float)
        __y_pixel = -(__x - img.shape[1] / 2).astype(np.float)

        __d, __a = __to_polar_coords(__x_pixel, __y_pixel)

        # Sort by distance and get closest one
        __ind = np.argsort(__d)
        __closest_d = __d[__ind[0]]
        __closest_a = __a[__ind[0]]
        return __closest_d, __closest_a


# Get position of closest rock pixel to rover in rock image
def get_closest_rock(img):
    __d, __a = get_closest_rock_polar(img)
    __x, __y = __from_polar_coords(__d, __a)

    __x_r = img.shape[1]/2 - __y
    __y_r = img.shape[0] - __x

    return __x_r, __y_r
