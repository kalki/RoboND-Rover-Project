import numpy as np

from rectangle import Rectangle

# Left area ahead of rover with some space
L_IMPACT = Rectangle(151, 135, 159, 154)
# Left area ahead of impact area, to detect front edge
L_FRONT_CLOSE = Rectangle(151, 125, 159, 134)
L_FRONT_FAR = Rectangle(151, 115, 159, 124)
# Area left to impact area, to follow edge
L_EDGE_CLOSE = Rectangle(141, 135, 150, 144)
L_EDGE_FAR = Rectangle(141, 125, 150, 134)
# Area left to left edge area, no in use now
L_GUIDE = Rectangle(131, 125, 140, 134)
# Right area ahead of rover with some space
R_IMPACT = Rectangle(160, 135, 169, 154)
# Right area ahead of impact area, to detect front edge
R_FRONT_CLOSE = Rectangle(160, 125, 169, 134)
R_FRONT_FAR = Rectangle(160, 115, 169, 124)
# Area right to impact area, to follow edge
R_EDGE_CLOSE = Rectangle(170, 135, 179, 144)
R_EDGE_FAR = Rectangle(170, 125, 179, 134)
# Area right to right follow area, not in use now
R_GUIDE = Rectangle(180, 125, 189, 134)

L_IMPACT_NAME = "l_impact"
L_FRONT_CLOSE_NAME = "l_front_close"
L_FRONT_FAR_NAME = "l_front_far"
L_EDGE_CLOSE_NAME = "l_edge_close"
L_EDGE_FAR_NAME = "l_edge_far"
L_GUIDE_NAME = "l_guide"
R_IMPACT_NAME = "r_impact"
R_FRONT_CLOSE_NAME = "r_front_close"
R_FRONT_FAR_NAME = "r_front_far"
R_EDGE_CLOSE_NAME = "r_edge_close"
R_EDGE_FAR_NAME = "r_edge_far"
R_GUIDE_NAME = "r_guide"


AREAS = {
    L_IMPACT_NAME: L_IMPACT,
    L_FRONT_CLOSE_NAME: L_FRONT_CLOSE,
    L_FRONT_FAR_NAME: L_FRONT_FAR,
    L_EDGE_CLOSE_NAME: L_EDGE_CLOSE,
    L_EDGE_FAR_NAME: L_EDGE_FAR,
    L_GUIDE_NAME: L_GUIDE,
    R_IMPACT_NAME: R_IMPACT,
    R_FRONT_CLOSE_NAME: R_FRONT_CLOSE,
    R_FRONT_FAR_NAME: R_FRONT_FAR,
    R_EDGE_CLOSE_NAME: R_EDGE_CLOSE,
    R_EDGE_FAR_NAME: R_EDGE_FAR,
    R_GUIDE_NAME: R_GUIDE
}

# BLOCKED | NORMAL | OPEN | CLEAR
BLOCKED_THRESHOLD = 30
OPEN_THRESHOLD = 70
CLEAR_THRESHOLD = 95


# Render area on input image, just for visualize the area definition
def render_image(img, max_increment, min_value=0, max_value=255):
    # Different area
    inc1 = max_increment
    inc2 = max_increment / 2

    __render_image(img, L_IMPACT, inc1)
    __render_image(img, L_FRONT_CLOSE, inc2)
    __render_image(img, L_FRONT_FAR, inc1)
    __render_image(img, L_EDGE_CLOSE, inc2)
    __render_image(img, L_EDGE_FAR, inc1)
    __render_image(img, L_GUIDE, inc2)
    __render_image(img, R_IMPACT, inc2)
    __render_image(img, R_FRONT_CLOSE, inc1)
    __render_image(img, R_FRONT_FAR, inc2)
    __render_image(img, R_EDGE_CLOSE, inc1)
    __render_image(img, R_EDGE_FAR, inc2)
    __render_image(img, R_GUIDE, inc1)

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
    img[__area.y1:__area.y2+1, __area.x1:__area.x2+1] += increment
    return 0


# Calculate non zero pixel percent of specified area in input image
def nonzero_percent(img, area):
    __area = area
    if not __area.is_contained(img.shape):
        return -1
    return len(img[__area.y1:__area.y2 + 1, __area.x1:__area.x2 + 1].nonzero()[0]) * 100 / __area.prod


class NavigationArea(object):

    def __init__(self, img):
        self.img = img

    def is_area_clear(self, area_name):
        __area = AREAS[area_name]
        __p = nonzero_percent(self.img, __area)
        if __p > CLEAR_THRESHOLD:
            return True
        else:
            return False

    def is_area_open(self, area_name):
        __area = AREAS[area_name]
        __p = nonzero_percent(self.img, __area)
        if __p > OPEN_THRESHOLD:
            return True
        else:
            return False

    def is_area_normal(self, area_name):
        __area = AREAS[area_name]
        __p = nonzero_percent(self.img, __area)
        if OPEN_THRESHOLD >= __p > BLOCKED_THRESHOLD:
            return True
        else:
            return False

    def is_area_blocked(self, area_name):
        __area = AREAS[area_name]
        __p = nonzero_percent(self.img, __area)
        if __p <= BLOCKED_THRESHOLD:
            return True
        else:
            return False
