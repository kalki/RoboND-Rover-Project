import numpy as np

from rectangle import Rectangle

# Left area ahead of rover with some space, no in use now
L_IMPACT = Rectangle(150, 135, 159, 154)
# Left area ahead of impact area, to detect front edge
L_FRONT_CLOSE = Rectangle(150, 125, 159, 134)
L_FRONT_FAR = Rectangle(150, 105, 159, 124)
# Area left to impact area, to follow edge
L_EDGE_CLOSE = Rectangle(140, 135, 149, 144)
L_EDGE_FAR = Rectangle(140, 125, 149, 134)
# Area left to left edge area, no in use now
L_GUIDE = Rectangle(130, 125, 139, 134)
# Right area ahead of rover with some space, no in use now
R_IMPACT = Rectangle(160, 135, 169, 154)
# Right area ahead of impact area, to detect front edge
R_FRONT_CLOSE = Rectangle(160, 125, 169, 134)
R_FRONT_FAR = Rectangle(160, 105, 169, 124)
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


def render_image(img, area, increment, min_value=0, max_value=255):
    __area = area
    if not __area.is_contained(img.shape):
        return -1

    img[__area.y1:__area.y2+1, __area.x1:__area.x2+1] += increment

    if min_value is not None and max_value is not None:
        np.clip(img, min_value, max_value, out=img)

    elif min_value is not None:
        __max = np.amax(img)
        np.clip(img, min_value, __max, out=img)

    elif max_value is not None:
        __min = np.amin(img)
        np.clip(img, __min, max_value, out=img)

    return 0


def navable_percent(img, area):
    __area = area
    if not __area.is_contained(img.shape):
        return -1
    return len(img[__area.y1:__area.y2 + 1, __area.x1:__area.x2 + 1].nonzero()[0]) * 100 / __area.prod


def is_left_edge_clear(img):
    __lec = L_EDGE_CLOSE
    __lec_p = navable_percent(img, __lec)
    __lef = L_EDGE_FAR
    __lef_p = navable_percent(img, __lef)
    if __lec_p > CLEAR_THRESHOLD and __lef_p > CLEAR_THRESHOLD:
        return True
    else:
        return False


def is_left_edge_open(img):
    __lec = L_EDGE_CLOSE
    __lec_p = navable_percent(img, __lec)
    __lef = L_EDGE_FAR
    __lef_p = navable_percent(img, __lef)
    if __lec_p > OPEN_THRESHOLD or __lef_p > OPEN_THRESHOLD:
        return True
    else:
        return False


def is_left_edge_left(img):
    __lec = L_EDGE_CLOSE
    __lec_p = navable_percent(img, __lec)
    __lef = L_EDGE_FAR
    __lef_p = navable_percent(img, __lef)
    if __lec_p <= OPEN_THRESHOLD < __lef_p:
        return True
    else:
        return False


def is_left_edge_normal(img):
    __lef = L_EDGE_FAR
    __lef_p = navable_percent(img, __lef)
    if BLOCKED_THRESHOLD <= __lef_p <= OPEN_THRESHOLD:
        return True
    else:
        return False


def is_left_edge_blocked(img):
    __lef = L_EDGE_FAR
    __lef_p = navable_percent(img, __lef)
    if __lef_p < BLOCKED_THRESHOLD:
        return True
    else:
        return False


def is_right_edge_clear(img):
    __rec = R_EDGE_CLOSE
    __rec_p = navable_percent(img, __rec)
    __ref = R_EDGE_FAR
    __ref_p = navable_percent(img, __ref)
    if __rec_p > CLEAR_THRESHOLD and __ref_p > CLEAR_THRESHOLD:
        return True
    else:
        return False


def is_right_edge_open(img):
    __rec = R_EDGE_CLOSE
    __rec_p = navable_percent(img, __rec)
    __ref = R_EDGE_FAR
    __ref_p = navable_percent(img, __ref)
    if __rec_p > OPEN_THRESHOLD or __ref_p > OPEN_THRESHOLD:
        return True
    else:
        return False


def is_right_edge_left(img):
    __rec = R_EDGE_CLOSE
    __rec_p = navable_percent(img, __rec)
    __ref = R_EDGE_FAR
    __ref_p = navable_percent(img, __ref)
    if __rec_p <= OPEN_THRESHOLD < __ref_p:
        return True
    else:
        return False


def is_right_edge_normal(img):
    __ref = R_EDGE_FAR
    __ref_p = navable_percent(img, __ref)
    if BLOCKED_THRESHOLD <= __ref_p <= OPEN_THRESHOLD:
        return True
    else:
        return False


def is_right_edge_blocked(img):
    __ref = R_EDGE_FAR
    __ref_p = navable_percent(img, __ref)
    if __ref_p < BLOCKED_THRESHOLD:
        return True
    else:
        return False


def is_right_front_blocked(img):
    __rf = R_FRONT_CLOSE
    __rf_p = navable_percent(img, __rf)
    if __rf_p < BLOCKED_THRESHOLD:
        return True
    else:
        return False


def is_left_front_blocked(img):
    __lf = R_FRONT_CLOSE
    __lf_p = navable_percent(img, __lf)
    if __lf_p < BLOCKED_THRESHOLD:
        return True
    else:
        return False


def is_front_area_blocked(img):
    __lf = L_FRONT_CLOSE
    __rf = R_FRONT_CLOSE
    __lf_p = navable_percent(img, __lf)
    __rf_p = navable_percent(img, __rf)
    if __lf_p < BLOCKED_THRESHOLD and __rf_p < BLOCKED_THRESHOLD:
        return True
    else:
        return False


def is_farfront_area_blocked(img):
    __lf = L_FRONT_FAR
    __rf = R_FRONT_FAR
    __lf_p = navable_percent(img, __lf)
    __rf_p = navable_percent(img, __rf)
    if __lf_p < BLOCKED_THRESHOLD and __rf_p < BLOCKED_THRESHOLD:
        return True
    else:
        return False


def is_farfront_area_open(img):
    __lf = L_FRONT_FAR
    __rf = R_FRONT_FAR
    __lf_p = navable_percent(img, __lf)
    __rf_p = navable_percent(img, __rf)
    if __lf_p > OPEN_THRESHOLD and __rf_p > OPEN_THRESHOLD:
        return True
    else:
        return False


def is_left_guide_open(img):
    __lg = L_GUIDE
    __lg_p = navable_percent(img, __lg)
    if __lg_p > OPEN_THRESHOLD:
        return True
    else:
        return False


def is_left_guide_clear(img):
    __lg = L_GUIDE
    __lg_p = navable_percent(img, __lg)
    if __lg_p > CLEAR_THRESHOLD:
        return True
    else:
        return False


class NavigationArea(object):

    def __init__(self, img):
        self.img = img

    def is_area_clear(self, area_name):
        __area = AREAS[area_name]
        __p = navable_percent(self.img, __area)
        if __p > CLEAR_THRESHOLD:
            return True
        else:
            return False

    def is_area_open(self, area_name):
        __area = AREAS[area_name]
        __p = navable_percent(self.img, __area)
        if __p > OPEN_THRESHOLD:
            return True
        else:
            return False

    def is_area_normal(self, area_name):
        __area = AREAS[area_name]
        __p = navable_percent(self.img, __area)
        if OPEN_THRESHOLD >= __p > BLOCKED_THRESHOLD:
            return True
        else:
            return False

    def is_area_blocked(self, area_name):
        __area = AREAS[area_name]
        __p = navable_percent(self.img, __area)
        if __p <= BLOCKED_THRESHOLD:
            return True
        else:
            return False

