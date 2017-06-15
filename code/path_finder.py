import numpy as np
import math


def search_path(image, start, goal):
    __start = (int(start[0]), int(start[1]))
    __goal = (int(goal[0]), int(goal[1]))
    __closed_set = set()
    __open_set = set()
    __came_from = dict()

    __open_set.add(__start)

    __y, __x = image.nonzero()
    __list = np.stack((__x, __y), axis=-1)

    __g_score = dict()
    __f_score = dict()

    for __e in map(tuple, __list):
        __g_score[__e] = 1000.0
        __f_score[__e] = 1000.0

    if __start not in __g_score or __goal not in __g_score:
        return []

    __g_score[__start] = 0.0
    __f_score[__start] = __esti_dist(__start, __goal)

    while len(__open_set) > 0:
        __current = __get_smallest_ele(__open_set, __f_score)
        if __current == __goal:
            __path = __reconstruct_path(__came_from, __current)
            # __path = __extend_path(__path, __g_score)
            return __path

        __open_set.remove(__current)
        __closed_set.add(__current)

        __neighbours = __get_neighbouts(__current, __g_score)

        for __neighbour in __neighbours:
            if __neighbour in __closed_set:
                continue

            if __neighbour not in __open_set:
                __open_set.add(__neighbour)

            __nn = len(__get_neighbouts(__neighbour, __g_score))

            __nn_rate = 1

            if __nn < 8:
                __nn_rate = (8 - __nn) * 2

            __score = __g_score[__current] + __dist(__current, __neighbour) * __nn_rate

            if __score > __g_score[__neighbour]:
                continue

            __came_from[__neighbour] = __current
            __g_score[__neighbour] = __score
            __f_score[__neighbour] = __g_score[__neighbour] + __esti_dist(__neighbour, __goal)

    return []


def get_fisrt_outside_point(path, img):

    __path = path
    if len(__path) <= 0:
        return None

    __img = img
    __y, __x = __img.nonzero()
    __list = np.stack((__x, __y), axis=-1)
    __img_points = set()
    for __e in map(tuple, __list):
        __img_points.add(__e)

    for __ele in __path:
        if __ele not in __img_points:
            return __ele
    return None


def reduce_path(path, img):
    __goal = path[0]
    __path = [__goal]

    __index = 0
    __test1 = __goal
    while __index + 1 < len(path):
        __test2 = path[__index + 1]
        if is_point_to_point_cover_by_img(__test1, __test2, img):
            __index += 1
        else:
            __path.append(path[__index])
            __test1 = path[__index]

    # Do not add start point
    return __path


def is_point_to_point_cover_by_img(p1, p2, img):
    __img = img
    __y, __x = __img.nonzero()
    __list = np.stack((__x, __y), axis=-1)
    __img_points = set()
    for __e in map(tuple, __list):
        __img_points.add(__e)

    if p1 not in __img_points:
        return False
    if p2 not in __img_points:
        return False

    __lines = get_line_from_points(p1[0], p1[1], p2[0], p2[1])

    for __ele in __lines:
        if __ele not in __img_points:
            return False

    return True


def get_line_from_points(x1, y1, x2, y2):
    __x1 = int(x1)
    __x2 = int(x2)
    __y1 = int(y1)
    __y2 = int(y2)
    __dx = __x2 - __x1
    __dy = __y2 - __y1

    if __dx == 0:
        __inc_x = 0
    else:
        __inc_x = int(__dx / np.abs(__dx))

    if __dy == 0:
        __inc_y = 0
    else:
        __inc_y = int(__dy / np.abs(__dy))

    if __inc_x == 0 and __inc_y == 0:
        __p = (__x1, __y1)
        __line = [__p]
        return __line

    __current_x = __x1
    __current_y = __y1
    __line = list()
    if np.abs(__dx) > np.abs(__dy):
        __count = 0
        __max = np.abs(__dx)
        while __count <= __max:
            __p = (__current_x, __current_y)
            __line.append(__p)
            __current_x += __inc_x
            __diff_x = __current_x - x1
            __diff_y = __diff_x * float(__dy) / float(__dx)
            __current_y = int(__y1 + __diff_y)
            __count += 1
    else:
        __count = 0
        __max = np.abs(__dy)
        while __count <= __max:
            __p = (__current_x, __current_y)
            __line.append(__p)
            __current_y += __inc_y
            __diff_y = __current_y - y1
            __diff_x = __diff_y * float(__dx) / float(__dy)
            __current_x = int(__x1 + __diff_x)
            __count += 1
    return __line


def __extend_path(path, image):
    __new_path = []
    for ind in range(0, len(path)-1):
        __current = path[ind]
        __next = path[ind+1]
        if __dist(__current, __next) > 1:
            __ext1 = (__current[0], __next[1])
            __ext2 = (__next[0], __current[1])
            __new_path.append(__current)
            if __ext1 in image:
                __new_path.append(__ext1)
            if __ext2 in image:
                __new_path.append(__ext2)
        else:
            __new_path.append(__current)
    __new_path.append(path[len(path)-1])
    return __new_path


def __esti_dist(start, goal):
    return __dist(start, goal)


def __dist(start, goal):
    return math.sqrt((start[0] - goal[0]) ** 2.0 + (start[1] - goal[1]) ** 2.0)


def __get_smallest_ele(key_set, value_map):
    smallest_value = 9999.0
    smallest_key = None
    for key in key_set:
        if key in value_map:
            if value_map[key] < smallest_value:
                smallest_value = value_map[key]
                smallest_key = key
    return smallest_key


def __get_neighbouts(current, score_map):
    __list = [
        (current[0] - 1, current[1] - 1),
        (current[0] - 1, current[1]),
        (current[0] - 1, current[1] + 1),
        (current[0], current[1] - 1),
        (current[0], current[1] + 1),
        (current[0] + 1, current[1] - 1),
        (current[0] + 1, current[1]),
        (current[0] + 1, current[1] + 1)
    ]

    __list1 = []
    for __ele in __list:
        if __ele in score_map:
            __list1.append(__ele)

    return __list1


def __reconstruct_path(route, current):
    __current = current
    __path = [__current]

    while __current in route:
        __current = route[__current]
        __path.append(__current)

    return __path
