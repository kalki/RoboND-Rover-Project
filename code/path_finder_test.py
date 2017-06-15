import numpy as np
from PIL import Image
from matplotlib import pyplot as plt
import path_finder as pf

# TEST FIND PATH

__img = Image.open('../calibration_images/map_bw.png')
img = np.array(__img)

start = (110, 155)
goal = (55, 95)

path = pf.search_path(img, start, goal)
print(path)

for ele in path:
    if img[ele[1]:ele[1] + 1, ele[0]:ele[0] + 1] == 0:
        img[ele[1]:ele[1] + 1, ele[0]:ele[0] + 1] = 64
    else:
        img[ele[1]:ele[1] + 1, ele[0]:ele[0] + 1] = 127

# TEST REDUCE PATH

img1 = np.array(__img)
path1 = pf.reduce_path(path, img1)
for ele in path1:
    img1[ele[1]:ele[1] + 1, ele[0]:ele[0] + 1] = 192

# TEST OUTSIDE POINTS

img2 = np.zeros_like(img)
img2[80:110, 40:70] = 1

p = pf.get_fisrt_outside_point(path, img2)

print(p)

# TEST LINE

img3 = np.zeros((200, 200))

img3[135:165, 60:140] = 127

lines = [
    pf.get_line_from_points(10, 10, 90, 10),
    pf.get_line_from_points(10, 10, 90, 40),
    pf.get_line_from_points(10, 10, 90, 90),
    pf.get_line_from_points(10, 10, 40, 90),
    pf.get_line_from_points(10, 10, 10, 90),
    pf.get_line_from_points(91, 91, 10, 91),
    pf.get_line_from_points(91, 91, 91, 10),
    pf.get_line_from_points(190, 10, 110, 10),
    pf.get_line_from_points(190, 10, 110, 40),
    pf.get_line_from_points(190, 10, 110, 90),
    pf.get_line_from_points(190, 10, 150, 90),
    pf.get_line_from_points(190, 10, 190, 90),
    pf.get_line_from_points(50, 130, 150, 130),
    pf.get_line_from_points(80, 140, 120, 140),
    pf.get_line_from_points(50, 150, 150, 150),
    pf.get_line_from_points(80, 160, 120, 160),
    pf.get_line_from_points(50, 170, 150, 170),
]

print(pf.is_point_to_point_cover_by_img((50, 130), (150, 130), img3))
print(pf.is_point_to_point_cover_by_img((80, 140), (120, 140), img3))
print(pf.is_point_to_point_cover_by_img((50, 150), (150, 150), img3))
print(pf.is_point_to_point_cover_by_img((80, 160), (120, 160), img3))
print(pf.is_point_to_point_cover_by_img((50, 170), (150, 170), img3))

for line in lines:
    for ele in line:
        img3[ele[1]:ele[1] + 1, ele[0]:ele[0] + 1] = 255

# VISUALIZE

plt.figure()
plt.subplot(221)
plt.imshow(img, cmap='gray')
plt.subplot(222)
plt.imshow(img1, cmap='gray')
plt.subplot(223)
plt.imshow(img3, cmap='gray')

plt.show()

