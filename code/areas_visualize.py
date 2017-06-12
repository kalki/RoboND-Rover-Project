import numpy as np
from matplotlib import pyplot as plt

import navigatable_area as nav
import rock_locator as ro


img1 = np.zeros((160, 320))
nav.render_image(img1, 127, 0, 255)

img2 = np.zeros((160, 320))
img3 = np.zeros((160, 320))
img3[130:131, 145:146] = 1

ro.render_rock(img2, img3, 127, 0, 255, 1)
ro.render_image(img2, 127, 0, 255)

plt.figure()
plt.subplot(211)
plt.imshow(img1, cmap='gray')
plt.subplot(212)
plt.imshow(img2, cmap='gray')

plt.show()
