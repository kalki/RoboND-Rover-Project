import numpy as np
from matplotlib import pyplot as plt

import navigatable_area as nav


img = np.zeros((160, 320))
nav.render_image(img, nav.L_IMPACT, 127)
nav.render_image(img, nav.R_IMPACT, 63)
nav.render_image(img, nav.L_FRONT_CLOSE, 63)
nav.render_image(img, nav.R_FRONT_CLOSE, 127)
nav.render_image(img, nav.L_FRONT_FAR, 127)
nav.render_image(img, nav.R_FRONT_FAR, 63)
nav.render_image(img, nav.L_EDGE_CLOSE, 63)
nav.render_image(img, nav.L_EDGE_FAR, 127)
nav.render_image(img, nav.L_GUIDE, 63)
nav.render_image(img, nav.R_EDGE_CLOSE, 127)
nav.render_image(img, nav.R_EDGE_FAR, 63)
nav.render_image(img, nav.R_GUIDE, 127)

plt.imshow(img, cmap='gray')
plt.show()
