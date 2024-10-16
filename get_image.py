from pyk4a import PyK4A

k4a = PyK4A()
k4a.start()

# Capture color image
capture = k4a.get_capture()
img_color = capture.color
from matplotlib import pyplot as plt
plt.imshow(img_color[:, :, 2::-1])

#save the image with updated name
plt.imsave('front_bottom_right.png', img_color[:, :, 2::-1])
plt.show()