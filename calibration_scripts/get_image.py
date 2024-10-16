from pyk4a import PyK4A

k4a = PyK4A()
k4a.start()

# Capture color image
capture = k4a.get_capture()
img_color = capture.color
from matplotlib import pyplot as plt
plt.imshow(img_color[:, :, 2::-1])

# Save the image with defined name
plt.imsave('test_image.png', img_color[:, :, 2::-1])
plt.show()