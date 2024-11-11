from pyk4a import PyK4A
from matplotlib import pyplot as plt

def capture_images(output_dir, path):
    k4a = PyK4A()
    k4a.start()

    # Capture color image
    capture = k4a.get_capture()
    img_color = capture.color

    # Save the image with defined name
    plt.imsave(output_dir + path + '_color.png', img_color[:, :, 2::-1])
    plt.imsave(output_dir + path + '_depth.png', img_color[:, :, 2::-1])
