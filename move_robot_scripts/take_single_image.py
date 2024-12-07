from pyk4a import PyK4A, Config, ImageFormat, DepthMode, ColorResolution
import cv2 
import numpy as np

def take_image():
    config = Config(color_resolution = ColorResolution.RES_2160P, depth_mode = DepthMode.WFOV_2X2BINNED)
    k4a = PyK4A(config=config)
    k4a.start() 
   
    capture = k4a.get_capture()
    img_bgr = capture.color[:,:, :3]
    img_d = capture.transformed_depth
    img_bgr = np.ascontiguousarray(img_bgr)
    rgb_image_path = "rgb.jpg"
    d_image_path = "depth.png"
    cv2.imwrite(rgb_image_path, img_bgr) 
    cv2.imwrite(d_image_path, img_d)

if __name__ == '__main__':
    take_image()