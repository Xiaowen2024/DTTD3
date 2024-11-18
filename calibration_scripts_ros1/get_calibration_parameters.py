#!/usr/bin/env python

import numpy as np
import rospy
import tf
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from pyk4a import PyK4A, Config, ImageFormat, DepthMode
import cv2
import matplotlib.pyplot as plt
import time
import tf2_ros
from scipy.spatial.transform import Rotation as R
import json


class CameraCalibration:
    @staticmethod
    def save_color_camera_matrix(calibration):
        camera_matrix = calibration.get_camera_matrix(1)
        with open('updated_calibration_color_camera_matrix.txt', 'w') as f:
            np.savetxt(f, camera_matrix, fmt='%.6f')

    @staticmethod
    def get_color_camera_matrix(calibration):
        return calibration.get_camera_matrix(1)

    @staticmethod
    def save_color_dist_coefficients(calibration):
        dist_coeffs = calibration.get_distortion_coefficients(1)
        with open('updated_calibration_color_camera_dist_coeffs.txt', 'w') as f:
            np.savetxt(f, dist_coeffs, fmt='%.6f')

    @staticmethod
    def get_color_dist_coefficients(calibration):
        return calibration.get_distortion_coefficients(1)
    
    @staticmethod
    def estimate_pose_single_markers(corners, camera_matrix, dist_coeffs, marker_size):
        marker_points = np.array([[-marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, marker_size / 2, 0],
                                [marker_size / 2, -marker_size / 2, 0],
                                [-marker_size / 2, -marker_size / 2, 0]], dtype=np.float32)
        rvecs = []
        tvecs = []
        for c in corners:
            _, R, t = cv2.solvePnP(marker_points, c, camera_matrix, dist_coeffs, False, cv2.SOLVEPNP_IPPE_SQUARE)
            rvecs.append(R)
            tvecs.append(t)
        return np.array(rvecs), np.array(tvecs)
    
    @staticmethod
    def get_target_camera_transform(image_path, camera_matrix, dist_coeffs, marker_length):
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        parameters = cv2.aruco.DetectorParameters()
        frame = cv2.imread(image_path)
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # Detect ArUco markers in the image
        corners, ids, _ = detector.detectMarkers(frame)

    
        if ids is not None:
            # Draw detected markers for visualization
            frame_markers = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
            # retval, rvec, tvec = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length,  camera_matrix, dist_coeffs)
            # Estimate pose of each marker
            rvecs, tvecs = CameraCalibration.estimate_pose_single_markers(corners, camera_matrix, dist_coeffs, marker_length)
            tvec = tvecs[0]
            rvec = rvecs[0]
            R_target2cam, _ = cv2.Rodrigues(rvec)
            print(f"R_target2Cam: {R_target2cam}")
            print(f"T_target2Cam: {tvec}") 
            
            # Save R_target2cam
            # Load and update rotations
            # Handle rotations
            try:
                with open('calibration_ready_parameters/calibration_target2cam_rotations.json', 'r') as f:
                    data = json.load(f)
                    rotations = np.array(data['rotations'])
                    # Add new rotation
                    rotations = np.vstack((rotations.reshape(-1, 3, 3),
                                         R_target2cam.reshape(1, 3, 3)))
            except (OSError, IOError, ValueError):
                # Initialize new rotations array if file doesn't exist
                rotations = R_target2cam.reshape(1, 3, 3)

            # Save updated rotations
            with open('calibration_ready_parameters/calibration_target2cam_rotations.json', 'w') as f:
                json.dump({
                    'rotations': rotations.tolist()
                }, f, indent=2)

            # Handle translations
            try:
                with open('calibration_ready_parameters/calibration_target2cam_translations.json', 'r') as f:
                    data = json.load(f)
                    translations = np.array(data['translations'])
                    # Add new translation
                    translations = np.vstack((translations.reshape(-1, 3),
                                           tvec.reshape(1, 3)))
            except (OSError, IOError, ValueError):
                # Initialize new translations array if file doesn't exist
                translations = tvec.reshape(1, 3)

            # Save updated translations
            with open('calibration_ready_parameters/calibration_target2cam_translations.json', 'w') as f:
                json.dump({
                    'translations': translations.tolist()
                }, f, indent=2)

            return R_target2cam, tvec

class TF2Echo:
    def __init__(self):
        # Initialize the ROS node
        # rospy.init_node('tf2_echo_node', anonymous=True)
        
        # Create a tf2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.target_frame = 'lbr_link_0'  # The target frame in which you want to get the transform
        self.source_frame = 'lbr_link_ee'  # The source frame from which you want the transform
        
        self.rotation = None
        self.translation = None
        self.transform_received = False
    
    def listen_for_transform(self, transform_saved = False):
        while not rospy.is_shutdown() and not transform_saved:
            try:
                # Lookup the transform from source_frame to target_frame
                transform = self.tf_buffer.lookup_transform(self.target_frame, self.source_frame, rospy.Time(0))
                
                self.translation = np.array([
                    transform.transform.translation.x,
                    transform.transform.translation.y,
                    transform.transform.translation.z
                ])
                self.rotation = self.quaternion_to_rotation_matrix(
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z,
                    transform.transform.rotation.w
                )
                print(f"rotation: {self.rotation}")
                print(f"translation: {self.translation}")
                transform_saved = True;
                self.save_transform(transform)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr(f'Could not get transform: {str(e)}')
            time.sleep(0.1)  # Wait briefly before trying again

    def save_transform(self, transform):
        try:
            translation = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z
            ])
            rotation = self.quaternion_to_rotation_matrix(
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w
            )

            # Save transform data as JSON
            transform_data = {
                'rotation': rotation.tolist(),
                'translation': translation.tolist()
            }
            # Save transform data to JSON file
            # Handle rotations
            # try:
            #     with open('11_17_parameters/calibration_gripper2base_rotations.json', 'r') as f:
            #         data = json.load(f)
            #         rotations = np.array(data['rotations'])
            #         if len(rotations.shape) == 2:
            #             rotations = rotations.reshape(-1, 3, 3)
            #         rotations = np.vstack((rotations,
            #                              rotation.reshape(1, 3, 3)))
            # except (OSError, IOError, ValueError):
            #     rotations = rotation.reshape(1, 3, 3)

            # # Save rotations
            # with open('11_17_parameters/calibration_gripper2base_rotations.json', 'w') as f:
            #     json.dump({
            #         'rotations': rotations.tolist()
            #     }, f, indent=2)

            # # Handle translations
            # try:
            #     with open('11_17_parameters/calibration_gripper2base_translations.json', 'r') as f:
            #         data = json.load(f)
            #         translations = np.array(data['translations'])
            #         # Ensure translations is 2D array with shape (-1, 3) before vstack
            #         if len(translations.shape) == 1:
            #             translations = translations.reshape(-1, 3)
            #         translations = np.vstack((translations,
            #                                translation.reshape(1, 3)))
            # except (OSError, IOError, ValueError):
            #     translations = translation.reshape(1, 3)

            # # Save translations
            # with open('11_17_parameters/calibration_gripper2base_translations.json', 'w') as f:
            #     json.dump({
            #         'translations': translations.tolist()
            #     }, f, indent=2)

            try:
                with open('calibration_ready_parameters/calibration_gripper2base_rotations.json', 'r') as f:
                    data = json.load(f)
                    rotations = np.array(data['rotations'])
                    # Add new rotation
                    rotations = np.vstack((rotations.reshape(-1, 3, 3),
                                         rotation.reshape(1, 3, 3)))
            except (OSError, IOError, ValueError):
                # Initialize new rotations array if file doesn't exist
                rotations = rotation.reshape(1, 3, 3)

            # Save updated rotations
            with open('calibration_ready_parameters/calibration_gripper2base_rotations.json', 'w') as f:
                json.dump({
                    'rotations': rotations.tolist()
                }, f, indent=2)

            # Handle translations
            try:
                with open('calibration_ready_parameters/calibration_gripper2base_translations.json', 'r') as f:
                    data = json.load(f)
                    translations = np.array(data['translations'])
                    # Add new translation
                    translations = np.vstack((translations.reshape(-1, 3),
                                           translation.reshape(1, 3)))
            except (OSError, IOError, ValueError):
                # Initialize new translations array if file doesn't exist
                translations = translation.reshape(1, 3)

            # Save updated translations
            with open('calibration_ready_parameters/calibration_gripper2base_translations.json', 'w') as f:
                json.dump({
                    'translations': translations.tolist()
                }, f, indent=2)

            print("saving gripper2base")

            return rotation, translation
        

         
            
            # Save rotation matrix
        #     try:
        #         existing_rotations = np.loadtxt('updated_calibration_gripper2base_rotations.txt')
        #         existing_rotations = existing_rotations.reshape(-1, 3, 3)
        #         np.savetxt('updated_calibration_gripper2base_rotations.txt',
        #                   np.vstack((existing_rotations, rotation.reshape(1, 3, 3))).reshape(-1, 3),
        #                   fmt='%.6f')
        #     except (OSError, IOError):
        #         np.savetxt('updated_calibration_gripper2base_rotations.txt', rotation, fmt='%.6f')

        #     # Save translation vector
        #     try:
        #         existing_translations = np.loadtxt('updated_calibration_gripper2base_translations.txt')
        #         existing_translations = existing_translations.reshape(-1, 3)
        #         np.savetxt('updated_calibration_gripper2base_translations.txt',
        #                   np.vstack((existing_translations, translation.reshape(1, 3))),
        #                   fmt='%.6f')
        #     except (OSError, IOError):
        #         np.savetxt('updated_calibration_gripper2base_translations.txt', translation.reshape(1, 3), fmt='%.6f')

        #     return rotation, translation
    
          # Save rotation matrix
        #     try:
        #         existing_rotations = np.loadtxt('updated_calibration_gripper2base_rotations.txt')
        #         existing_rotations = existing_rotations.reshape(-1, 3, 3)
        #         np.savetxt('updated_calibration_gripper2base_rotations.txt',
        #                   np.vstack((existing_rotations, rotation.reshape(1, 3, 3))).reshape(-1, 3),
        #                   fmt='%.6f')
        #     except (OSError, IOError):
        #         np.savetxt('updated_calibration_gripper2base_rotations.txt', rotation, fmt='%.6f')

        #     # Save translation vector
        #     try:
        #         existing_translations = np.loadtxt('updated_calibration_gripper2base_translations.txt')
        #         existing_translations = existing_translations.reshape(-1, 3)
        #         np.savetxt('updated_calibration_gripper2base_translations.txt',
        #                   np.vstack((existing_translations, translation.reshape(1, 3))),
        #                   fmt='%.6f')
        #     except (OSError, IOError):
        #         np.savetxt('updated_calibration_gripper2base_translations.txt', translation.reshape(1, 3), fmt='%.6f')

        #     return rotation, translation
        except Exception as e:
            rospy.logerr(f'Could not save transform: {str(e)}')


    # def quaternion_to_rotation_matrix(self, x, y, z, w):
    #     """
    #     Convert a quaternion (x, y, z, w) to a rotation matrix (3x3).
    #     """
    #     R = np.zeros((3, 3))
    #     R[0, 0] = 1 - 2 * (y * y + z * z)
    #     R[0, 1] = 2 * (x * y - z * w)
    #     R[0, 2] = 2 * (x * z + y * w)
    #     R[1, 0] = 2 * (x * y + z * w)
    #     R[1, 1] = 1 - 2 * (x * x + z * z)
    #     R[1, 2] = 2 * (y * z - x * w)
    #     R[2, 0] = 2 * (x * z - y * w)
    #     R[2, 1] = 2 * (y * z + x * w)
    #     R[2, 2] = 1 - 2 * (x * x + y * y)
        
    #     return R

    def quaternion_to_rotation_matrix(self, x, y, z, w):
        return R.from_quat([x, y, z, w]).as_matrix()
    
def initialize():
    config = Config(color_format=ImageFormat.COLOR_MJPG, depth_mode = DepthMode.OFF, synchronized_images_only = False)
    k4a = PyK4A(config=config)
    k4a.start() 
    tf2_echo = TF2Echo()
    return k4a, tf2_echo

    
def close_camera(camera):
    camera.stop()

def get_parameters(index, k4a, tf2_echo):
    # config = Config(color_format=ImageFormat.COLOR_MJPG, depth_mode = DepthMode.OFF, synchronized_images_only = False)
    # k4a = PyK4A(config=config)
    # k4a.start()  
    capture = k4a.get_capture()
    img_color = capture.color
    img_color = cv2.imdecode(img_color, cv2.IMREAD_COLOR)
    image_path = 'calibration_ready_images/calibration' + index + ".jpg"
    plt.imsave(image_path, img_color)
    calibration = k4a.calibration
    camera_matrix = CameraCalibration.get_color_camera_matrix(calibration)
    # print(f"camera_matrix: {camera_matrix}")
    dist_coeffs = CameraCalibration.get_color_dist_coefficients(calibration)
    # print(f"dist_coeffs: {dist_coeffs}")
    tf2_echo.listen_for_transform(False)
    CameraCalibration.get_target_camera_transform(
        image_path,
        camera_matrix,
        dist_coeffs,
        0.2047
    )
 

def main():
    # Initialize the K4A camera
    config = Config(color_format=ImageFormat.COLOR_MJPG, depth_mode = DepthMode.OFF, synchronized_images_only = False)
    k4a = PyK4A(config=config)
    k4a.start()  
    capture = k4a.get_capture()
    img_color = capture.color
    img_color = cv2.imdecode(img_color, cv2.IMREAD_COLOR)
    image_path = 'calibration_ready_images/calibration.jpg'
    plt.imsave(image_path, img_color)
    calibration = k4a.calibration
    camera_matrix = CameraCalibration.get_color_camera_matrix(calibration)
    print(f"camera_matrix: {camera_matrix}")
    dist_coeffs = CameraCalibration.get_color_dist_coefficients(calibration)
    print(f"dist_coeffs: {dist_coeffs}")
    CameraCalibration.get_target_camera_transform(
        image_path,
        camera_matrix,
        dist_coeffs,
        0.2047
    )
    # Initialize TF2Echo node
    tf2_echo = TF2Echo()
    tf2_echo.listen_for_transform()
    
    # Shutdown the K4A camera and rospy
    # k4a.stop()
    # rospy.shutdown()


if __name__ == '__main__':
    main()
