import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from pyk4a import PyK4A, Config
import cv2
import matplotlib.pyplot as plt


class CameraCalibration:
    @staticmethod
    def save_color_camera_matrix(calibration):
        camera_matrix = calibration.get_camera_matrix(1)
        with open('color_camera_matrix.txt', 'w') as f:
            np.savetxt(f, camera_matrix, fmt='%.6f')

    @staticmethod
    def get_color_camera_matrix(calibration):
        return calibration.get_camera_matrix(1)

    @staticmethod
    def save_color_dist_coefficients(calibration):
        dist_coeffs = calibration.get_distortion_coefficients(1)
        with open('color_camera_dist_coeffs.txt', 'w') as f:
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
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
        # Detect ArUco markers in the image
        corners, ids, rejected = detector.detectMarkers(frame)
        if ids is not None:
            # Draw detected markers for visualization
            frame_markers = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)

            #Estimate pose of each marker
            rvecs, tvecs = CameraCalibration.estimate_pose_single_markers(corners, camera_matrix, dist_coeffs, marker_length)

            # Get rotation and translation vectors
            rvec = rvecs[0]  # This should now be a 3x1 array
            tvec = tvecs[0]

            # Convert rotation vector to rotation matrix
            print(rvec.shape)
            print(tvec.shape)
            R_target2cam, _ = cv2.Rodrigues(rvec)
            return R_target2cam, tvec


class GripperToBaseTransform(Node):
    def __init__(self):
        super().__init__('gripper_to_base_transform')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_frame = 'link_0'  # The base frame of robot
        self.gripper_frame = 'link_ee'  # The gripper/end-effector frame
        
    def get_transform(self):
        now = rclpy.time.Time()
        transform = self.tf_buffer.lookup_transform(self.base_frame, self.gripper_frame, now)
        t_gripper2base = transform.transform.translation
        R_gripper2base = transform.transform.rotation
        return t_gripper2base, R_gripper2base

# 	cv.calibrateHandEye(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam[, R_cam2gripper[, t_cam2gripper[, method]]])
class CalibrateHandEye(Node):
    def __init__(self, k4a, image_path):
        super().__init__('calibrate_hand_eye')
        self.gripper_to_base_transform = GripperToBaseTransform()
        self.k4a = k4a
        self.calibration = self.k4a.calibration
        self.camera_matrix = CameraCalibration.get_color_camera_matrix(self.calibration)
        self.dist_coeffs = CameraCalibration.get_color_dist_coefficients(self.calibration)
        self.target_camera_transform = CameraCalibration.get_target_camera_transform(
            image_path, 
            self.camera_matrix, 
            self.dist_coeffs, 
            0.2159
        )
        self.R_gripper2base, self.t_gripper2base = self.gripper_to_base_transform.get_transform()
        self.R_target2cam, self.t_target2cam = self.target_camera_transform


    def calibrate_hand_eye(self):
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(self.R_gripper2base, self.t_gripper2base, self.R_target2cam, self.t_target2cam)
        print(R_cam2gripper)
        print(t_cam2gripper)
        return R_cam2gripper, t_cam2gripper


def main():
    # Initialize and start the K4A camera
    k4a = PyK4A()
    k4a.start()

    rclpy.init()
    
    # Create CalibrateHandEye node
    calibrate_node = CalibrateHandEye(k4a, 'front_bottom_right.png')
    R_cam2gripper, t_cam2gripper = calibrate_node.calibrate_hand_eye()
    print(R_cam2gripper, t_cam2gripper)
    
    k4a.stop()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
