import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from pyk4a import PyK4A, Config
import cv2
import matplotlib.pyplot as plt
import time


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
            rvec = rvecs[0] 
            tvec = tvecs[0]
            R_target2cam, _ = cv2.Rodrigues(rvec)
            return R_target2cam, tvec



class TF2Echo(Node):
    def __init__(self):
        super().__init__('tf2_echo_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_frame = 'link_0'
        self.source_frame = 'world'
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.transform = None

    def timer_callback(self):
        self.transform = self.get_transform()

    def get_transform(self):
        try:
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.target_frame, self.source_frame, rclpy.time.Time())
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
            return rotation, translation
        except Exception as e:
            self.get_logger().error(f'Could not get transform: {str(e)}')
        return None, None
    

    def quaternion_to_rotation_matrix(self, x, y, z, w):
        """
        Convert a quaternion (x, y, z, w) to a rotation matrix (3x3).
        """
        R = np.zeros((3, 3))
        R[0, 0] = 1 - 2 * (y * y + z * z)
        R[0, 1] = 2 * (x * y - z * w)
        R[0, 2] = 2 * (x * z + y * w)
        R[1, 0] = 2 * (x * y + z * w)
        R[1, 1] = 1 - 2 * (x * x + z * z)
        R[1, 2] = 2 * (y * z - x * w)
        R[2, 0] = 2 * (x * z - y * w)
        R[2, 1] = 2 * (y * z + x * w)
        R[2, 2] = 1 - 2 * (x * x + y * y)
        
        return R

        
            


class CalibrateHandEye(Node):
    def __init__(self, k4a, image_path):
        super().__init__('calibrate_hand_eye')
        self.tf2_echo = TF2Echo()
        self.k4a = k4a
        self.calibration = self.k4a.calibration
        self.camera_matrix = CameraCalibration.get_color_camera_matrix(self.calibration)
        print(f"camera_matrix: {self.camera_matrix}")
        self.dist_coeffs = CameraCalibration.get_color_dist_coefficients(self.calibration)
        print(f"dist_coeffs: {self.dist_coeffs}")
        self.R_gripper2base, self.t_gripper2base = self.tf2_echo.get_transform()
        print(f"R_gripper2base: {self.R_gripper2base}")
        print(f"t_gripper2base: {self.t_gripper2base}")
        self.R_target2cam, self.t_target2cam = CameraCalibration.get_target_camera_transform(
            image_path, 
            self.camera_matrix, 
            self.dist_coeffs, 
            0.2159
        )
        print(f"R_target2cam: {self.R_target2cam}")
        print(f"t_target2cam: {self.t_target2cam}")

    def calibrate_hand_eye(self):
        if self.R_gripper2base is None or self.t_gripper2base is None:
            self.get_logger().error('Failed to get gripper to base transform')
            return None, None

        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            self.R_gripper2base, self.t_gripper2base, 
            self.R_target2cam, self.t_target2cam
        )
        return R_cam2gripper, t_cam2gripper


def main():
    rclpy.init()

    # Initialize and start the K4A camera
    k4a = PyK4A()
    k4a.start()
    
    # Create and spin TF2Echo node for a few seconds to get transforms
    tf2_echo = TF2Echo()
    rclpy.spin_once(tf2_echo)
    rotation, translation = tf2_echo.get_transform()
    while rotation is None and translation is None:
        print("listening to transform")
        rclpy.spin_once(tf2_echo)
        time.sleep(0.1) 

    # Create CalibrateHandEye node only if we got the transform
    if rotation is not None or translation is not None:
        calibrate_node = CalibrateHandEye(k4a, 'front_bottom_right.png')
        R_cam2gripper, t_cam2gripper = calibrate_node.calibrate_hand_eye()

        if R_cam2gripper is not None and t_cam2gripper is not None:
            print("Camera to Gripper Rotation:")
            print(R_cam2gripper)
            print("Camera to Gripper Translation:")
            print(t_cam2gripper)
        else:
            print("Calibration failed")
    else:
        print("Failed to get transform")

    tf2_echo.destroy_node()
    k4a.stop()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
