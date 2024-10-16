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


class TF2Echo(Node):
    def __init__(self):
        super().__init__('tf2_echo_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.base_frame = 'link_0'
        self.gripper_frame = 'link_ee'
        # self.timer = self.create_timer(1.0, self.timer_callback)

    # def timer_callback(self):
    #     self.get_transform()

    def get_transform(self):
        try:
            # Lookup the transform from source_frame to target_frame
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                self.base_frame, self.gripper_frame, rclpy.time.Time())
            
            # Print the transformation information
            self.get_logger().info(f'Transform from {self.base_frame} to {self.gripper_frame}:')
            self.get_logger().info(f'  Translation: x={transform.transform.translation.x}, '
                                   f'y={transform.transform.translation.y}, '
                                   f'z={transform.transform.translation.z}')
            self.get_logger().info(f'  Rotation: x={transform.transform.rotation.x}, '
                                   f'y={transform.transform.rotation.y}, '
                                   f'z={transform.transform.rotation.z}, '
                                   f'w={transform.transform.rotation.w}')
        except Exception as e:
            self.get_logger().error(f'Could not get transform: {str(e)}')


class CalibrateHandEye(Node):
    def __init__(self, k4a, image_path):
        super().__init__('calibrate_hand_eye')
        self.tf2_echo = TF2Echo()
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
        self.R_gripper2base, self.t_gripper2base = self.tf2_echo.get_transform()
        self.R_target2cam, self.t_target2cam = self.target_camera_transform

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
    tf2_echo = TF2Echo()
    rclpy.spin(tf2_echo)
    tf2_echo.destroy_node()

    # Create CalibrateHandEye node
    calibrate_node = CalibrateHandEye(k4a, 'front_bottom_right.png')
    R_cam2gripper, t_cam2gripper = calibrate_node.calibrate_hand_eye()

    if R_cam2gripper is not None and t_cam2gripper is not None:
        print("Camera to Gripper Rotation:")
        print(R_cam2gripper)
        print("Camera to Gripper Translation:")
        print(t_cam2gripper)
    else:
        print("Calibration failed")

    k4a.stop()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
