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
            print(f"R_target2Cam: {R_target2cam}")
            print(f"T_target2Cam: {tvec}")
            
            # Save R_target2cam
            try:
                existing_rotations = np.loadtxt('target2cam_rotations.txt')
                existing_rotations = existing_rotations.reshape(-1, 3, 3)
                np.savetxt('target2cam_rotations.txt',
                          np.vstack((existing_rotations, R_target2cam.reshape(1, 3, 3))).reshape(-1, 3),
                          fmt='%.6f')
            except (OSError, IOError):
                np.savetxt('target2cam_rotations.txt', R_target2cam, fmt='%.6f')

            # Save tvec
            try:
                existing_translations = np.loadtxt('target2cam_translations.txt')
                existing_translations = existing_translations.reshape(-1, 3)
                np.savetxt('target2cam_translations.txt',
                          np.vstack((existing_translations, tvec.reshape(1, 3))),
                          fmt='%.6f')
            except (OSError, IOError):
                np.savetxt('target2cam_translations.txt', tvec.reshape(1, 3), fmt='%.6f')

            return R_target2cam, tvec



class TF2Echo(Node):
    def __init__(self):
        super().__init__('tf2_echo_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_frame = 'link_0'
        self.source_frame = 'link_ee'
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.rotation = None;
        self.translation = None;


        self.timer = self.create_timer(1.0, self.timer_callback)
        self.transform_received = False

    
    def timer_callback(self):
            if not self.transform_received:
                try:
                    transform: TransformStamped = self.tf_buffer.lookup_transform(
                        self.target_frame, self.source_frame, rclpy.time.Time())
                
                    
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
                    self.transform_received = True
                    self.save_transform(transform)
                    self.timer.cancel()
                except Exception as e:
                    self.get_logger().error(f'Could not get transform: {str(e)}')


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
            # Save rotation matrix
            try:
                existing_rotations = np.loadtxt('gripper2base_rotations.txt')
                existing_rotations = existing_rotations.reshape(-1, 3, 3)
                np.savetxt('gripper2base_rotations.txt',
                          np.vstack((existing_rotations, rotation.reshape(1, 3, 3))).reshape(-1, 3),
                          fmt='%.6f')
            except (OSError, IOError):
                np.savetxt('gripper2base_rotations.txt', rotation, fmt='%.6f')

            # Save translation vector
            try:
                existing_translations = np.loadtxt('gripper2base_translations.txt')
                existing_translations = existing_translations.reshape(-1, 3)
                np.savetxt('gripper2base_translations.txt',
                          np.vstack((existing_translations, translation.reshape(1, 3))),
                          fmt='%.6f')
            except (OSError, IOError):
                np.savetxt('gripper2base_translations.txt', translation.reshape(1, 3), fmt='%.6f')

            return rotation, translation
        except Exception as e:
            self.get_logger().error(f'Could not get transform: {str(e)}')

    

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

        
            

def main():
    rclpy.init()

    # Initialize and start the K4A camera
    k4a = PyK4A()
    k4a.start()  
    capture = k4a.get_capture()
    img_color = capture.color
    image_path = 'front_left.png'
    plt.imshow(img_color[:, :, 2::-1])
    plt.imsave(image_path, img_color[:, :, 2::-1])
    calibration = k4a.calibration
    camera_matrix = CameraCalibration.get_color_camera_matrix(calibration)
    print(f"camera_matrix: {camera_matrix}")
    dist_coeffs = CameraCalibration.get_color_dist_coefficients(calibration)
    print(f"dist_coeffs: {dist_coeffs}")
    CameraCalibration.get_target_camera_transform(
        image_path,
        camera_matrix,
        dist_coeffs,
        0.2159
    )
    
    # Create and spin TF2Echo node to get transforms
    tf2_echo = TF2Echo()

    # # Loop to manually spin until we get the transform
    while rclpy.ok():
        rclpy.spin_once(tf2_echo)
        
        # Check if the transform has been received
        if tf2_echo.translation is not None and tf2_echo.rotation is not None:
            print("Transform obtained, starting calibration")
            break  # Exit the loop when transform is received

        # Sleep briefly to prevent busy waiting
        time.sleep(0.1)

    # # Proceed with calibration if the transform is obtained
    # if tf2_echo.translation is not None and tf2_echo.rotation is not None:
    #     print("started calculation")
    #     calibrate_node = CalibrateHandEye(k4a, 'front_bottom_right.png', tf2_echo.rotation, tf2_echo.translation)
    #     R_cam2gripper, t_cam2gripper = calibrate_node.calibrate_hand_eye()

    #     if R_cam2gripper is not None and t_cam2gripper is not None:
    #         print("Camera to Gripper Rotation:")
    #         print(R_cam2gripper)
    #         print("Camera to Gripper Translation:")
    #         print(t_cam2gripper)
    #     else:
    #         print("Calibration failed")
    # else:
    #     print("Failed to get transform")

    tf2_echo.destroy_node()
    k4a.stop()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
