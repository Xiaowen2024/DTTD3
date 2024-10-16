import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

class GripperToBaseTransform(Node):
    def __init__(self):
        super().__init__('gripper_to_base_transform')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Set the correct frames
        self.base_frame = 'link_0'  # The base frame of your robot
        self.gripper_frame = 'link_ee'  # The gripper/end-effector frame
        
        # Call the timer callback to check the transform periodically
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(self.base_frame, self.gripper_frame, now)

            # Extract the rotation and translation
            t_gripper2base = transform.transform.translation
            R_gripper2base = transform.transform.rotation

            # Log the transformation
            self.get_logger().info(f'Translation: {t_gripper2base}')
            self.get_logger().info(f'Rotation (Quaternion): {R_gripper2base}')
            
        except Exception as e:
            self.get_logger().warn(f'Could not get transform: {e}')

import cv2
import numpy as np

# Load the ArUco dictionary and create detector parameters
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters_create()

# Load your camera's calibration parameters (intrinsics and distortion coefficients)
# Example: Kinect calibration parameters (you need to have them from camera calibration)
camera_matrix = np.array([[615.0, 0.0, 320.0],
                          [0.0, 615.0, 240.0],
                          [0.0, 0.0, 1.0]])  # Replace with your Kinect intrinsics
dist_coeffs = np.array([0.1, -0.25, 0.001, 0.0, 0.1])  # Replace with your camera's distortion coefficients

# The size of the ArUco marker in meters (you must measure this)
marker_length = 0.05  # 5 cm

# Open a video stream from your Kinect (adjust for Kinect's camera input)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open video stream.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Could not read frame.")
        break

    # Convert the image to grayscale (ArUco marker detection works on grayscale images)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers in the image
    corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None:
        # Draw detected markers for visualization
        frame_markers = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)

        # Estimate pose of each marker
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs)

        # Iterate over all detected markers and display their poses
        for i in range(len(ids)):
            # Get rotation and translation vectors
            rvec = rvecs[i][0]
            tvec = tvecs[i][0]

            # Draw the axis for the marker (helps to visualize pose)
            cv2.aruco.drawAxis(frame_markers, camera_matrix, dist_coeffs, rvec, tvec, 0.05)

            # Convert rotation vector to rotation matrix
            R_target2cam, _ = cv2.Rodrigues(rvec)

            # Print rotation matrix and translation vector (pose)
            print(f"Marker ID: {ids[i]}")
            print(f"R_target2cam (Rotation Matrix):\n{R_target2cam}")
            print(f"t_target2cam (Translation Vector): {tvec}\n")

            # The R_target2cam and t_target2cam can now be used for calibration purposes

        # Show the frame with markers and axes
        cv2.imshow('ArUco Detection', frame_markers)

    else:
        # Show the original frame if no markers were detected
        cv2.imshow('ArUco Detection', frame)

    # Press 'q' to quit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close windows
cap.release()
cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = GripperToBaseTransform()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
