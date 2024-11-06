import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import cv2

class CalibrateHandEye():
    def __init__(self, R_gripper2base_path, t_gripper2base_path, R_target2cam_path, t_target2cam_path):
        super().__init__()
        self.R_gripper2base = np.loadtxt(R_gripper2base_path).reshape(-1, 3, 3)
        self.t_gripper2base = np.loadtxt(t_gripper2base_path).reshape(-1, 3)
        self.R_target2cam = np.loadtxt(R_target2cam_path).reshape(-1, 3, 3)
        self.t_target2cam = np.loadtxt(t_target2cam_path).reshape(-1, 3)


    def calibrate_hand_eye(self):
        if self.R_gripper2base is None or self.t_gripper2base is None or self.R_target2cam is None or self.t_target2cam is None:
            self.get_logger().error('Missing parameters')
            return None, None

        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            self.R_gripper2base, self.t_gripper2base, 
            self.R_target2cam, self.t_target2cam
        )
        return R_cam2gripper, t_cam2gripper

def main():
    calibrateHandEye = CalibrateHandEye('gripper2base_rotations.txt', 'gripper2base_translations.txt', 'target2cam_rotations.txt', 'target2cam_translations.txt')
    R_cam2gripper, t_cam2gripper = calibrateHandEye.calibrate_hand_eye()
    print(f"R_cam2gripper: {R_cam2gripper}")
    print(f"t_cam2gripper: {t_cam2gripper}")


if __name__ == "__main__":
    main()
