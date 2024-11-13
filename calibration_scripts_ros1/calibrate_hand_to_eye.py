import numpy as np
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import cv2
import matplotlib.pyplot as plt


class CalibrateHandEye():
    def __init__(self, R_gripper2base_path, t_gripper2base_path, R_target2cam_path, t_target2cam_path):
        super().__init__()
        gripper_to_camera = np.array([
            [-1, 0, 0],
            [-1, 0, 1],
            [-1, -1, 0]
        ])
        self.R_gripper2base = np.loadtxt(R_gripper2base_path).reshape(-1, 3, 3)
        self.t_gripper2base = np.loadtxt(t_gripper2base_path).reshape(-1, 3)
        self.R_gripper2base = self.R_gripper2base[:, [0, 2, 1]]
        self.R_gripper2base[:, 0] = -self.R_gripper2base[:, 0]
        self.R_gripper2base[:, 1] = -self.R_gripper2base[:, 1]
        self.t_gripper2base = np.dot(self.t_gripper2base, gripper_to_camera)
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
    



def compute_residuals(R_cam2gripper, t_cam2gripper, R_gripper2base, t_gripper2base, R_target2cam, t_target2cam):
    residuals = []
    for i in range(len(R_gripper2base)):
        # Reconstruct the target pose using the estimated hand-eye transformation
        est_t_target = R_gripper2base[i] @ t_cam2gripper + t_gripper2base[i]
        residual = np.linalg.norm(est_t_target - t_target2cam[i])
        residuals.append(residual)
    return residuals


def main():
    calibrateHandEye = CalibrateHandEye('new_calibration_gripper2base_rotations.txt', 'new_calibration_gripper2base_translations.txt', 'new_calibration_target2cam_rotations.txt', 'new_calibration_target2cam_translations.txt')
    R_cam2gripper, t_cam2gripper = calibrateHandEye.calibrate_hand_eye()
    print(f"R_cam2gripper: {R_cam2gripper}")
    print(f"t_cam2gripper: {t_cam2gripper}")
    residuals = compute_residuals(R_cam2gripper, t_cam2gripper, calibrateHandEye.R_gripper2base, calibrateHandEye.t_gripper2base, calibrateHandEye.R_target2cam, calibrateHandEye.t_target2cam)
    print("t_gripper2base sample:", calibrateHandEye.t_gripper2base[:5])
    print("t_target2cam sample:", calibrateHandEye.t_target2cam[:5])

    plt.plot(residuals)
    plt.title('Residuals of Calibration')
    plt.xlabel('Sample Index')
    plt.ylabel('Residual Error')
    plt.show()
   



if __name__ == "__main__":
    main()
