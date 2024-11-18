import numpy as np
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import cv2
import matplotlib.pyplot as plt
import json
import sys


class CalibrateHandEye():
    def __init__(self, R_gripper2base_path, t_gripper2base_path, R_target2cam_path, t_target2cam_path):
        super().__init__()
        # Load gripper2base rotations from JSON file
        with open(R_gripper2base_path, 'r') as f:
            data = json.load(f)
            self.R_gripper2base = np.array(data['rotations'])#[:10,:,]
            print(self.R_gripper2base.shape)
        
        # Load gripper2base translations from JSON file 
        with open(t_gripper2base_path, 'r') as f:
            data = json.load(f)
            self.t_gripper2base = np.array(data['translations'])#[:10,:,]
            print(self.t_gripper2base.shape)

        # Load target2cam rotations from text file
        # Try loading from JSON first
    
        with open(R_target2cam_path, 'r') as f:
            data = json.load(f)
            self.R_target2cam = np.array(data['rotations'])#[:10,:,]
            print(self.R_target2cam.shape)
    

        # Try loading translations from JSON first
        with open(t_target2cam_path, 'r') as f:
            data = json.load(f)
            self.t_target2cam = np.array(data['translations'])#[:10,:,]
            print(self.t_target2cam.shape)
       


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
    calibrateHandEye = CalibrateHandEye('../move_robot_scripts/calibration_ready_parameters/calibration_gripper2base_rotations.json', '../move_robot_scripts/calibration_ready_parameters/calibration_gripper2base_translations.json', '../move_robot_scripts/calibration_ready_parameters/calibration_target2cam_rotations.json', '../move_robot_scripts/calibration_ready_parameters/calibration_target2cam_translations.json')
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
    sys.exit()
   



if __name__ == "__main__":
    main()
