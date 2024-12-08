import cv2 
import numpy as np
import tf
import rospy
from get_calibration_parameters import initialize, get_parameters, close_camera, return_parameters

def get_target2base(k4a, tf_echo, r_cam2gripper, t_cam2gripper):
    r_gripper2base, t_gripper2base, r_target2cam, t_target2cam = return_parameters(k4a, tf_echo)
    r_target2base = r_gripper2base @ r_cam2gripper @ r_target2cam
    t_target2base = r_target2cam @ ( r_cam2gripper @ t_gripper2base + t_cam2gripper) + t_target2cam
    close_camera(k4a)
    return r_target2base, t_target2base

def solve_for_new_target2cam(k4a, tf_echo, r_cam2gripper, t_cam2gripper):
    r_gripper2base, t_gripper2base, r_target2cam, t_target2cam = return_parameters(k4a, tf_echo)
    r_cam2base = r_gripper2base @ r_cam2gripper
    t_cam2base = r_cam2gripper @ t_gripper2base + t_cam2gripper
    solved_r_target2cam = np.linalg.inv(r_cam2base) @ r_target2base
    solved_t_target2cam = t_target2base - solved_r_target2cam @ t_cam2base
    print("Solved r_target2cam: ", solved_r_target2cam)
    print("Solved t_target2cam: ", solved_t_target2cam)
    print("Given r_target2cam: ", r_target2cam)
    print("Given t_target2cam: ", t_target2cam)
    return solved_r_target2cam, solved_t_target2cam

if __name__ == "__main__":
    rospy.init_node('kuka_joint_controller', anonymous=True)
    k4a, tf_echo = initialize()
    r_cam2gripper = np.array([[ 0.99373069,  0.10929466, -0.02353701],
                              [-0.08965095,  0.90478364,  0.41632831],
                              [ 0.06679836, -0.41160811,  0.90890965]])
    t_cam2gripper = np.array([[0.03441822],
                              [0.00742683],
                              [0.06282144]])
    
    r_target2base, t_target2base = get_target2base(k4a, tf_echo, r_cam2gripper, t_cam2gripper)
    solve_for_new_target2cam(k4a, tf_echo, r_cam2gripper, t_cam2gripper, r_target2base, t_target2base)