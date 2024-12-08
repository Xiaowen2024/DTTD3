import cv2 
import numpy as np
import tf
import rospy
from get_calibration_parameters import initialize, get_parameters, close_camera, return_parameters

def get_target2base(k4a, tf_echo, r_cam2gripper, t_cam2gripper):
    r_gripper2base, t_gripper2base, r_target2cam, t_target2cam = return_parameters(k4a, tf_echo)
    r_target2base = r_gripper2base @ r_cam2gripper @ r_target2cam
    t_gripper2base = t_gripper2base.reshape(3, 1)
    t_cam2gripper = t_cam2gripper.reshape(3, 1)
    t_target2cam = t_target2cam.reshape(3, 1)
    t_target2base = r_target2cam @ ( r_cam2gripper @ t_gripper2base + t_cam2gripper) + t_target2cam
    print("t_target2base: ", t_target2base)
    print("r_target2base: ", r_target2base)
    close_camera(k4a)
    return r_target2base, t_target2base

def solve_for_new_target2cam(k4a, tf_echo, r_cam2gripper, t_cam2gripper,  r_target2base, t_target2base):
    r_gripper2base, t_gripper2base, r_target2cam, t_target2cam = return_parameters(k4a, tf_echo)
    t_gripper2base = t_gripper2base.reshape(3, 1)
    t_cam2gripper = t_cam2gripper.reshape(3, 1)
    r_cam2base = r_gripper2base @ r_cam2gripper
    t_cam2base = r_cam2gripper @ t_gripper2base + t_cam2gripper
    t_cam2base = t_cam2base.reshape(3, 1)
    t_target2base = t_target2base.reshape(3, 1)
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
    
    
    # r_target2base, t_target2base = get_target2base(k4a, tf_echo, r_cam2gripper, t_cam2gripper)
    r_target2base = np.array([[ 0.99334568, -0.10700226, 0.04260122], 
                               [-0.02854864,  -0.58711803, -0.80899777], 
                              [ 0.11157653,   0.80239824, -0.58626594]])
    t_target2base = np.array([[ 0.07865686],
                               [-0.60613613],
                              [ 0.29058196]])
    solve_for_new_target2cam(k4a, tf_echo, r_cam2gripper, t_cam2gripper, r_target2base, t_target2base)