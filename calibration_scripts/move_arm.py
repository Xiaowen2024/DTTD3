from mujoco_env_only_kuka import KukaTennisEnv
from stable_baselines3 import PPO

import rospy
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf
from copy import deepcopy
import time
from scipy.spatial.transform import Rotation as R
import numpy as np


RATE = 20
# Callback function for the subscriber
def joint_state_callback(data):
    global current_positions, current_velocities, joint_names
    # Extract the current joint positions and velocities from the state message
    current_positions = data.actual.positions
    current_velocities = np.array(data.actual.velocities)
    joint_names = data.joint_names
    # print(time.time())
    # rospy.loginfo("Current Joint Positions: %s", current_positions)
    # rospy.loginfo("Current Joint Velocities: %s", current_velocities)

    # Now we read the transform between lbr_link_0 and lbr_link_ee
    
    

def publish_trajectory_command():
    global current_positions, current_velocities, listener, trajectory_pub, env, model
    
    try:
        # Wait for the transform to be available, with a timeout of 1 second
        listener.waitForTransform('lbr_link_0', 'lbr_link_ee', rospy.Time(), rospy.Duration(1.0))
        
        # Get the transform from lbr_link_0 to lbr_link_ee
        (trans, rot) = listener.lookupTransform('lbr_link_0', 'lbr_link_ee', rospy.Time(0))
        
        # rospy.loginfo("Translation: %s", trans)
        # rospy.loginfo("Rotation (quaternion): %s", rot)
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        rospy.logwarn("Failed to get the transform between lbr_link_0 and lbr_link_ee")

    # Get z axis of the end effector
    # dir_z = R.from_quat(rot).as_matrix()
    # print(dir_z)
    t = np.array(trans)
    # print(0.1*dir_z,rot)
    new_rot_mat = R.from_quat(rot).as_matrix() @ np.array([[0, 0, 1], [0, -1, 0], [1, 0, 0]])
    quat = R.from_matrix(new_rot_mat).as_quat()
    # Rotate the end effector by 90 degrees around the z axis
    joint_info = list(current_positions) + list(current_velocities)

    obs, _, done, _, info = env.step_from_robot(joint_info, t, quat)
    if done:
        obs, _ = env.reset()
    obs[:14] = joint_info
    # obs[7:14] = 0
    action, _ = model.predict(obs, deterministic=True)
    # print(t,quat)
    # Create a new JointTrajectory message to command the robot
    trajectory_msg = JointTrajectory()
    
    # Set the joint names (ensure that these correspond to the robot's joint names)
    trajectory_msg.joint_names = joint_names
    
    # Create a JointTrajectoryPoint message
    trajectory_point = JointTrajectoryPoint()
    print(action[0],current_positions[0],current_velocities[0])
    new_positions = list(np.array(current_positions)+0.2*np.array(action))
    # print(new_positions[0])
    # new_positions[0] = 0
    # Copy the current positions as the desired positions (or modify them if needed)
    trajectory_point.positions = list(new_positions)
    # trajectory_point.velocities = list(np.array(action)*0.4*RATE)
    
    # Set a duration for reaching the target position
    trajectory_point.time_from_start = rospy.Duration(1/RATE)  # 0.1 second
    env.render()

    # Add the trajectory point to the trajectory message
    trajectory_msg.points.append(trajectory_point)
    
    # Publish the trajectory command
    trajectory_pub.publish(trajectory_msg)

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('kuka_joint_controller', anonymous=True)
        env = KukaTennisEnv(proc_id=1)
        model = PPO.load("logs/best_model/best_model")
        obs, _ = env.reset()
        # Initialize the TransformListener
        listener = tf.TransformListener()
        
        # Subscribe to the joint state topic
        rospy.Subscriber('/lbr/PositionJointInterface_trajectory_controller/state', 
                         JointTrajectoryControllerState, joint_state_callback)

        # Publisher for the joint command topic
        trajectory_pub = rospy.Publisher('/lbr/PositionJointInterface_trajectory_controller/command', 
                                         JointTrajectory, queue_size=10)

        # Keep the node alive and processing callbacks
        # Set the rate for publishing (10 Hz = 0.1 seconds)
        rate = rospy.Rate(RATE)  # 10 Hz

        # Main loop to publish commands at 20 Hz
        while not rospy.is_shutdown():
            publish_trajectory_command()
            rate.sleep()

        

    except rospy.ROSInterruptException:
        pass


# env = KukaTennisEnv(proc_id=1)
# model = PPO.load("logs/best_model/best_model")
# obs, _ = env.reset()
# for i in range(20000):
#     # print(i)
#     action, _ = model.predict(obs, deterministic=True)
#     # action = env.get_expert_cmd()
#     obs, reward, done, _, info = env.step(action*0)
#     # print(i, reward)
#     env.render()
#     if done:
#         # print("Reset requested")
#         obs, _ = env.reset()
#     # if done:
#     #     obs, _  = env.reset()

# env.close()