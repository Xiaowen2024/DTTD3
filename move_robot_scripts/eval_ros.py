from mujoco_env_only_kuka import KukaTennisEnv
# from stable_baselines3 import PPO
from geometry_msgs.msg import PoseStamped
import rospy
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf
from copy import deepcopy
import time
from scipy.spatial.transform import Rotation as R
import numpy as np
import argparse
from mujoco_env_only_kuka_ik import KukaTennisEnv as KukaTennisEnvIK
import moveit_commander
import math
import copy

parser = argparse.ArgumentParser()
parser.add_argument('--render', action='store_true', help='Enable rendering')
parser.add_argument('--ik_rl', action='store_true', help='Enable using RL based IK')
parser.add_argument('--ik', action='store_true', help='Enable using traditional IK')
parser.add_argument('--mocap', action='store_true', help='Enable using mocap')

args = parser.parse_args()

if args.ik and args.ik_rl:
    print("IK and IK RL cannot be enabled at the same time")
    exit(0)

RATE = 60

def reset_target():
    curr_target = np.array([0.,0.,0.,0.,0.,0.,0.])
    curr_target[0] = np.random.uniform(-0.2,0.2)
    curr_target[1] = np.random.uniform(-0.5,0.5)
    curr_target[2] = np.random.uniform(0.75,1.05)
    xr,yr,zr = np.random.uniform(-1,1,3)*0.5
    z_axis = np.array([1.,yr,zr])
    z_axis = z_axis/np.linalg.norm(z_axis)
    theta_z = np.arctan2(curr_target[2],curr_target[1])+xr*0.5
    x_axis = np.array([0.,np.cos(theta_z),np.sin(theta_z)])
    x_axis[0] = (-x_axis[1]*z_axis[1] - x_axis[2]*z_axis[2])/z_axis[0]
    x_axis = x_axis/np.linalg.norm(x_axis)

    y_axis = np.cross(z_axis,x_axis)
    Rot = np.array([x_axis,y_axis,z_axis]).T @ np.array([[0,0,1],[0,-1,0],[1,0,0]])
    r = R.from_matrix(Rot)
    q = r.as_quat()
    curr_target[3:7] = q
    return curr_target


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
    
def restrict_range(val, min_val, max_val):
    return min(max(val, min_val), max_val)

def publish_trajectory_command():
    global current_positions, current_velocities, listener, trajectory_pub, env, model, env_ik, model_ik, obs_ik
    
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
    if args.ik_rl:
        env_ik.reset_pose(current_positions)
        env_ik.set_target_pose(env.curr_target)
        for j in range(50):
            action_ik, _ = model_ik.predict(obs_ik, deterministic=True)
            obs_ik, reward, done, _, info = env_ik.step(action_ik)
        action = (env_ik.data.qpos[:7]- current_positions)
    
    if args.mocap:
        obs, _, done, _, info = env.step_from_robot(joint_info, t, quat, change_target=False)
    else:
        obs, _, done, _, info = env.step_from_robot(joint_info, t, quat)

    # if done:
    #     obs, _ = env.reset()
    obs[:14] = joint_info
    # obs[7:14] = 0
    if not args.ik_rl:
        action, _ = model.predict(obs, deterministic=True)
    # print(t,quat)
    # Create a new JointTrajectory message to command the robot
    trajectory_msg = JointTrajectory()
    
    # Set the joint names (ensure that these correspond to the robot's joint names)
    trajectory_msg.joint_names = joint_names
    
    # Create a JointTrajectoryPoint message
    trajectory_point = JointTrajectoryPoint()
    # print(action[0],current_positions[0],current_velocities[0])
    if args.ik_rl:
        new_positions = list(np.array(current_positions)+1.0*np.array(action))
    else :
        new_positions = list(np.array(current_positions)+0.2*np.array(action))
    env.prev_actions[:-1,:] = env.prev_actions[1:,:]
    env.prev_actions[-1,:] = action
    new_positions[0] = restrict_range(new_positions[0],-2.96,2.96)
    new_positions[1] = restrict_range(new_positions[1],-2.09,2.09)
    new_positions[2] = restrict_range(new_positions[2],-2.94,2.94)
    new_positions[3] = restrict_range(new_positions[3],-2.09,2.09)
    new_positions[4] = restrict_range(new_positions[4],-2.94,2.94)
    new_positions[5] = restrict_range(new_positions[5],-2.09,2.09)
    new_positions[6] = restrict_range(new_positions[6],-3,3)
    # print(new_positions)
    # print(new_positions[0])
    # new_positions[0] = 0
    # Copy the current positions as the desired positions (or modify them if needed)
    trajectory_point.positions = list(new_positions)
    # trajectory_point.velocities = list(np.array(action)*0.4*RATE)
    
    # Set a duration for reaching the target position
    trajectory_point.time_from_start = rospy.Duration(1/RATE)  # 0.1 second
    if args.render:
        env.render()

    # Add the trajectory point to the trajectory message
    trajectory_msg.points.append(trajectory_point)
    
    # Publish the trajectory command
    trajectory_pub.publish(trajectory_msg)

def pose_callback(msg: PoseStamped):
    global current_target_pose, env, listener
    # Update the current_target_pose with the received position and quaternion
    position = msg.pose.position
    orientation = msg.pose.orientation
    curr_q = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
    r = R.from_quat(curr_q)
    rot_ = np.array([[0,1,0],[-1,0,0],[0,0,1]])
    rot1 = np.array([[-1,0,0],[0,0,1],[0,1,0]])
    rot2 = np.array([[1,0,0],[0,0,1],[0,-1,0]])
    Rot_new = rot2@r.as_matrix()@rot1#@rot_
    q = R.from_matrix(Rot_new).as_quat()
    current_target_pose = np.array([
        position.x, position.z, -position.y, 
        q[0], q[1], q[2], q[3]
    ])
    print("Got new pose")
    # Restrict current_target_pose to the workspace limits
    current_target_pose[0] = restrict_range(current_target_pose[0], -0.2, 0.2)
    current_target_pose[1] = restrict_range(current_target_pose[1], -0.5, 0.5)
    current_target_pose[2] = restrict_range(current_target_pose[2], 0.75, 1.05)
    z_axis = np.array([1.,0.,0.])
    x_axis = np.array([0.,0.,1.])
    y_axis = np.cross(z_axis, x_axis)
    q_straight = R.from_matrix(np.array([x_axis,y_axis,z_axis]).T).as_quat()
    # Angle between q and q_straight
    q_rel = R.from_quat(q) * R.from_quat(q_straight).inv() 
    angle = q_rel.magnitude()
    print("Angle: ", angle) 
    if angle > 1.4:
        current_target_pose[3:7] = q_straight
    env.set_target_pose(current_target_pose)
    # self.get_logger().info(f'Updated target pose: {self.current_target_pose}')

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('kuka_joint_controller', anonymous=True)
        env = KukaTennisEnv(proc_id=1)
        # model = PPO.load("logs/best_model/best_model")
        obs, _ = env.reset()
        
        if args.ik_rl:
            env_ik = KukaTennisEnvIK(proc_id=1)
            model_ik = PPO.load("logs/best_model_ik1/best_model")
            obs_ik, _ = env_ik.reset()
        # Initialize the TransformListener
        listener = tf.TransformListener()
        
        # Subscribe to the joint state topic
        rospy.Subscriber('/lbr/PositionJointInterface_trajectory_controller/state', 
                         JointTrajectoryControllerState, joint_state_callback)

        if args.mocap:
            rospy.Subscriber(
                '/vrpn_client_node/bat_close/pose',
                PoseStamped,
                pose_callback
            )
        
        # Publisher for the joint command topic
        trajectory_pub = rospy.Publisher('/lbr/PositionJointInterface_trajectory_controller/command', 
                                         JointTrajectory)

        # Keep the node alive and processing callbacks
        # Set the rate for publishing (10 Hz = 0.1 seconds)
        rate = rospy.Rate(RATE)  # 10 Hz
        if args.ik :
            name = 'arm'
            group = moveit_commander.MoveGroupCommander(name)
            group.set_max_velocity_scaling_factor(1.0)

            # move to a named target
            target = 'home'
            group.set_named_target(target)
            rospy.loginfo('Moving to named target "{}"...'.format(target))
            group.go()
            group.stop()
            rospy.loginfo('Done.')   

            for i in range(10):
                # point to point motion (PTP)
                pose = group.get_current_pose().pose
                
                waypoints = []

                new_target = reset_target()
                # move along negative x
                pose.position.x = new_target[0]
                pose.position.y = new_target[1]
                pose.position.z = new_target[2]
                pose.orientation.x = new_target[3]
                pose.orientation.y = new_target[4]
                pose.orientation.z = new_target[5]
                pose.orientation.w = new_target[6]
                waypoints.append(copy.deepcopy(pose))

                plan, fraction = group.compute_cartesian_path(waypoints, eef_step=0.01)
                rospy.loginfo('Moving along cartesian path...')
                group.execute(plan)
                group.stop()
                rospy.loginfo('Done.')
        else :
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
