## Calibration
Scripts for hand-to-eye calibration, using transformation data from a robot and a camera.

### Steps to Run

1. **Generate Calibration Parameters**  
   Run `get_calibration_parameters.py` to create four files for calibration:
   - `gripper2base_rotations.txt`
   - `gripper2base_translations.txt`
   - `target2cam_rotations.txt`
   - `target2cam_translations.txt`

2. **Run Calibration**  
   Use `calibrate_hand_to_eye.py` to perform the calibration, calculating and printing the camera-to-gripper rotation and translation matrices.

### Code Details

- #### `get_calibration_parameters.py`
   - Captures camera parameters and estimates the pose of ArUco markers.
   - Saves camera matrix and distortion coefficients to `color_camera_matrix.txt` and `color_camera_dist_coeffs.txt`.
   - Uses a TF listener to save gripper-to-base transformations as `gripper2base_rotations.txt` and `gripper2base_translations.txt`.
   - Detects ArUco markers in an image, estimates transformations, and saves target-to-camera matrices.

- #### `calibrate_hand_to_eye.py`
   - Loads saved transformations from `get_calibration_parameters.py`.
   - Uses OpenCV's `calibrateHandEye` to compute the rotation (`R_cam2gripper`) and translation (`t_cam2gripper`) matrices between camera and gripper, outputting these values.

## Steps to move the robot 

1. **Connect to kuka arm**
   ```bash
   source devel/setup.bash
   ```

   ```bash
   roslaunch lbr_moveit moveit_planning_execution.launch model:=iiwa14 sim:=false
   ```

2. **Run scripts to move arm to set position**
   ```bash
   export ROS_NAMESPACE=lbr
   ```
   
   ```bash
   python3 eval_ros.py --ik
   ```
