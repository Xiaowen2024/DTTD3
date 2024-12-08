import json
import numpy as np

def combine_calibration_data():
    processed_dir = '../move_robot_scripts/calibration_ready_parameters/processed/'
    
    # Load and combine rotation data
    with open(processed_dir + 'processed_gripper2base_rotations.json', 'r') as f:
        data1 = json.load(f)
        R_gripper2base_1 = np.array(data1['rotations'])
    
    with open(processed_dir + 'extra_gripper2base_rotations.json', 'r') as f:
        data2 = json.load(f)
        R_gripper2base_2 = np.array(data2['rotations'])
    
    # Combine rotations
    R_gripper2base_combined = np.concatenate([R_gripper2base_1, R_gripper2base_2], axis=0)
    
    # Load and combine translation data
    with open(processed_dir + 'processed_gripper2base_translations.json', 'r') as f:
        data1 = json.load(f)
        t_gripper2base_1 = np.array(data1['translations'])
    
    with open(processed_dir + 'extra_gripper2base_translations.json', 'r') as f:
        data2 = json.load(f)
        t_gripper2base_2 = np.array(data2['translations'])
    
    # Combine translations
    t_gripper2base_combined = np.concatenate([t_gripper2base_1, t_gripper2base_2], axis=0)
    
    # Load and combine target2cam data
    with open(processed_dir + 'processed_target2cam_rotations.json', 'r') as f:
        data1 = json.load(f)
        R_target2cam_1 = np.array(data1['rotations'])
    
    with open(processed_dir + 'extra_target2cam_rotations.json', 'r') as f:
        data2 = json.load(f)
        R_target2cam_2 = np.array(data2['rotations'])
    
    R_target2cam_combined = np.concatenate([R_target2cam_1, R_target2cam_2], axis=0)
    
    with open(processed_dir + 'processed_target2cam_translations.json', 'r') as f:
        data1 = json.load(f)
        t_target2cam_1 = np.array(data1['translations'])
    
    with open(processed_dir + 'extra_target2cam_translations.json', 'r') as f:
        data2 = json.load(f)
        t_target2cam_2 = np.array(data2['translations'])
    
    t_target2cam_combined = np.concatenate([t_target2cam_1, t_target2cam_2], axis=0)
    
    # Save combined data
    combined_data = {
        'rotations': R_gripper2base_combined.tolist()
    }
    with open(processed_dir + 'combined_gripper2base_rotations.json', 'w') as f:
        json.dump(combined_data, f)
    
    combined_data = {
        'translations': t_gripper2base_combined.tolist()
    }
    with open(processed_dir + 'combined_gripper2base_translations.json', 'w') as f:
        json.dump(combined_data, f)
    
    combined_data = {
        'rotations': R_target2cam_combined.tolist()
    }
    with open(processed_dir + 'combined_target2cam_rotations.json', 'w') as f:
        json.dump(combined_data, f)
    
    combined_data = {
        'translations': t_target2cam_combined.tolist()
    }
    with open(processed_dir + 'combined_target2cam_translations.json', 'w') as f:
        json.dump(combined_data, f)
    
    print("Combined shapes:")
    print("R_gripper2base:", R_gripper2base_combined.shape)
    print("t_gripper2base:", t_gripper2base_combined.shape)
    print("R_target2cam:", R_target2cam_combined.shape)
    print("t_target2cam:", t_target2cam_combined.shape)

if __name__ == "__main__":
    combine_calibration_data() 