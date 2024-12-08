import json
import numpy as np

def parse_json(R_gripper2base_path, t_gripper2base_path, R_target2cam_path, t_target2cam_path):
    indices_to_delete = [41, 51, 54, 95, 105, 106, 108]
    
    # Load and reshape gripper2base rotations
    with open(R_gripper2base_path, 'r') as f:
        data = json.load(f)
        R_gripper2base = np.array(data['rotations'])[:153, :]
        R_gripper2base = R_gripper2base.reshape(-1, 3, 3)
        print("Initial R_gripper2base shape:", R_gripper2base.shape)
    
    # Load and reshape gripper2base translations
    with open(t_gripper2base_path, 'r') as f:
        data = json.load(f)
        t_gripper2base = np.array(data['translations'])[:153, :]
        t_gripper2base = t_gripper2base.reshape(-1, 3)
        print("Initial t_gripper2base shape:", t_gripper2base.shape)

    # Load and reshape target2cam rotations
    with open(R_target2cam_path, 'r') as f:
        data = json.load(f)
        R_target2cam = np.array(data['rotations'])
        R_target2cam = R_target2cam.reshape(-1, 3, 3)
        print("Initial R_target2cam shape:", R_target2cam.shape)

    # Load and reshape target2cam translations
    with open(t_target2cam_path, 'r') as f:
        data = json.load(f)
        t_target2cam = np.array(data['translations'])
        t_target2cam = t_target2cam.reshape(-1, 3)
        print("Initial t_target2cam shape:", t_target2cam.shape)

    # Delete indices after reshaping
    counter = 0
    for idx in indices_to_delete: 
        idx -= counter
        if 0 <= idx and idx < len(R_gripper2base):  # Changed <= to < to avoid index out of bounds
            R_gripper2base = np.delete(R_gripper2base, idx, axis=0)
            t_gripper2base = np.delete(t_gripper2base, idx, axis=0)
            R_target2cam = np.delete(R_target2cam, idx, axis=0)
            t_target2cam = np.delete(t_target2cam, idx, axis=0)
            counter += 1
    
    print("\nFinal shapes after deletion:")
    print("R_gripper2base:", R_gripper2base.shape)
    print("t_gripper2base:", t_gripper2base.shape)
    print("R_target2cam:", R_target2cam.shape)
    print("t_target2cam:", t_target2cam.shape)

    # Load existing processed data and append new data
    output_dir = '../move_robot_scripts/calibration_ready_parameters/processed/'
    
    # Helper function to load existing data or create new
    def load_or_create(filepath):
        try:
            with open(filepath, 'r') as f:
                return json.load(f)
        except (FileNotFoundError, json.JSONDecodeError):
            return {'rotations': []} if 'rotations' in filepath else {'translations': []}

    # Append to processed gripper2base rotations
    filepath = output_dir + 'extra_gripper2base_rotations.json'
    existing_data = load_or_create(filepath)
    existing_data['rotations'].extend(R_gripper2base.tolist())
    with open(filepath, 'w') as f:
        json.dump(existing_data, f)

    # Append to processed gripper2base translations
    filepath = output_dir + 'extra_gripper2base_translations.json'
    existing_data = load_or_create(filepath)
    existing_data['translations'].extend(t_gripper2base.tolist())
    with open(filepath, 'w') as f:
        json.dump(existing_data, f)

    # Append to processed target2cam rotations
    filepath = output_dir + 'extra_target2cam_rotations.json'
    existing_data = load_or_create(filepath)
    existing_data['rotations'].extend(R_target2cam.tolist())
    with open(filepath, 'w') as f:
        json.dump(existing_data, f)

    # Append to processed target2cam translations
    filepath = output_dir + 'extra_target2cam_translations.json'
    existing_data = load_or_create(filepath)
    existing_data['translations'].extend(t_target2cam.tolist())
    with open(filepath, 'w') as f:
        json.dump(existing_data, f)

    return R_gripper2base, t_gripper2base, R_target2cam, t_target2cam


if __name__ == "__main__":
    parse_json('../move_robot_scripts/calibration_ready_parameters/extra_gripper2base_rotations.json', '../move_robot_scripts/calibration_ready_parameters/extra_gripper2base_translations.json', '../move_robot_scripts/calibration_ready_parameters/extra_target2cam_rotations.json', '../move_robot_scripts/calibration_ready_parameters/extra_target2cam_translations.json')