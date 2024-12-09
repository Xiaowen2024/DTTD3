import argparse
import cv2
import numpy as np
from tqdm import tqdm
import yaml

import os, sys
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.join(dir_path, ".."))

from utils.constants import SCENES_DIR, REAL_SENSE_COLOR_HEIGHT, REAL_SENSE_COLOR_WIDTH
from utils.frame_utils import load_bgr, load_label, load_depth

def overlay_label_color(color, label):
    opacity = 0.7
    colors = np.array([(0, 0, 0), (51, 102, 255), (153, 51, 255), (204, 0, 204), (255, 204, 0), (153, 204, 0)
            , (0, 102, 102), (51, 102, 0), (153, 0, 204), (102, 0, 51), (102, 255, 255), (102, 255, 153)
            , (153, 51, 0), (102, 153, 153), (102, 51, 0), (153, 153, 102), (255, 204, 153), (255, 102, 102), (0, 255, 153)
            , (102, 0, 102), (153, 255, 51), (51, 102, 153)
            , (9, 88, 172), (88, 9, 172), (9, 172, 88), (88, 172, 9), (172, 88, 9), (172, 9, 88)])
    colors = np.tile(np.expand_dims(np.expand_dims(colors, 0), 0), (REAL_SENSE_COLOR_HEIGHT, REAL_SENSE_COLOR_WIDTH, 1, 1))

    label = np.expand_dims(np.expand_dims(label, -1), -1)
    label_colors = np.take_along_axis(colors, label, axis=2).squeeze(2)

    color = color.astype(np.float32) / 255.
    label_colors = label_colors.astype(np.float32) / 255. * opacity

    color += label_colors
    color = np.clip(color, 0, 1)

    color = (color * 255.).astype(np.uint8)
    return color

def main():
    parser = argparse.ArgumentParser(description='Generate semantic labeling and meta labeling')
    parser.add_argument('scene_name', type=str, help='scene directory (contains scene_meta.yaml and data (frames) and camera_poses)')

    args = parser.parse_args()

    scene_dir = os.path.join(SCENES_DIR, args.scene_name)
    frames_dir = os.path.join(scene_dir, "data")

    scene_metadata_file = os.path.join(scene_dir, "scene_meta.yaml")
    with open(scene_metadata_file, 'r') as file:
        scene_metadata = yaml.safe_load(file)

    out_arr = []

    num_frames = scene_metadata["num_frames"]


    for frame_id in tqdm(range(num_frames), total=num_frames):
        color_img = load_bgr(frames_dir, frame_id, "jpg")
        label = load_label(frames_dir, frame_id)
        overlay = overlay_label_color(color_img, label)
        depth = load_depth(frames_dir, frame_id)

        label *= 3000
        color_img = cv2.resize(color_img, dsize=(REAL_SENSE_COLOR_WIDTH//2, REAL_SENSE_COLOR_HEIGHT//2), interpolation=cv2.INTER_CUBIC)
        label = cv2.resize(label, dsize=(REAL_SENSE_COLOR_WIDTH//2, REAL_SENSE_COLOR_HEIGHT//2), interpolation=cv2.INTER_CUBIC)
        overlay = cv2.resize(overlay, dsize=(REAL_SENSE_COLOR_WIDTH//2, REAL_SENSE_COLOR_HEIGHT//2), interpolation=cv2.INTER_CUBIC)
        depth = cv2.resize(depth, dsize=(REAL_SENSE_COLOR_WIDTH//2, REAL_SENSE_COLOR_HEIGHT//2), interpolation=cv2.INTER_NEAREST)

        label = np.ascontiguousarray(cv2.cvtColor(label, cv2.COLOR_GRAY2BGR))
        depth = np.ascontiguousarray(cv2.cvtColor(depth, cv2.COLOR_GRAY2BGR))

        out = np.zeros((color_img.shape[0]*2, color_img.shape[1]*2, 3)).astype(np.uint8)

        out[:color_img.shape[0],:color_img.shape[1],:] = color_img
        out[:color_img.shape[0],color_img.shape[1]:,:] = depth
        out[color_img.shape[0]:,:color_img.shape[1],:] = overlay
        out[color_img.shape[0]:,color_img.shape[1]:,:] = label

        out_arr.append(out)

    size = (REAL_SENSE_COLOR_WIDTH, REAL_SENSE_COLOR_HEIGHT)

    fourcc = cv2.VideoWriter_fourcc('m', 'p', '4', 'v')
    out_vid = cv2.VideoWriter(os.path.join(dir_path, '..', 'demos' , '{0}.mp4'.format(args.scene_name)), fourcc, 15, size)

    for i in range(len(out_arr)):
        out_vid.write(out_arr[i])

    out_vid.release()

if __name__ == "__main__":
    main()