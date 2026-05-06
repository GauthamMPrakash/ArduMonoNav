r"""
    _            _       __  __                   _   _             
   / \   _ __ __| |_   _|  \/  | ___  _ __   ___ | \ | | __ ___   __
  / _ \ | '__/ _` | | | | |\/| |/ _ \| '_ \ / _ \|  \| |/ _` \ \ / /
 / ___ \| | | (_| | |_| | |  | | (_) | | | | (_) | |\  | (_| |\ V / 
/_/   \_\_|  \__,_|\__,_|_|  |_|\___/|_| |_|\___/|_| \_|\__,_| \_/  
                                                                    

This script reads RGB images from disk, applies the same optional undistort/crop
pipeline used by mononav.py, and estimates metric depth using DepthAnythingV2.

The following are saved to file:
│   ├── <transform_rgb_images>   # transformed RGB images used for depth inference
│   ├── <transform_depth_images> # estimated depth (.npy for fusion and .jpg visualization)

"""

data_dir = ""   # example to specify a data dir: data_dir = "data/demo_hallway" 
                # if left empty, the latest run (file modification date) with the prefix specified in config.yml will be used

import time
import os
import sys
import torch
import cv2
import numpy as np

# Add DepthAnythingV2-metric path
repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if repo_root not in sys.path:
    sys.path.insert(0, repo_root)
metric_depth_path = os.path.join(repo_root, 'DepthAnythingV2-metric')
sys.path.insert(0, metric_depth_path)
from depth_anything_v2.dpt import DepthAnythingV2

from utils.utils import (
  adjust_intrinsics_to_frame_size,
  compute_depth,
  get_calibration_resolution,
  get_calibration_values,
  load_config,
  split_filename,
  transform_image,
)

"""
This script runs a depth estimation model on a directory of RGB images and saves the depth images.
"""

# LOAD CONFIG
CONFIG_PATH = os.path.join(repo_root, "config.yml")
config = load_config(CONFIG_PATH)

def _latest_data_dir(prefix):
  prefix = os.path.normpath(prefix)
  directory_parent = os.path.dirname(prefix)
  basename = os.path.basename(prefix)
  if not basename:
    raise ValueError(f"data_dir prefix must have a basename, got '{prefix}'")

  if os.path.isabs(directory_parent):
    parent_dir = directory_parent
  elif directory_parent:
    parent_dir = os.path.join(repo_root, directory_parent)
  else:
    parent_dir = repo_root

  exact_dir = prefix if os.path.isabs(prefix) else os.path.join(repo_root, prefix)
  exact_dir = os.path.abspath(exact_dir)
  if os.path.isdir(exact_dir) and os.path.isdir(os.path.join(exact_dir, "rgb-images")):
    return exact_dir

  parent_dir = os.path.abspath(parent_dir)
  if not os.path.isdir(parent_dir):
    raise FileNotFoundError(f"parent data directory '{parent_dir}' not found")

  candidates = [
    os.path.join(parent_dir, entry)
    for entry in os.listdir(parent_dir)
    if entry.startswith(basename)
    and os.path.isdir(os.path.join(parent_dir, entry))
    and os.path.isdir(os.path.join(parent_dir, entry, "rgb-images"))
  ]
  if not candidates:
    raise FileNotFoundError(f"no data directories starting with '{basename}' in '{parent_dir}'")

  latest_dir = max(candidates, key=os.path.getmtime)
  print(f"[estimate_depth] using latest data directory: {latest_dir}", flush=True)
  return latest_dir


def _resolve_repo_path(path):
  return path if os.path.isabs(path) else os.path.join(repo_root, path)


def _resolve_user_data_dir(path):
  if os.path.isabs(path):
    return path
  if path.startswith("data/") or path.startswith("data\\"):
    return _resolve_repo_path(path)
  return _resolve_repo_path(os.path.join("data", path))

if not data_dir:
  data_dir = _latest_data_dir(config["data_dir"])
else:
  data_dir = _resolve_user_data_dir(data_dir)
  print(f"[estimate_depth] using data directory: {data_dir}", flush=True)

# Set & create directories for images
rgb_dir = os.path.join(data_dir, f"rgb-images")
transform_img_dir = os.path.join(data_dir, "transform-rgb-images")
transform_depth_dir = os.path.join(data_dir, "transform-depth-images")
os.makedirs(transform_img_dir, exist_ok=True)
os.makedirs(transform_depth_dir, exist_ok=True)
print(f"Saving transformed RGB images to: {transform_img_dir}")
print(f"Saving depth images to: {transform_depth_dir}")

# Load the calibration values
camera_calibration_path = config.get("camera_calibration_path")
if camera_calibration_path:
  camera_calibration_path = _resolve_repo_path(camera_calibration_path)
enable_undistort = config.get("enable_undistort", True)
print()
if enable_undistort:
    print("Undistort enabled")
else:
  print("Undistort disabled")
print()
if camera_calibration_path:
  mtx, dist, optimal_mtx, roi = get_calibration_values(camera_calibration_path)
  calib_width, calib_height = get_calibration_resolution(camera_calibration_path)
else:
  mtx = dist = optimal_mtx = roi = None
  calib_width = calib_height = None

# Load the DepthAnythingV2 model
INPUT_SIZE = config["INPUT_SIZE"]
CHECKPOINT = _resolve_repo_path(config["DA2_CHECKPOINT"])
ENCODER = CHECKPOINT[-8:-4]
if ENCODER is None:
  ENCODER = CHECKPOINT.split('_')[-1].split('.')[0]
MAX_DEPTH = config["MODEL_MAX_DEPTH"]

model_configs = {
  'vits': {'encoder': 'vits', 'features': 64, 'out_channels': [48, 96, 192, 384]},
  'vitb': {'encoder': 'vitb', 'features': 128, 'out_channels': [96, 192, 384, 768]},
  'vitl': {'encoder': 'vitl', 'features': 256, 'out_channels': [256, 512, 1024, 1024]},
}

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
print("Device is:", DEVICE)
depth_anything = DepthAnythingV2(**{**model_configs[ENCODER], 'max_depth': MAX_DEPTH})
depth_anything.load_state_dict(torch.load(CHECKPOINT, map_location=DEVICE))
depth_anything = depth_anything.to(DEVICE).eval()

# Collect all RGB image files in frame order
rgb_filenames = [
  name
  for name in os.listdir(rgb_dir)
  if os.path.isfile(os.path.join(rgb_dir, name)) and name.endswith(".jpg")
]
rgb_filenames = sorted(rgb_filenames, key=split_filename)
if len(rgb_filenames) == 0:
  raise RuntimeError(f"No RGB images found in {rgb_dir}")

first_bgr = cv2.imread(os.path.join(rgb_dir, rgb_filenames[0]))
if first_bgr is None:
  raise RuntimeError(f"Failed to read first image: {rgb_filenames[0]}")
frame_height, frame_width = first_bgr.shape[:2]

print(f"[camera] Raw frame resolution: {frame_width}x{frame_height}")
if mtx is not None:
  print(f"[camera] Calibration resolution: {calib_width}x{calib_height}")
  print(f"[camera] Calibration focal length: fx={mtx[0,0]:.1f}, fy={mtx[1,1]:.1f} pixels")

# Adjust intrinsics ONLY for raw camera resolution differences (calibration res vs actual stream res)
# This is separate from crop/undistort transformations applied later
if calib_width is not None and calib_height is not None:
  mtx, dist, optimal_mtx, roi = adjust_intrinsics_to_frame_size(
    mtx,
    dist,
    optimal_mtx,
    roi,
    frame_width,
    frame_height,
    calib_width,
    calib_height,
  )
  if abs(frame_width - calib_width) < 1e-6 and abs(frame_height - calib_height) < 1e-6:
    print(f"[camera] Resolution matches calibration (no scaling applied)")
  else:
    print(f"[camera] Resolution scaled; new focal length: fx={mtx[0,0]:.1f}, fy={mtx[1,1]:.1f} pixels")

end_frame = len(rgb_filenames)

start_time = time.time()

for idx, filename in enumerate(rgb_filenames):
  frame_number = split_filename(filename)
  print("Applying DepthAnythingV2 to: %d/%d" % (idx + 1, end_frame))
  bgr = cv2.imread(os.path.join(rgb_dir, filename))
  if bgr is None:
    print(f"[warning] failed to read image: {filename}")
    continue

  transform_bgr = transform_image(bgr, mtx, dist, optimal_mtx, roi, enable_undistort)
  # Matches mononav.py: DepthAnythingV2 predicts meters; compute_depth converts
  # to uint16 millimeters for Open3D integration with depth_scale=1000.
  depth_numpy, depth_colormap = compute_depth(transform_bgr, depth_anything, INPUT_SIZE)

  cv2.imwrite(os.path.join(transform_img_dir, f"transform_frame-{frame_number:06d}.rgb.jpg"), transform_bgr)
  cv2.imwrite(os.path.join(transform_depth_dir, f"transform_frame-{frame_number:06d}.depth.jpg"), depth_colormap)
  np.save(os.path.join(transform_depth_dir, f"transform_frame-{frame_number:06d}.depth.npy"), depth_numpy)

print("Time to compute depth for %d images: %f" % (end_frame, time.time() - start_time))
