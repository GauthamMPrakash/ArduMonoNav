r"""
    _            _       __  __                   _   _             
   / \   _ __ __| |_   _|  \/  | ___  _ __   ___ | \ | | __ ___   __
  / _ \ | '__/ _` | | | | |\/| |/ _ \| '_ \ / _ \|  \| |/ _` \ \ / /
 / ___ \| | | (_| | |_| | |  | | (_) | | | | (_) | |\  | (_| |\ V / 
/_/   \_\_|  \__,_|\__,_|_|  |_|\___/|_| |_|\___/|_| \_|\__,_| \_/                                                                   

The purpose of this script is to fuse depth images and poses into a 3D reconstruction.
Here, we use Open3D's tensor reconstruction system: the VoxelBlockGrid.

After fusion, the reconstruction is visualized (in addition to the camera poses), and saved to file.

"""
addPose = True  # Visualize camera poses in addition to the point cloud
data_dir = ""   # if empty, will automatically look for latest data directory with prefix specified in config.yml

import numpy as np
import time
import os
import sys
import open3d as o3d
from PIL import Image
import numpy as np
import yaml

# Ensure the repository root is on sys.path so we can import `utils` from anywhere
repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
if repo_root not in sys.path:
    sys.path.insert(0, repo_root)

from utils.utils import *
#####################################################################

CONFIG_PATH = os.path.join(repo_root, "config.yml")
with open(CONFIG_PATH, "r") as f:
    config = yaml.safe_load(f)


def _resolve_repo_path(path):
    return path if os.path.isabs(path) else os.path.join(repo_root, path)


def _resolve_user_data_dir(path):
    if os.path.isabs(path):
        return path
    if path.startswith("data/") or path.startswith("data\\"):
        return _resolve_repo_path(path)
    return _resolve_repo_path(os.path.join("data", path))


def _latest_data_dir(prefix):
    prefix = os.path.normpath(prefix)
    directory_parent = os.path.dirname(prefix)
    basename = os.path.basename(prefix)
    if not basename:
        raise ValueError(f"data_dir must have a basename, got '{prefix}'")

    exact_dir = prefix if os.path.isabs(prefix) else os.path.join(repo_root, prefix)
    exact_dir = os.path.abspath(exact_dir)
    if os.path.isdir(exact_dir) and os.path.isdir(os.path.join(exact_dir, "rgb-images")):
        return exact_dir

    if os.path.isabs(directory_parent):
        parent_dir = directory_parent
    elif directory_parent:
        parent_dir = os.path.join(repo_root, directory_parent)
    else:
        parent_dir = repo_root

    parent_dir = os.path.abspath(parent_dir)
    if not os.path.isdir(parent_dir):
        raise FileNotFoundError(f"parent data directory '{parent_dir}' not found")

    candidates = [
        os.path.join(parent_dir, entry)
        for entry in os.listdir(parent_dir)
        if entry.startswith(basename) and os.path.isdir(os.path.join(parent_dir, entry))
    ]
    if not candidates:
        raise FileNotFoundError(f"no data directories starting with '{basename}' in '{parent_dir}'")

    latest_dir = max(candidates, key=os.path.getmtime)
    print(f"[fuse_depth] using latest data directory: {latest_dir}", flush=True)
    return latest_dir

if not data_dir:
    data_dir = _latest_data_dir(config["data_dir"])
else:
    data_dir = _resolve_user_data_dir(data_dir)
    print(f"[fuse_depth] using data directory: {data_dir}", flush=True)

rgb_dir = os.path.join(data_dir, "rgb-images")
depth_dir = os.path.join(data_dir, "transform-depth-images")
pose_dir = os.path.join(data_dir, "poses")
#####################################################################

# Initialize TSDF VoxelBlockGrid
depth_scale = config["VoxelBlockGrid"]["depth_scale"]
depth_max = config["VoxelBlockGrid"]["depth_max"]
trunc_voxel_multiplier = config["VoxelBlockGrid"]["trunc_voxel_multiplier"]
weight_threshold = config["weight_threshold"] 
if config['VoxelBlockGrid']['device'] == "None":
    import torch
    device = 'CUDA:0' if torch.cuda.is_available() else 'CPU:0'
else:
    device = config['VoxelBlockGrid']['device']

# Match the offline fusion intrinsics to the actual camera stream used for depth
# estimation. This mirrors the online undistort/ROI path.
# Intrinsics for undistort (optional)
camera_calibration_path = config.get('camera_calibration_path')
enable_undistort = config.get('enable_undistort', True)
print()
if enable_undistort:
    print("Undistort enabled")
else:
  print("Undistort disabled")
print()
if camera_calibration_path:
    camera_calibration_path = _resolve_repo_path(camera_calibration_path)
    mtx, dist, optimal_mtx, roi = get_calibration_values(camera_calibration_path)  # for the robot's camera
    calib_width, calib_height = get_calibration_resolution(camera_calibration_path)
    # only compute cropped intrinsics if roi is valid
    if roi is not None:
        fusion_intrinsics = get_cropped_intrinsics(optimal_mtx, roi)
    else:
        fusion_intrinsics = None
else:                                   # no calibration available; fall back to an ideal matrix once frame size is known
    mtx = dist = optimal_mtx = roi = None
    calib_width = calib_height = None
    fusion_intrinsics = None

vbg = VoxelBlockGrid(depth_scale, depth_max, trunc_voxel_multiplier, o3d.core.Device(device), intrinsic_matrix=fusion_intrinsics)

raw_rgb_files = []
if os.path.isdir(rgb_dir):
    raw_rgb_files = sorted(
        [
            name
            for name in os.listdir(rgb_dir)
            if os.path.isfile(os.path.join(rgb_dir, name)) and name.endswith(".jpg")
        ],
        key=split_filename,
    )

if mtx is not None and len(raw_rgb_files) > 0:
    raw_sample_path = os.path.join(rgb_dir, raw_rgb_files[0])
    raw_sample = np.asarray(Image.open(raw_sample_path).convert("RGB"))
    raw_h, raw_w = raw_sample.shape[:2]
    print(f"[camera] Raw frame resolution: {raw_w}x{raw_h}")
    if calib_width is not None:
      print(f"[camera] Calibration resolution: {calib_width}x{calib_height}")
      print(f"[camera] Calibration focal length: fx={mtx[0,0]:.1f}, fy={mtx[1,1]:.1f} pixels")
    # Adjust intrinsics ONLY for raw camera resolution differences (calibration res vs actual stream res)
    # This is separate from crop/undistort transformations applied during depth estimation
    if calib_width is not None and calib_height is not None:
        mtx, dist, optimal_mtx, roi = adjust_intrinsics_to_frame_size(
            mtx, dist, optimal_mtx, roi, raw_w, raw_h, calib_width, calib_height
        )
        if abs(raw_w - calib_width) < 1e-6 and abs(raw_h - calib_height) < 1e-6:
          print(f"[camera] Resolution matches calibration (no scaling applied)")
        else:
          print(f"[camera] Resolution scaled; new focal length: fx={mtx[0,0]:.1f}, fy={mtx[1,1]:.1f} pixels")
    # Then apply crop adjustment (if undistort enabled) or raw matrix
    if enable_undistort and optimal_mtx is not None and roi is not None:
        vbg.set_intrinsics(get_cropped_intrinsics(optimal_mtx, roi))
    else:
        vbg.set_intrinsics(mtx)
        
#####################################################################

poses = [] # for visualization
t_start = time.time()

depth_files = [name for name in os.listdir(depth_dir) if os.path.isfile(os.path.join(depth_dir, name)) and name.endswith(".jpg")]
depth_files = sorted(depth_files)

# Get last frame
first_frame = split_filename(depth_files[0])
end_frame = split_filename(depth_files[-1])
total_frames = len(depth_files)

# Main integration loop (structured like mononav.py control loop)
frame_idx = 0
while frame_idx < total_frames:
    period_start = time.perf_counter()
    
    # Inner loop: integrate frames for this batch (like mononav.py period loop)
    while frame_idx < total_frames:
        filename = depth_files[frame_idx]
        frame_number = split_filename(filename)
        
        print(f"Integrating frame {frame_number}/{end_frame}")
        
        # Load camera pose
        pose_file = os.path.join(pose_dir, "frame-%06d.pose.txt"%frame_number)
        cam_pose = np.loadtxt(pose_file)
        poses.append(cam_pose)
        
        # Load TRANSFORMED color image (cropped/undistorted) to match depth dimensions
        transform_rgb_dir = os.path.join(data_dir, "transform-rgb-images")
        rgb_file = os.path.join(transform_rgb_dir, f"transform_frame-{frame_number:06d}.rgb.jpg")
        color = Image.open(rgb_file).convert("RGB")
        
        # Load and integrate depth
        depth_file = os.path.join(depth_dir, "transform_frame-%06d.depth.npy"%frame_number)
        depth_numpy = np.load(depth_file)  # in millimeters
        vbg.integration_step(color, depth_numpy, cam_pose)
        
        frame_idx += 1
        # Process all frames in one batch for offline mode
        if frame_idx >= total_frames:
            break
    
    # Batch complete
    period_elapsed = time.perf_counter() - period_start
    fps = total_frames / period_elapsed if period_elapsed > 0 else 0
    print(f"Integrated {total_frames} frames in {period_elapsed:.2f}s ({fps:.1f} FPS)", flush=True)

#####################################################################
# Print out timing information
t_end = time.time()
total_time = t_end - t_start
print(f"Total time taken (s): {total_time:.2f}")
print(f"Total FPS: {total_frames/total_time:.2f}" if total_time > 0 else "Total FPS: N/A")

pcd = vbg.vbg.extract_point_cloud(weight_threshold)

if addPose:
    pose_lineset = get_poses_lineset(poses)
    visualizer = o3d.visualization.Visualizer()
    visualizer.create_window()
    visualizer.add_geometry(pcd.to_legacy())
    visualizer.add_geometry(pose_lineset)
    for pose in poses:
        # Add coordinate frame ( The x, y, z axis will be rendered as red, green, and blue arrows respectively.)
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame().scale(0.5, center=(0, 0, 0))
        visualizer.add_geometry(coordinate_frame.transform(pose))
    visualizer.run()
    visualizer.destroy_window()
else:
    o3d.visualization.draw([pcd])

#####################################################################

npz_save_filename = os.path.join(data_dir, "vbg.npz")
ply_filename = os.path.join(data_dir, "pointcloud.ply")
save_dir = os.path.dirname(npz_save_filename)

if save_dir and os.path.isdir(save_dir):
    print('Saving npz to {}...'.format(npz_save_filename))
    print('Saving ply to {}...'.format(ply_filename))

    vbg.vbg.save(npz_save_filename)
    o3d.io.write_point_cloud(ply_filename, pcd.to_legacy())

    print('Saving finished')
else:
    print('Save directory not present; skipping VBG and point cloud save')
