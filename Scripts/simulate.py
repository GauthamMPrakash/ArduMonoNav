r"""
    _            _       __  __                   _   _             
   / \   _ __ __| |_   _|  \/  | ___  _ __   ___ | \c | | __ ___   __
  / _ \ | '__/ _` | | | | |\/| |/ _ \| '_ \ / _ \|  \| |/ _` \ \ / /
 / ___ \| | | (_| | |_| | |  | | (_) | | | | (_) | |\  | (_| |\ V / 
/_/   \_\_|  \__,_|\__,_|_|  |_|\___/|_| |_|\___/|_| \_|\__,_| \_/                                                           


The purpose of this script is to step through the 3D reconstruction and to execute the MonoNav planner.
Steps:
1) load the reconstruction, poses, and trajectory library,
2) for each pose, choose the optimal motion primitive according to the planner,
3) visualize the reconstruction, poses, and motion primitives (both available and chosen).

By default, the script iterates over every 5th pose, but you can change this by changing `n` in the code.
Loads the latest run in the data directory

This script is a useful way to debug and debrief the planner, as well as to see how changes to the planner
and trajectory library affect the planning performance. Specify

"""

data_dir = ""   # example to specify a data dir: data_dir = "data/demo_hallway" 
                # if left empty, the latest run (file modification date) in with the prefix specified in config.yml will be used
n = 2           # iterate over every n poses

import os
import open3d as o3d
import numpy as np
import copy
import sys
import tempfile
import time

# Ensure the repository root is on sys.path so we can import `utils` from anywhere
repo_root = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
if repo_root not in sys.path:
    sys.path.insert(0, repo_root)

from utils.utils import load_config, poses_from_posedir, get_poses_lineset, get_trajlist, get_traj_linesets, choose_primitive

def _latest_data_dir(prefix):
    prefix = os.path.normpath(prefix)
    directory_parent = os.path.dirname(prefix)
    basename = os.path.basename(prefix)
    if not basename:
        raise ValueError(f"save_dir_prefix must have a basename, got '{prefix}'")

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
    print(f"[simulate] using latest data directory: {latest_dir}", flush=True)
    return latest_dir


def _resolve_vbg_device(config):
    vbg_device_cfg = config.get("VoxelBlockGrid", {}).get("device")
    if vbg_device_cfg is None or str(vbg_device_cfg).lower() == "none":
        return "CUDA:0" if o3d.core.cuda.is_available() else "CPU:0"

    device_cfg_str = str(vbg_device_cfg)
    if device_cfg_str.lower().startswith("cuda"):
        suffix = device_cfg_str.split(":", 1)[1] if ":" in device_cfg_str else "0"
        return f"CUDA:{suffix}"
    if device_cfg_str.lower().startswith("cpu"):
        suffix = device_cfg_str.split(":", 1)[1] if ":" in device_cfg_str else "0"
        return f"CPU:{suffix}"
    return device_cfg_str


def _load_vbg_npz(npz_path, desired_device):
    try:
        vbg = o3d.t.geometry.VoxelBlockGrid.load(npz_path)
    except RuntimeError as exc:
        if "Unsupported device \"CUDA:0\"" not in str(exc):
            raise

        with np.load(npz_path, allow_pickle=True) as data:
            payload = {key: data[key] for key in data.files}

        for device_key in list(payload):
            if device_key.startswith("CUDA:") or device_key.startswith("CPU:"):
                payload["CPU:0"] = payload.pop(device_key)
                break
        else:
            raise RuntimeError(
                f"{npz_path} does not contain a device marker Open3D can rewrite to CPU"
            ) from exc

        with tempfile.NamedTemporaryFile(suffix=".npz", delete=False) as tmp_file:
            tmp_path = tmp_file.name

        try:
            np.savez(tmp_path, **payload)
            vbg = o3d.t.geometry.VoxelBlockGrid.load(tmp_path)
        finally:
            try:
                os.remove(tmp_path)
            except FileNotFoundError:
                pass

    if desired_device.startswith("CUDA:"):
        return vbg.cuda(int(desired_device.split(":", 1)[1]))
    return vbg.cpu()

config = load_config('../config.yml')
if not data_dir:
    data_dir = _latest_data_dir(config["data_dir"])
else:
    data_dir = data_dir if os.path.isabs(data_dir) else os.path.join(repo_root, data_dir)
print(f"[simulate] using data directory: {data_dir}", flush=True)
pose_dir = os.path.join(data_dir, "poses")
trajlib_dir = os.path.join(repo_root, config.get("trajlib_dir", "utils/trajlib"))
vbg_device = _resolve_vbg_device(config)

# Load the VoxelBlockGrid from file.
files = [file for file in os.listdir(data_dir) if file.endswith('.npz')]
assert len(files) > 0, "No *.npz files found."
npz_filename = "vbg.npz" if "vbg.npz" in files else files[0]
print("Loading", npz_filename, "with Open3D.", flush=True)
vbg = _load_vbg_npz(os.path.join(data_dir, npz_filename), vbg_device)
pcd = vbg.extract_point_cloud(config["weight_threshold"])

# Planning presets
filterYvals = config["filterYvals"]
filterWeights = config["filterWeights"]
filterTSDF = config["filterTSDF"]
debug_trajectory_scoring = True
turn_weight = config.get("turn_weight", 1.5)
repulsion_weight = config.get("repulsion_weight", 1.0)
if "goal_position_rdf" in config:
    goal_position = np.array(config["goal_position_rdf"]).reshape(1, 3) # OpenCV frame: +X RIGHT, +Y DOWN, +Z FORWARD
else:
    goal_position = None
print("Goal position:", goal_position[0] if goal_position is not None else "N/A", flush=True)
min_dist2obs = config["min_dist2obs"]
weight_threshold = config["weight_threshold"] # for planning and visualization
fallback_primitive = config.get("fallback_primitive", False)

# Load poses from directory (like reading from drone in mononav.py control loop)
poses = poses_from_posedir(pose_dir)
# Get pose lineset
pose_lineset = get_poses_lineset(poses)

# Load the trajectory linesets from the trajlib directory
traj_list = get_trajlist(trajlib_dir)
traj_linesets, period, forward_speed, amplitudes = get_traj_linesets(traj_list)

print(f"Loaded {len(poses)} poses for trajectory planning.", flush=True)

# Create the visualizer and add components
visualizer = o3d.visualization.Visualizer()
visualizer.create_window()
visualizer.add_geometry(pcd.to_legacy())
visualizer.add_geometry(pose_lineset)

# Main planning loop (structured like mononav.py control loop)
# For each pose, compute the optimal motion primitive (similar to choose_primitive in mononav.py)
print("Planning trajectories...", flush=True)
t_start = time.time()

for i in range(0, len(poses), n):
    pose = poses[i]
    if debug_trajectory_scoring:
        print(f"\n[simulate DEBUG] evaluating pose index {i}", flush=True)
    # Compute optimal trajectory at this state (like mononav.py line 501)
    max_traj_idx = choose_primitive(
        vbg,
        pose,
        traj_linesets,
        goal_position,
        min_dist2obs,
        filterYvals,
        filterWeights,
        filterTSDF,
        weight_threshold,
        turn_weight=turn_weight,
        repulsion_weight=repulsion_weight,
        fallback_primitive=fallback_primitive,
        DEBUG=debug_trajectory_scoring,
    )
    if debug_trajectory_scoring:
        print(f"[simulate DEBUG] chosen trajectory index: {max_traj_idx}", flush=True)
    
    # Visualize trajectories at this pose
    for traj_idx, traj_lineset in enumerate(traj_linesets):
        traj_lineset_copy = copy.deepcopy(traj_lineset)
        traj_lineset_copy.transform(pose)

        if traj_idx == max_traj_idx:
            traj_lineset_copy.paint_uniform_color([0, 1, 0])  # Green = optimal
        else:
            traj_lineset_copy.paint_uniform_color([0, 0, 0])  # Black = other

        visualizer.add_geometry(traj_lineset_copy)

    # # (Optional) Uncomment to add coordinate frame, which may look busy.
    # coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame().scale(0.5, center=(0, 0, 0))
    # visualizer.add_geometry(coordinate_frame.transform(pose))

t_end = time.time()
print(f"Planning complete: {len(poses)//n} decision points evaluated in {t_end-t_start:.2f}s", flush=True)

visualizer.run()
visualizer.destroy_window()
