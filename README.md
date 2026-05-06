# ArduMonoNav: MAV Navigation via Monocular Depth Estimation and Reconstruction

---

ArduMonoNav is a monocular navigation stack that uses RGB images and camera poses to build a 3D reconstruction, enabling conventional planning techniques on a MAV with only a single camera instead of heavy and expensive setups like LiDAR, stereo cameras or RGB-D cameras. The original [MonoNav](https://github.com/natesimon/MonoNav) pipeline used ZoeDepth on a Crazyflie. This fork adapts the pipeline for an **ArduPilot drone** and replaces the depth estimator with the [metric depth version](https://github.com/DepthAnything/Depth-Anything-V2/tree/main/metric_depth) of **[Depth Anything V2](https://github.com/DepthAnything/Depth-Anything-V2)**.

At each planning step, ArduMonoNav:

1. receives an RGB frame from the onboard camera,
2. estimates metric depth with Depth Anything V2,
3. fuses RGB-D frames and vehicle poses into an Open3D TSDF reconstruction,
4. scores a library of motion primitives against the reconstruction and goal,
5. sends the selected primitive as MAVLink velocity/yaw commands, and
6. repeats until the goal is reached or no safe primitive is available.

This project currently uses an ESP32-CAM sending images at 480x320 over TCP using the ESP32 `CameraWebServer` example code in Arduino.

**Note**: Not tested with fisheye lenses. Possibility of scale mismatch exists as seen when running the [MonoNav](https://github.com/natesimon/MonoNav) out-of-the-box demo with the provided intrinsics. More info given down below in `Demo: out of the Box!` section 

## Overview

This repository contains code to run:

**ArduMonoNav pipeline** ([`mononav.py`](mononav.py)): the integrated real-time navigation loop for an ArduCopter vehicle. It connects over MAVLink, reads an ESP32-style MJPEG camera stream, estimates depth with Depth Anything V2, maintains a TSDF map, chooses motion primitives, and commands the vehicle in `GUIDED` mode.

There are also scripts that break ArduMonoNav into sub-parts for offline experimentation:

1. **Data collection pipeline**: Run [mononav.py](mononav.py) with `FLY_VEHICLE: False` in [config.yml](config.yml) and fly the drone manually (Remember to press 'g' to start the data collection before flying forward). This will save all the data as the online run but the /poses and /rgb-images will be the data to be used by 1.b. below. 
2. **Depth estimation pipeline** ([`Scripts/estimate_depth.py`](Scripts/estimate_depth.py)): estimate depths from RGB images.
3. **Fusion pipeline** ([`Scripts/fuse_depth.py`](Scripts/fuse_depth.py)): fuse depth images and camera poses into a 3D reconstruction.
4. **Simulate ArduMonoNav** ([`Scripts/simulate.py`](Scripts/simulate.py)): step through a reconstruction and visualize the motion primitives chosen by the planner.


The repository includes:

- [`DepthAnythingV2-metric/`](DepthAnythingV2-metric): [Depth Anything V2 metric-depth] code.
- [`utils/mavlink_control.py`](utils/mavlink_control.py): MAVLink helpers for ArduCopter.
- [`utils/generate_primitives.py`](utils/generate_primitives.py): generate and visualize motion primitives.
- [`utils/calibration/`](utils/calibration): camera calibration helpers and sample intrinsics.
- [`data/demo_hallway`](data/demo_hallway): a sample dataset for out-of-the-box demo.
- [`ArduCopter Setup`](ArduCopter_Setup.md): ArduPilot setup notes for the vehicle side.

## Improvements

- DepthAnythingV2 is faster and more accurate than ZoeDepth
- Automatic intrinsics scaling for any resolution after initial camera calibration so you don't have to recalibrate. Just directly run the code
- Modified trajectory selection for better navigation. The original [implementation] tried to align the vehicle's heading with the vehicle-goal vector as soon as possible and got stuck if faced with an obstacle. The current implementation moves around obstacles implementing a potential-field repulsion navigation. 

## Installation and Configuration

Clone the repository:

```bash
git clone https://github.com/GauthamMPrakash/ArduMonoNav
cd ArduMonoNav
```

Create the conda environment:

```bash
conda env create --file environment.yml
conda activate mononav
```

or with mamba:

```bash
mamba env create --file environment.yml
mamba activate mononav
```

Install/check any extra Depth Anything V2 metric requirements if needed (generally not needed as the previous step already installs required libraries):

```bash
pip install -r DepthAnythingV2-metric/requirements.txt
```

**Tested on:** (release / driver / GPU)  
- Linux Mint 22.1  / NVIDIA 535 / RTX 3050
- Linux Mint 22.1  / ----------------- / i3-1125G4 with Intel UHD

### Depth Anything V2 Checkpoint

Download a Depth Anything V2 metric checkpoint from [here](https://github.com/DepthAnything/Depth-Anything-V2/tree/main/metric_depth#pre-trained-models) and place it under:

```text
DepthAnythingV2-metric/checkpoints/
```

You will only need either Small or Base

The default [`config.yml`](config.yml) expects:

```yaml
DA2_CHECKPOINT: "DepthAnythingV2-metric/checkpoints/depth_anything_v2_metric_hypersim_vitb.pth"
MODEL_MAX_DEPTH: 20
INPUT_SIZE: 252
```

For indoor flight, the Hypersim models are usually the right starting point. Make sure the checkpoint name matches the encoder (`vits`, `vitb`, or `vitl`), because `mononav.py` derives the encoder from the checkpoint filename.

Theoretically, you can use ArduMonoNav outside with the VKITTI 2 dataset checkpoints but this hasn't been tested.

### ArduCopter and Camera Configuration

Read [`ArduCopter Setup`](ArduCopter_Setup.md) before flying. At minimum, you need:

- an ArduPilot-compatible drone,
- a monocular RGB camera stream,
- a telemetry link to the ground computer,
- reliable pose estimation for indoor flight, such as optical flow plus EKF,
- a rangefinder recommended for altitude/terrain following, and
- a tuned vehicle that is already safe to fly manually.

If you do not have a discrete GPU, set the VoxelBlockGrid device in `config.yml` to `CPU:0`.
Setting the aforementioned parameter to None enables automatic device selection. This will only detect the first instances of a GPU and a CPU and the priority order of device selection is GPU:0 > CPU:0

## Demo: Out of the Box!

The repository includes a demo_hallway dataset captured by us located at [`data/demo_hallway`](data/demo_hallway):

```text
data/demo_hallway/
├── rgb-images/
└── poses/
```

These data were captured using our [hardware](ArduCopter_Setup.md). The defaults in `config.yml` will work.

1. To demonstrate Depth Anything V2 depth estimation, run `python Scripts/estimate_depth.py`. This reads in the RGB images and transforms them to match the camera intrinsics used in the Depth Anything V2 training dataset. This is crucial for depth estimation accuracy (see Camera Calibration for more details). The transformed images are saved in `transform-rgb-images/` and used to estimate depth. The estimated depths are saved as numpy arrays and colormaps (for visualization) in `transform-depth-images/`. After running, take a look at the resulting images and note the loss of peripheral information as the raw images are undistorted.

2. To demonstrate fusion, run: `python Scripts/fuse_depth.py`. This script reads in the (transformed) images, poses, and depths, and integrates them using Open3D's TSDF Fusion. After completion, a reconstruction should be displayed with coordinate frames to mark the camera poses throughout the run. The reconstruction is saved to file as a VoxelBlockGrid (vbg.npz) and pointcloud (pointcloud.ply - which can be opened using MeshLab).

3. Next, run `python Scripts/simulate.py`. This loads the reconstruction (vbg.npz) and executes the ArduMonoNav planner. The planner is executed at each of the camera poses, and does the following:

    (i) visualizes (in black) the available motion primitives in the trajectory library (`utils/trajlib`),

    (ii) chooses a motion primitive according to the planner: `choose_primitive()` in `utils/utils.py` selects the primitive that makes the most progress towards goal_position while remaining min_dist2obs from all obstacles in the reconstruction,

    (iii) paints the chosen primitive green. `simulate.py` is useful for debugging and de-briefing, and also to anticipate how changes in the trajectory library or planner affect performance. For example, by changing `min_dist2obs` in `config.yml`, it is possible to see how increasing/decreasing the distance threshold to obstacles affects planner performance.

4. Finally, try changing the motion primitives to see how they affect planner performance! To modify and generate the trajectory library, open `utils/generate_primitives.py`. Try changing `num_trajectories` from 7 to 11, and run `python utils/generate_primitives.py`. This will display the new motion primitives and update the trajectory library. Note that each motion primitive is defined by a set of gentle turns left, right, or straight. An "extension" segment is added to the primitive (but not flown) to encourage foresight in the planner. See our paper for more details. Feel free to re-run `python Scripts/simulate.py` to try out the new primitives!

The tutorial should result in the additional files added to `data/demo_hallway`:

```text
<demo_hallway>
├── <transform-rgb-images>   # images transformed to remove distortion
├── <transform-depth-images> # estimated depth (.npy for fusion and .jpg for visualization)
├── vbg.npz / pointcloud.ply # reconstructions generated by fuse_depth.py
```

**Demo ArduMonoNav on MonoNav Crazyflie dataset**

The repository also includes the original MonoNav demo dataset at [`data/crazyflie_demo_hallway`](data/crazyflie_demo_hallway). 
For this one to work, you need to scale down the reconstruction sizes by a factor. This is probably due to incorrect intrinsics or a quirk of DepthAnythingV2 when it works on fisheye. Another possibility is that fisheye calibration has to be used instead of the pinhole model currently used in [utils.py](utils/utils.py). The original MonoNav used a simpler pinhole model with the fisheye lens but it has worked there. A quick fix is ot set `depth_scale` to 2750. This number has been found empirically to be 2x the averga zoom factor on either axes after the cropping.
We have implemented a scaling code in utils.py to scale down the image because of the zooming effect caused by heavy cropping after undistortion. To enable this, set `depth_scale_scaling = True` in line 29 of utils.py. But we have found empirically, that it is still off by a factor of 2 as mentioned above. So you also need to set `_depth_scale_zoom_factor = 2.0`

Also set `camera_calibration_path: 'utils/calibration/cf_demo_intrinsics.json'` and use the original `goal_position_rdf` of (10m, -1m, -10m)

It contains RGB images and poses:

```text
data/crazyflie_demo_hallway/
├── rgb-images/
└── poses/
```

A demo snippet from the original repo:

<img src="utils/reconstruction.gif" height="250px" alt="reconstruction animation"/>

## Running ArduMonoNav

The integrated ArduCopter pipeline writes live-flight data using:

```text
data/mononav-<timestamp>/
├── rgb-images/
├── poses/
├── transform-rgb-images/
├── transform-depth-images/
├── trajectories.csv
└── vbg.npz
```

The offline scripts are useful for debugging a saved run. The real-time [`mononav.py`](mononav.py) script is the primary entry point for this ArduCopter version.

Before running, confirm:

1. the vehicle is configured and test-flown using ArduPilot,
2. `config.yml` has the correct MAVLink connection string and camera stream URL,
3. the Depth Anything V2 checkpoint exists at `DA2_CHECKPOINT`,
4. camera calibration is correct, or `enable_undistort` is set appropriately,
5. the motion primitive library exists in [`utils/trajlib`](utils/trajlib), and
6. you can stop/land the vehicle safely from an RC transmitter or ground station.

**Note:** Use an **integer** number for `camera_src` in config.yml for USB-interfaced feeds like FPV camera receivers.

`camera_src: 0`

Start the integrated pipeline:

```bash
python mononav.py
```

The script connects to the drone, sets the EKF origin, starts the camera/depth/fusion loop, and switches to `GUIDED` when flight is enabled. Set:

```yaml
FLY_VEHICLE: False
```

to test the perception and planning loop without arming/taking off.

### Keyboard Controls

During flight:

```text
g: enable MonoNav autonomous mode
a: manually fly left primitive
w: manually fly straight primitive
d: manually fly right primitive
q: yaw left
e: yaw right
f: fuse current frame without moving
c: brake and land
r: switch to SMART_RTL
p: emergency stop (Motors will stop and drone will crash! Only use in case of an emergency)
```

Manual primitive control is useful for checking that camera poses, depth, and reconstruction are sensible before enabling autonomous mode. Start in a controlled environment and keep a human pilot ready to take over.

Currently the planner treats unseen space as free so while using narrow field of view cameras, the fusion pipeline can only see a few meters in front. Therefore it will start turning right into an obstacle to the unseen sides if the goal specified falls in that direction and the vbg integration for the newly viewd areas don't occur fast enough. Therefore, it is recommended to start the vehicle a few meters behind.

## Planning and Reconstruction

MonoNav plans over a trajectory library stored in [`utils/trajlib`](utils/trajlib). Each primitive is checked against the Open3D TSDF reconstruction. The planner in [`utils/utils.py`](utils/utils.py) selects a primitive that satisfies `min_dist2obs` and, when `goal_position_rdf` is configured, makes progress toward the goal.

Important planning settings in [`config.yml`](config.yml):

```yaml
goal_position_rdf:
  - 1
  - -1.5
  - 7
min_dist2obs: 0.7
min_dist2goal: 0.7
forward_speed: 0.5
traj_period: None
filterYvals: True
filterWeights: True
filterTSDF: True
weight_threshold: 3
```
Goal position is given in local coordinates aligned with the heading of the drone at the arming point.

Note: Currently, the 'down' value in goal_position_rdf is unused other than for checking if the drone reached its goal. The project currently does not implement 3D control so keep this value close to the takeoff altitude.

Comment out the goal position lines for undirected exploration where the planner picks the trajectory with the most clearnace from obstacles.

To regenerate primitives:

```bash
python utils/generate_primitives.py
```

Inspect [`utils/trajlib/visualization.png`](utils/trajlib/visualization.png) or run the offline simulator on a saved reconstruction to see how changes affect the planner.

## Camera Calibration

Metric depth and TSDF fusion are sensitive to camera intrinsics. Use [`utils/calibration/intrinsics.json`](utils/calibration/intrinsics.json) as a template, but calibrate your own camera whenever possible.

**Note: Unless you are using a lens with noticeable distortion, you need not calibrate the camera and set `enable_undistort` to False.**

This repository currently includes:

- [`utils/calibration/calibration.py`](utils/calibration/calibration.py) -> Obtain intrinsic matrix using OpenCV calibration
- [`utils/calibration/charuco_board.png`](utils/calibration/charuco_board.png) -> Board to calibrate camera
- [`utils/calibration/intrinsics.json`](utils/calibration/intrinsics.json) -> Intrinsics saved here
- [`utils/calibration/cube.py`](utils/calibration/cube.py) -> To test distortion correction

Either print or use a fullscreen image of the ChAruCo board on screen. 

Change these parameters in the calibration.py code:\
`SQUARES_X, SQUARES_Y = 10, 7` -> defines number of inner squares in the ChAruCo. 10x7 ChAruCo actually has 11x8 sqaures\
`SQUARE_LENGTH, MARKER_LENGTH = 0.0235, 0.0175` -> square length and breadth in meters\
`URL = "http://192.168.53.56:81/stream"` -> URL of the camera stream. Use an integer number for USB-interfaced feeds\
\
Set the calibration path in `config.yml`:

```yaml
camera_calibration_path: "utils/calibration/intrinsics.json"
enable_undistort: True
```

If your lens has strong distortion, enabling undistortion can improve depth quality, but it changes the image crop and field of view. The code will usually handle this but check transformed images before flight.

**The code automatically rescales the intrinsics for any input resolution after an initial calibration of the camera**

## Important Tuning Before Successful Runs

- Fly the drone to a goal straight ahead
- Land and check the reconstruction. It should be like the images shown below. Anything too sparse may lead to crashed into walls and anything too dense will be noisy. It's better to be slighly on the sparser side.
- You can tune the density of the reconstruction by changing these variables in config.yml

  **Depth Model settings**\
    `INPUT_SIZE: 252`    -> DepthAnythingV2 scales the smaller dimension of the input image to this size while also correspondingly resizing the longer dimension to maintain aspect ratio. Higher values take more compute. Ensure this is a multiple of 14 not exceeding the smaller dimension of the input image
    `DA2_CHECKPOINT: "DepthAnythingV2-metric/checkpoints/depth_anything_v2_metric_hypersim_vitb.pth"`   -> We find that the Small model leads to denser depth maps which could be due to faster processing and more integrations for a given scene.

  **Reconstruction settings**\
    `weight_threshold: 3`     # determines the weight threshold for filtering (higher threshold -> fewer points)
  
- After the above steps are completed and if you observe the drone tends to crash into walls or does not move towards the goal even while in open space, try tuning the `min_dist2obs` parameter and the motion primitive "foresight" extension defined as `traj_extension` [generate_primitives.py](utils/generate_primitives.py)\
The drone will choose the trajectory which gets it closer to goal while prioritising obstacle clearance. An "obstacle" here refers to any occupied voxel. `min_dist2obs` is the "bubble" (safety radius) around each point on the motion primitive. In goalless mode, primitives with points inside these bubbles are discarded. In goal-seeking mode, primitives are safety-gated by this threshold, and among safe trajectories, the planner balances goal progress, turn aggressiveness, and clearance margin. If no safe trajectories exist, the planner falls back to the primitive with the best clearance.

**Tuning the primitive chooser**

- If the planner keeps picking the same straight primitive and ignores the goal, lower the repulsion term or increase the goal weight.
- If it starts making awkward aggressive turns, increase the turn weight above 1.5.
- If it feels too cautious and never changes behavior across goals, 1.5 is probably too small compared with the obstacle repulsion term.
- **Fallback behavior**: If all trajectories violate the safety threshold, the planner enters fallback mode and selects the primitive with the best (least bad) clearance. This prevents the drone from getting stuck, but may result in high-risk maneuvers if `min_dist2obs` is too large relative to the environment. This fallback behaviour should probably not be used and reliability has to be improved by tuning other parameters instead.

- It is better to tune the drone's yaw acceleration to ensure it can hit the most extreme motion primitives in the required time period. This [script](tests/fly_primitives.py) might be helpful to evaluate the flight

## Additional Notes and Safety

- This project does not sync the image frames with pose data. This might be required if you plan on flying at higher speeds. 
- In this project, we connected both the ESP32-CAM and the Dronebridge ESP32 to the ground station PC set up as a hotspot or by connecting them to a router to which the GCS is also connected.
- Keep the camera fairly pointed straight. Slight deviations do not matter! But it may be necessary to perform a camera-IMU calibration for better reconstruction especially when using angled cameras.
- A point of failure for the depth estimation is sudden changes in lighting which take place before the camera has time to adjust the exposure which can throw off the model's estimation resulting in false obstacles appearing in front.

- **The code does not send heartbeats at regular intervals so you need to connect a GCS software like MissionPlanner or QGroundControl connected to the vehicle. Make sure to only run the code after connecting the GCS otherwise the GCS will override the parameter stream rates. It is better to disable GCS controlled parameter stream rates anyway to avoid congestion.**

- **In MP, the stream rates are found in Planner tab in Config. Set them to 0 or -1 (controlled by vehicle). In QGC, go to Settings -> Telemetry -> Stream Rates -> Enable "Controlled By vehicle"**

- **Highly recommend to have an RC or a joystick interfaced via a GCS software like MissionPlanner or QGroundControl to quickly switch modes to land or perform emergency stops in case the code crashes**

## ArduPilot BendyRuler Approach

[AP_ObstacleAvoidance.py](AP_ObstacleAvoidance.py) was developed to use monocular depth estimation as a substitute for a Realsense RGB-D camera but BendyRuler does not appear to be working. It is a modified version of the [d4xx_to_mavlink.py](https://github.com/thien94/vision_to_mavros/blob/master/scripts/d4xx_to_mavlink.py) script. Refer https://ardupilot.org/copter/docs/common-oa-bendyruler.html

We did however test that the OBSTACLE_DISTANCE MavLink message works for the now deprecated (but available to be built into [custom ArduPilot firmwares](https://custom.ardupilot.org/)) simple obstacle avoidance in Stop mode.

## Useful Test Scripts

The [`tests/`](tests) directory contains practical hardware checks:

- [`tests/fly_primitives.py`](tests/fly_primitives.py): command individual motion primitives.
- [`tests/run_camera_stream_depth.py`](tests/run_camera_stream_depth.py): test camera stream plus Depth Anything V2 inference.
- [`tests/keyboard_ctrl.py`](tests/keyboard_ctrl.py): A fun script for keyboard teleoperation through MAVLink.
- [`tests/udp_cam_da2.py`](tests/udp_cam_da2.py): Depth Anything V2 stream test over UDP. Not used currently.

Run these in a safe setup before attempting autonomous flight.

[`utils/mavlink_control.py`](utils/mavlink_control.py) has a `test()` function you can modify and run as a standalone script to verify telemetry functionality. Ensure you have a fully tuned and configured drone and perform any flight tests safely.

## Future Work

ArduMonoNav remains a work in progress. Useful directions include:

- Improving planner conservatism in unseen space. Currently, the planner treats unseen space as open so it may make decisions to go into those areas before the actual obstacles there will be detected. 
- Limit frequency of the main loop so that everyone can potentially obtain similar results without twiddling with the settings mentioned [here](README.md#important-tuning-before-successful-runs)         
- Implement a global planner like D*-Lite or RRT* and use ArduPilot's waypoint navigation which is probably more efficient. The current depth map can be constantly evaluated to see if there's an obstacle in front of the vehicle instead of checking the VoxelBlockGrid and a replanning can be triggered.
- If using motion primitives, it may be better to add a stop and yaw primitive instead of having a constant forward velocity always.
- Adding ROS/ROS 2 interoperability 
- Evaluating newer, better monocular metric-depth models
- Using SLAM pipeline for localization using camera only, so that we can neglect an optical flow sensor (DepthAnything3 seems like it does both SLAM and monocular depth estimation while also being very heavy in terms of computation)
