# ArduCopter Setup

**Tested on:** Copter v4.6.3 and 4.7-beta

## Hardware Requirements

- ArduPilot compatible drone
- RGB Camera (This project uses an ESP32-CAM)
- Link to wirelessly send camera frames to GCS
- Wireless telemetry link (We used an [ESP32 Dronebridge](https://ardupilot.org/copter/docs/common-esp32-telemetry.html))
- Optical Flow sensor for indoor pose estimation
- Rangefinder is recommended
- Compass for yaw

## If you are new to ArduPilot

**YOU NEED TO SETUP AND TUNE THE DRONE**
- Recommended to use the [ArduPilot Methodic Configurator](https://github.com/ArduPilot/MethodicConfigurator)
- [AMC tuning guide document](https://ardupilot.github.io/MethodicConfigurator/TUNING_GUIDE_ArduCopter.html)
- [ArduPilot wiki](https://ardupilot.org/ardupilot/index.html)

## Software Configuration
- `EK3_SRC1_POSZ` is generally recommended to be set to baro unless you get wildly varying altitudes/bad reconstruction due to baro noise indoors and you have a flat floor.
- `EK#_SRC1_VELXY` to OpticalFlow
- `EK3_SRC1_YAW` to Compass
- Recommended to use terrain following using rangefinder with `WP_RFND_USE = 1`. This will keep the drone hugging the floor gradient and move along staircases while still providing correct actual altitude.
- Highly recommended to set up a GCS failsafe to Land or SmartRTL (slightly dangerous). This will mean the drone will execute the failsafe if it doesnt receive heartbeat packets due to the code crashing. If you do use a joystick interfaced with a GCS software, that GCS will still send heartbeats by default but you have a method of manual intervention available.
- Decrease [SRTL_ACCURACY](https://ardupilot.org/copter/docs/smartrtl-mode.html) from the default value. We set it to 0.5m.
- If you do not have an RC, you *might* have to disable the RC pre-arm check
- Tune the Loiter position controller if the drone oscillates during hover. Tune braking and jerk parametrs for smooth snappy stopping. Guided mode uses the same control loops as Loiter.

### Where to find help

- Wiki
- [ArduPilot Discourse](https://discuss.ardupilot.org/latest)
