# Mine Clearing Robot Vision Grasp

ROS 2 Humble workspace notes for using an Orbbec/Astra depth camera mounted near an OpenMANIPULATOR-X base to detect a paper cup-like object and trigger a MoveIt pick sequence.

## Current Status

- Orbbec/Astra depth stream is used for object localization.
- `open_manipulator_vision_grasp` publishes:
  - `/vision/target_pose`
  - `/vision/target_marker`
  - `/vision/debug_image`
- `/pick_latest_target` triggers one manual pick attempt.
- The integrated launch starts hardware, MoveIt, camera, detector, and grasp node.
- The gripper close step can use a softer custom close position and timeout-based completion.
- After grasping, the arm can move to a configured post-grasp joint lift pose.
- The calibrated camera transform currently uses:
  - `camera_y = -0.12`
  - `camera_yaw = -1.57079632679`

## Terminal Setup

Every terminal should source the local Orbbec dependency path:

```bash
cd ~/colcon_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source setup_orbbec_local_deps.bash
```

## Build

```bash
colcon build --symlink-install --packages-select open_manipulator_vision_grasp
source install/setup.bash
```

## Integrated Launch

```bash
ros2 launch open_manipulator_vision_grasp dabai_vision_grasp.launch.py \
  start_hardware:=true \
  start_moveit:=true \
  start_detector:=true \
  start_grasp:=true \
  port_name:=/dev/ttyACM0 \
  execute_on_target:=false \
  camera_x:=0.03 \
  camera_z:=0.06 \
  camera_roll:=0.0 \
  camera_pitch:=0.0
```

Do not pass `camera_y` or `camera_yaw` unless recalibrating. The defaults are tuned for the current setup.

## Check Detection

```bash
ros2 topic echo /vision/target_pose --once
ros2 topic hz /vision/target_marker
```

Open RViz from the launch and check the `Vision Target` marker. You can also use `rqt` Image View:

```bash
rqt
```

Useful image topics:

- `/camera/depth/image_raw`
- `/vision/debug_image`

## Manual Pick

Run this only after `/vision/target_pose` is stable and the target marker matches the real cup location:

```bash
ros2 service call /pick_latest_target std_srvs/srv/Trigger {}
```

Automatic picking can be enabled with `execute_on_target:=true`, but manual picking is safer while calibrating.

## Gripper And Lift Tuning

The close grip is intentionally not fully closed by default:

```yaml
use_custom_close_position: true
close_gripper_joint_position_m: 0.0045
close_gripper_timeout_sec: 1.5
```

For a weaker grip, increase `close_gripper_joint_position_m`. For a stronger grip, decrease it.

The post-grasp lift uses a configured joint pose by default:

```yaml
use_joint_lift_pose: true
lift_joint1_rad: -1.4835298642
lift_joint2_rad: -0.1919862177
lift_joint3_rad: -0.4712388980
lift_joint4_rad: 2.181661565
```

Set `use_joint_lift_pose: false` to use the Cartesian z-lift fallback.

## Notes

- Detection is depth/shape based, not YOLO/classification based.
- Default depth detection range is 12-20 cm from the camera.
- The MoveIt pick sequence uses position-only targets for the arm because OpenMANIPULATOR-X has limited DOF and strict pose targets failed planning.
