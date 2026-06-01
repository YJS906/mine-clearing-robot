# Mine Clearing Robot Vision Grasp

OpenMANIPULATOR-X 로봇팔과 Orbbec/Astra depth camera를 이용해서 원기둥 형태의 물체, 예를 들어 종이컵 같은 물체를 인식하고 MoveIt 기반으로 집는 ROS 2 Humble 프로젝트입니다.

현재 프로젝트는 depth image로 물체 위치를 추정하고, `/pick_latest_target` 서비스를 호출하면 로봇팔이 물체를 잡은 뒤 지정된 자세로 들어 올리는 동작까지 수행합니다.

## Demo (YouTube Shorts)

<a href="https://youtu.be/azZYgt5A1QA">
  <img src="https://img.youtube.com/vi/grw7qH93KJ0/hqdefault.jpg" width="900">
</a>

---

## 현재 동작 상태

- Orbbec/Astra depth camera를 사용해서 물체 위치를 검출합니다.
- `open_manipulator_vision_grasp` 패키지가 다음 topic을 publish합니다.
  - `/vision/target_pose`
  - `/vision/target_marker`
  - `/vision/debug_image`
- `/pick_latest_target` 서비스를 호출하면 현재 인식된 물체를 한 번 집습니다.
- 통합 launch 하나로 hardware, MoveIt, camera, detector, grasp node를 같이 실행할 수 있습니다.
- gripper는 너무 세게 닫히지 않도록 custom close position을 사용합니다.
- gripper close가 물체 때문에 완전히 끝나지 않아도 timeout 이후 잡힌 것으로 간주하고 다음 동작으로 넘어갑니다.
- 물체를 잡은 뒤 설정된 joint lift pose로 들어 올립니다.
- 현재 카메라 보정값은 아래 기준입니다.
  - `camera_y = -0.12`
  - `camera_yaw = -1.57079632679`

## 환경세팅

- Ubuntu 22.04
- ROS 2 Humble
- MoveIt 2
- `ros2_control` 관련 패키지
- OpenMANIPULATOR-X ROS 2 패키지
- DynamixelSDK
- Dynamixel hardware/interface 패키지
- Orbbec `ros2_astra_camera`
- Orbbec camera용 udev rule
- U2D2/OpenMANIPULATOR-X용 udev rule

참고 문서:

- OpenMANIPULATOR-X Quick Start Guide: <https://emanual.robotis.com/docs/en/platform/openmanipulator_x/quick_start_guide/>
- Orbbec ros2_astra_camera: <https://github.com/orbbec/ros2_astra_camera>
- ROS 2 Humble Install: <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>

## 1. Workspace 생성 및 이 Repo Clone

```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws

git clone https://github.com/YJS906/mine-clearing-robot.git .
```

## 2. ROS 2 Humble 및 기본 패키지 설치

ROS 2 Humble이 설치되어 있지 않다면 먼저 ROS 2 Humble을 설치합니다.

설치 후 필요한 패키지를 설치합니다.

```bash
sudo apt update

sudo apt install -y \
  git build-essential cmake pkg-config \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  ros-humble-desktop \
  ros-humble-ros2-control \
  ros-humble-moveit \
  ros-humble-gazebo-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-position-controllers \
  ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller \
  ros-humble-gripper-controllers \
  ros-humble-hardware-interface \
  ros-humble-xacro \
  ros-humble-cv-bridge \
  ros-humble-tf2-geometry-msgs \
  ros-humble-image-geometry \
  ros-humble-camera-info-manager \
  ros-humble-image-transport \
  ros-humble-image-publisher \
  ros-humble-rqt-image-view \
  libgflags-dev libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev
```

## 3. OpenMANIPULATOR-X 관련 패키지 Clone

```bash
cd ~/colcon_ws/src

git clone -b humble https://github.com/ROBOTIS-GIT/DynamixelSDK.git
git clone -b humble https://github.com/ROBOTIS-GIT/open_manipulator.git
git clone -b humble https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git
git clone -b humble https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git
```

## 4. Orbbec/Astra Camera Driver Clone

```bash
cd ~/colcon_ws/src

git clone https://github.com/orbbec/ros2_astra_camera.git
```

## 5. udev Rule 설정

카메라 권한 설정:

```bash
cd ~/colcon_ws/src/ros2_astra_camera/astra_camera/scripts

sudo bash install.sh
sudo udevadm control --reload-rules
sudo udevadm trigger
```

OpenMANIPULATOR-X / U2D2 권한 설정:

```bash
source /opt/ros/humble/setup.bash

ros2 run open_manipulator_bringup om_create_udev_rules
```

설정 후 USB를 다시 꽂거나 PC를 재부팅하는 것이 좋습니다.

## 6. Build

```bash
cd ~/colcon_ws

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 7. 터미널 기본 설정

매번 새 터미널을 열면 아래를 먼저 실행합니다.

```bash
cd ~/colcon_ws

source /opt/ros/humble/setup.bash
source install/setup.bash
source setup_orbbec_local_deps.bash
```

`setup_orbbec_local_deps.bash`는 Orbbec camera 실행 시 필요한 local library path를 잡아주는 script입니다.

## 8. 로봇팔과 카메라 실행

OpenMANIPULATOR-X의 U2D2 port가 `/dev/ttyACM0`인 경우 아래처럼 실행합니다.

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

주의:

- `camera_y`와 `camera_yaw`는 현재 세팅에 맞춰 기본값이 보정되어 있습니다.
- 다시 calibration하는 상황이 아니면 `camera_y`, `camera_yaw`는 따로 넘기지 않습니다.
- port가 다르면 `/dev/ttyACM0` 대신 실제 port를 넣어야 합니다.

port 확인:

```bash
ls /dev/ttyACM*
```

## 9. 카메라와 물체 인식 확인

물체 pose 확인:

```bash
ros2 topic echo /vision/target_pose --once
```

marker publish 확인:

```bash
ros2 topic hz /vision/target_marker
```

카메라 image 확인:

```bash
rqt
```

`rqt`에서 `Plugins > Visualization > Image View`를 열고 아래 topic을 선택합니다.

- `/camera/depth/image_raw`
- `/vision/debug_image`

RViz에서는 `Vision Target` marker가 실제 물체 위치와 맞는지 확인합니다.

## 10. 수동 집기 실행

물체가 안정적으로 인식되고, RViz marker 위치가 실제 물체 위치와 맞으면 아래 명령어를 실행합니다.

```bash
ros2 service call /pick_latest_target std_srvs/srv/Trigger {}
```

동작 순서:

1. Depth camera로 물체 위치 검출
2. Object pose를 로봇 기준 좌표로 변환
3. Pre-grasp 위치로 이동
4. Grasp 위치로 이동
5. Gripper close
6. Close timeout 이후 잡힌 것으로 간주
7. 설정된 joint lift pose로 물체를 들어 올림

자동 집기는 `execute_on_target:=true`로 켤 수 있지만, calibration 중에는 수동 집기가 더 안전합니다.

## 11. Gripper 힘 조절

gripper close 관련 설정은 아래 파일에서 조절합니다.

```bash
src/open_manipulator_vision_grasp/config/color_grasp.yaml
```

현재 설정:

```yaml
use_custom_close_position: true
close_gripper_joint_position_m: 0.0045
close_gripper_timeout_sec: 1.5
```

조절 방법:

- 더 약하게 잡고 싶으면 `close_gripper_joint_position_m` 값을 키웁니다.
- 더 강하게 잡고 싶으면 `close_gripper_joint_position_m` 값을 줄입니다.
- gripper close 후 기다리는 시간을 바꾸려면 `close_gripper_timeout_sec`를 수정합니다.

## 12. 물체를 잡은 뒤 들어 올리는 자세 조절

현재는 Cartesian z-lift 대신 미리 정한 joint pose로 들어 올립니다.

```yaml
use_joint_lift_pose: true

lift_joint1_rad: -1.4835298642
lift_joint2_rad: -0.1919862177
lift_joint3_rad: -0.4712388980
lift_joint4_rad: 2.181661565
```

각도 기준으로는 대략 아래 값입니다.

```text
joint1 = -85 deg
joint2 = -11 deg
joint3 = -27 deg
joint4 = 125 deg
```

Cartesian z 방향으로 들어 올리는 방식으로 바꾸고 싶으면 아래처럼 설정합니다.

```yaml
use_joint_lift_pose: false
lift_height_m: 0.20
```

## 13. 자주 확인하는 명령어

node 확인:

```bash
ros2 node list
```

topic 확인:

```bash
ros2 topic list
```

controller 확인:

```bash
ros2 control list_controllers
```

카메라 depth topic 확인:

```bash
ros2 topic hz /camera/depth/image_raw
```

target pose 확인:

```bash
ros2 topic echo /vision/target_pose --once
```

수동 집기:

```bash
ros2 service call /pick_latest_target std_srvs/srv/Trigger {}
```

## Notes

- 현재 object detection은 YOLO/classification 기반이 아니라 depth/shape 기반입니다.
- 기본적으로 종이컵처럼 원기둥 형태의 물체를 잡는 것을 목표로 합니다.
- OpenMANIPULATOR-X는 자유도가 제한적이기 때문에 strict orientation target을 쓰면 MoveIt planning이 실패할 수 있습니다.
- 그래서 현재 grasp 동작은 arm에 대해 position-only target을 사용합니다.
- 카메라 위치가 바뀌면 `camera_x`, `camera_y`, `camera_z`, `camera_yaw` calibration이 다시 필요합니다.
