# KUDOS Autorace 2026 Gazebo 연습 환경

Ubuntu 22.04 + ROS 2 Humble + Gazebo Classic 11에서 실행하는 LIMO 대회 연습 환경입니다.
**Gazebo 실행 터미널과 신호등 컨트롤러 터미널을 따로 사용합니다.** 경기장만 실행하면 신호등은 빨간불로 유지됩니다.

> 대회 도면을 바탕으로 만든 연습용 환경입니다. 배치 좌표는 공식 확정값이 아니며 자동 채점·자율주행 알고리즘은 포함하지 않습니다. Gazebo Classic 전용이며 Gazebo Sim용 패키지가 아닙니다.

## 1. 준비 환경

- Ubuntu 22.04, ROS 2 Humble 설치 완료
- 그래픽 화면을 사용할 수 있는 데스크톱 환경과 OpenGL 드라이버
- ROS 2 설치 안내: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

```bash
sudo apt update
sudo apt install -y \
  git build-essential cmake python3-colcon-common-extensions \
  ros-humble-gazebo-ros-pkgs ros-humble-gazebo-dev \
  ros-humble-xacro ros-humble-robot-state-publisher \
  ros-humble-ament-index-python ros-humble-launch-ros \
  ros-humble-rclcpp ros-humble-rclpy ros-humble-std-msgs \
  ros-humble-rcl-interfaces
```

이미 생성한 메시와 이미지를 포함하므로 일반 실행에는 OpenCV가 필요 없습니다.
원본 PNG에서 노면 메시를 다시 생성할 때만 `sudo apt install python3-opencv python3-numpy`가 필요합니다.

## 2. 다운로드 및 빌드

비공개 저장소일 경우 소유자가 팀원을 GitHub Settings → Collaborators에서 초대한 뒤, 팀원이 초대를 수락해야 합니다. HTTPS clone에는 GitHub CLI 로그인 또는 Personal Access Token을 사용하며 GitHub 계정 비밀번호를 입력하지 않습니다.

다른 프로젝트의 ROS 작업공간이 자동으로 source되지 않는 새 터미널에서 시작하세요. 기존 `KUDOS_scalecar`, `arena_final` 설치는 필요 없습니다.

```bash
mkdir -p ~/Workspace
cd ~/Workspace
git clone https://github.com/hotsixthekingrush/KUDOS_Autorace2026_Gazebo.git
cd KUDOS_Autorace2026_Gazebo

source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

저장소 자체가 작업공간입니다. 그 안에 `src/kudos_arena`, `src/limo_car` 두 패키지가 있습니다.

## 3. 터미널 1 — Gazebo + LIMO 실행

```bash
cd ~/Workspace/KUDOS_Autorace2026_Gazebo
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch kudos_arena competition.launch.py
```

정상 상태:

- 10 × 6 m 트랙, 출발선 앞 LIMO 1대
- 하단 3차선의 박스형 장애물 차량 7대, 주차 구역 2대
- 왼쪽 직선 S자 통로의 라바콘 14개
- 원형·삼각형 표지판, 경사형 방지턱, 빨간 신호등
- 갈림길 A/B는 실행할 때 무작위 선택. 연속으로 같은 표지가 나올 수 있음

차선추종·자율주행 노드는 실행하지 않습니다. 별도 알고리즘에서 센서를 구독하고 `/cmd_vel`을 발행해야 합니다.

차량 없이 경기장만 실행:

```bash
ros2 launch kudos_arena arena.launch.py
```

A/B 고정 테스트:

```bash
ros2 launch kudos_arena competition.launch.py branch_sign:=A
# B 고정: branch_sign:=B / 기본: branch_sign:=random
```

## 4. 터미널 2 — 신호등 컨트롤러 실행

```bash
cd ~/Workspace/KUDOS_Autorace2026_Gazebo
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run kudos_arena traffic_light_controller.py
```

기본 순환: **빨강 8초 → 초록 8초 → 노랑 2초 → 반복**.

- 컨트롤러를 실행하지 않으면 빨간불입니다.
- 컨트롤러를 `Ctrl+C`로 종료하면 1초의 **시뮬레이션 시간** 후 빨간불로 돌아갑니다.
- Gazebo 일시정지 중에는 신호 주기도 멈춥니다.
- `traffic_light_controller`는 한 개만 실행하세요.
- 컨트롤러가 발행하는 상태는 테스트용입니다. 카메라 인지 알고리즘의 정답 입력으로 사용하지 마세요.

주기를 지정해서 시작:

```bash
ros2 run kudos_arena traffic_light_controller.py --ros-args \
  -p red_seconds:=10.0 -p green_seconds:=12.0 -p yellow_seconds:=3.0
```

## 5. 터미널 3 — 신호등 고정 및 상태 확인

새 터미널마다 환경 설정이 필요합니다.

```bash
cd ~/Workspace/KUDOS_Autorace2026_Gazebo
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 param set /traffic_light_controller mode red
ros2 param set /traffic_light_controller mode green
ros2 param set /traffic_light_controller mode yellow
ros2 param set /traffic_light_controller mode auto
```

위 명령은 원하는 한 줄씩 실행합니다. 주기 변경:

```bash
ros2 param set /traffic_light_controller red_seconds 10.0
ros2 param set /traffic_light_controller green_seconds 12.0
ros2 param set /traffic_light_controller yellow_seconds 3.0
ros2 topic echo /traffic_light/state
```

`/traffic_light/command`: 컨트롤러 명령, `/traffic_light/state`: 플러그인이 적용한 상태(`std_msgs/msg/String`, RED/YELLOW/GREEN).
상태 토픽 확인과 별개로, 카메라 영상에서도 램프 변화가 보이는지 확인해야 합니다.

## 6. 라바콘 구간만 테스트

현재 Gazebo를 종료한 뒤:

```bash
ros2 launch kudos_arena competition.launch.py \
  x:=0.72 y:=1.50 z:=0.03 yaw:=1.5708
```

로봇이 왼쪽 직선 라바콘 입구에서 +Y 방향을 향합니다. 라바콘은 양쪽 7개씩이고, 이 구간에서는 별도 판단 코드에서 차선추종을 끄고 LiDAR 주행으로 전환해야 합니다.

## 7. 배치와 치수 수정

파일: `src/kudos_arena/config/track.json`

| 설정 | 역할 |
|---|---|
| `robot_spawn` | 기본 차량 스폰 위치 |
| `cones` | 개별 라바콘 배치 |
| `parked_vehicles` | 3차선·주차장 장애물 배치 |
| `signs` | 표지판 위치·종류·방향 |
| `speed_bump` | 방지턱 위치·길이·폭·높이 |
| `traffic_light` | 신호등 위치·방향·크기 |

좌표 단위는 m, yaw는 rad입니다. 도면 왼쪽 아래가 원점이고 +X는 오른쪽, +Y는 위쪽입니다.
라바콘 실제 좌표는 `(x + dx, y + dy)`이며 보정값은 누적되지 않습니다.
`cone_course`는 최초 배치의 설명용 메타데이터이며 실제 배치는 `cones`에서 수정합니다.

```json
{
  "name": "cone_left_01",
  "x": 0.36,
  "y": 1.9,
  "dx": 0.05,
  "dy": 0.0
}
```

예시는 왼쪽 첫 라바콘을 기본 위치에서 오른쪽으로 5 cm 옮깁니다.

수정 후 JSON 문법 확인:

```bash
python3 -m json.tool src/kudos_arena/config/track.json > /dev/null
```

`--symlink-install`로 빌드했다면 설정 변경 후 Gazebo 종료 → 재실행으로 반영됩니다.
실행마다 임시 world를 생성하므로 `worlds/track_2026.world`를 직접 수정하지 마세요.
새 파일·새 에셋 추가 또는 C++ 변경 시에는 `colcon build --symlink-install`이 필요합니다.
갈림길 기본값은 launch의 `random`이며 고정하려면 `branch_sign:=A/B`를 사용합니다.

## 8. 구현 범위와 주의할 차이

- 노면은 PNG 기반 근사 메시이며, 모든 치수가 최종 규정에 맞게 검증된 것은 아닙니다.
- 라바콘은 고정형입니다. 충돌은 가능하지만 밀림·전도 판정은 구현하지 않았습니다.
- 장애물 차량은 322 × 220 × 251 mm의 고정 박스입니다. 센서·제어 노드가 없습니다.
- 방지턱은 높이 15 mm, 길이 30 cm, 폭 50 cm이며 양쪽에 경사가 있습니다. 높이는 24 mm 미만으로 설정합니다. 차량 통과 안정성은 실험이 필요합니다.
- 신호등 몸체는 약 12 × 20.5 cm, 현재 램프 반지름은 2.7 cm입니다. 최종 규정의 판독부 치수 해석에 따라 수정할 수 있습니다.
- 현재 카메라는 640 × 480, 10 Hz, 수평 화각 약 80°입니다. 규정집의 약 71°와 다릅니다.
- 광고판 영상, 자동 채점, 정지선 판정, 자율주행 미션 전환은 구현하지 않았습니다.
- 신호등 플러그인 빌드·world 구조는 검증했으며, RGB 영상의 전체 색상 전환과 실제 방지턱 주행은 별도 실험이 필요합니다.

## 9. 문제 해결

### 패키지를 찾지 못함

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg prefix kudos_arena
ros2 pkg prefix limo_car
```

`arena_final`처럼 이 저장소에 없는 이름이 나오면 이전 작업공간 환경이 섞인 것입니다. `.bashrc`의 이전 workspace source 항목을 확인하고 해당 환경이 없는 터미널에서 다시 시작하세요. Gazebo 모델 경로를 `.bashrc`에 임의로 추가할 필요는 없습니다.

### 신호등이 계속 빨간불

- 두 번째 터미널에서 컨트롤러가 실행 중인지 확인합니다.
- Gazebo가 일시정지 상태인지 확인합니다.
- 두 터미널의 `ROS_DOMAIN_ID`가 같은지 확인합니다.
- `ros2 topic echo /traffic_light/state`와 Gazebo 로그의 `kudos_traffic_light` 로딩 오류를 확인합니다.

### 모델은 보이는데 이미지가 없음

빌드 후 `source install/setup.bash`를 실행했는지, `src/kudos_arena/models`의 메시·PNG·material 파일이 모두 있는지 확인합니다. launch에서 필요한 모델·플러그인 경로를 설정합니다.

### 카메라·LiDAR 확인

```bash
ros2 topic list -t
ros2 topic hz /scan
ros2 topic list | grep -E 'image|camera|rgb|depth'
```

이미지 토픽 이름은 실제 목록에서 확인하세요. 필요하면 `sudo apt install ros-humble-rqt-image-view` 후 `ros2 run rqt_image_view rqt_image_view`로 영상과 신호등 변화를 확인합니다.

## 10. 업데이트 받기

Gazebo와 컨트롤러를 종료한 뒤 실행합니다. 본인이 수정한 설정이 있으면 먼저 커밋하거나 별도 백업하세요.

```bash
cd ~/Workspace/KUDOS_Autorace2026_Gazebo
git pull --ff-only
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

출처와 차량 모델 변경 사항은 [ATTRIBUTION.md](ATTRIBUTION.md)를 참고하세요.
