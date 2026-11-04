# ar_lane_perception

카메라 영상에서 차선을 찾아 `/perception/lane_error` (`ar_msgs/LaneError`)를 발행한다.
판단팀 `ar_mission_supervisor`가 구독한다. 선 추적은 `limo_ros2_application/detect_line.py`와 같은 로직이다.

> ⚠️ **실차 미검증.** Gazebo 시뮬과 오프라인 합성 영상에서만 확인했다.
> 카메라 높이·피치·초점거리, 색 임계값, 차선 폭은 실물에서 다시 맞춰야 한다.

## 실행

```bash
colcon build --symlink-install --base-paths src --packages-select ar_msgs ar_lane_perception
ros2 launch ar_lane_perception camera_perception.launch.py            # 실차: /camera/color/image_raw
ros2 launch ar_lane_perception camera_perception.launch.py sim:=true  # 시뮬: /rgb/image_raw, lane_width_m 0.52
ros2 launch ar_lane_perception camera_perception.launch.py image_topic:=/my/image
```

## 입출력

| 방향 | 토픽 | 타입 | 비고 |
|---|---|---|---|
| 구독 | `image_topic` 파라미터 | `sensor_msgs/Image` | sensor data QoS |
| 발행 | `/perception/lane_error` | `ar_msgs/LaneError` | reliable, depth 10, 영상 프레임마다 |
| 발행 | `distance_y` | `std_msgs/Int32` | `publish_distance_y: true`일 때만 (기본 꺼짐). 기존 `limo_control`용 px 값 |
| 발행 | `debug_image` | `sensor_msgs/Image` | `publish_debug_image: true`일 때만 (기본 꺼짐) |

## LaneError 필드

| 필드 | 값 |
|---|---|
| `header.stamp` | 입력 영상의 `header.stamp` (촬영 시각) |
| `header.frame_id` | `base_link` |
| `valid` | false면 나머지 값은 쓰지 말 것. 판단팀은 **valid만** 보고 차선 유실로 처리한다 (confidence로 거르지 않음) |
| `lateral_offset_m` | 기준점에서 차선 중심선까지 수직거리. 중심선이 **왼쪽이면 +** [m] |
| `heading_error_rad` | 차선 중심선 방향 − 차 진행방향. 차선이 **왼쪽으로 꺾이면 +** [rad] |
| `curvature` | 항상 0.0 (아직 추정하지 않음) |
| `confidence` | 0~1 품질 지표. 확률로 보정된 값이 아니다 |
| `single_lane_estimated` | 한 줄이라도 한쪽 선 + `lane_width_m`/2 로 중심을 추정했으면 true |

**기준점:** `lateral_offset_m`과 `heading_error_rad`는 차 중심(`base_link` 원점)에서 **앞쪽 `reference_distance_m` = 0.5 m** 지점 기준이다.
**이 값은 판단팀 `ar_mission_supervisor`의 `lane.reference_distance_m`과 같아야 한다.** 한쪽만 바꾸면 제어가 틀어진다.

`valid=false`가 되는 경우: 쓸 수 있는 중심점이 2개 미만, 전방 거리 범위 10 cm 미만, 또는 아래 가드.

## 옆 도로 선 가드

**현상:** 자기 차선의 한쪽 선이 안 보이고 차가 그쪽으로 치우치면, 약 22 cm 바깥에 있는 옆 도로 구간의 노란 선을
그 선으로 잡아 `lateral_offset_m`가 약 22 cm 틀린다. 이때 confidence는 낮아지지 않는다.

**판별:** 이 상황에서는 한 영상 줄에 "자기 차선 선 + 옆 도로 선" 쌍이 같이 잡히고, 둘의 지면 간격이
차선 폭보다 약 0.22 m 넓다. 기존 추적은 이 쌍을 폭 이상으로 보고 한 선만 남기는데, 어느 선을 남겨도
기하적으로 대칭이라 한 프레임만으로는 맞는지 알 수 없다. 그래서:

1. 어떤 줄에서 두 선의 수직 폭(지면 y 간격 × cos heading)이 `lane_width_m`과 `guard_width_tol_m`(0.10 m) 넘게 다르고,
   한쪽 선 추정을 쓴 줄이 있으면 → **애매한 프레임**.
2. 애매한 프레임은 직전 정상 출력(`guard_memory_sec` 1.0 s 이내)과 lateral 차이가 `guard_jump_m`(0.10 m) 이하일 때만 valid.
3. 직전 정상값이 없거나 오래됐으면 valid=false.
4. 직전 정상값(기준)은 두 선이 같이 보인 줄이 있는 프레임에서만 새로 세운다. 한쪽 선만 보인 프레임은
   이미 있는 기준과 이어질 때만 기준을 갱신한다 (옆 도로 선 하나만 잡은 프레임이 틀린 기준을 세우지 못하게).

**시뮬 오프라인 확인 결과** (기존 주행 로그 포즈와 직선 합성 시퀀스로 영상을 렌더링해 확인):
- 직선 45조건: 22 cm 오검출 프레임 450/450 거름.
- 직선 중간에 안쪽 선 1 m가 끊기는 15조건: 오검출 50/50 거름, 정상 프레임은 하나도 버리지 않음.
- 기존 정상 주행 로그 11개 (`tools/log_*.csv` 포즈): 정상 프레임 3864개 중 valid=false로 바뀐 것 0개.
- 같은 로그 포즈에서 바깥선을 지운 경우: 정상 프레임 3818개 중 2개가 valid=false.
- 모서리에서는 영상 가로줄이 차선을 비스듬히 자르므로 폭에 cos(heading)을 곱한다.
  이 보정이 없으면 정상 로그 모서리에서 15프레임이 오탐으로 나왔다.

**한계:**
- 시작부터 계속 한쪽 선이 없고 옆 도로 선이 보이면 (비교할 직전값이 없음) 맞게 잡아도 valid=false가 된다 (보수적).
- 옆 도로 선만 보이고 자기 차선 선이 하나도 안 보이면 쌍이 없어서 그 프레임은 못 거른다.
  직전 기준이 없으면 그런 첫 프레임 1개는 틀린 값이 valid로 나갈 수 있다 (이후 애매한 프레임은 invalid).
- 직전 기준이 우연히 틀린 가설과 0.10 m 안으로 가까우면 통과한다.
- 판정 기준(0.10 m, 1.0 s)은 시뮬(0.4 m/s, 카메라 약 10 Hz)에서 정했다. 실차 속도·프레임률이 다르면 다시 확인할 것.

## 주요 파라미터 (`config/camera_perception.yaml`)

| 이름 | 기본값 | 설명 |
|---|---|---|
| `image_topic` | `/camera/color/image_raw` | launch의 `image_topic` / `sim` 인자로 바뀜 |
| `lane_width_m` | 0.55 | 선 중심 간 차선 폭. **0.55는 미확인**(규정집 0.55, 도면 0.50, 실물 미측정). 시뮬은 `camera_perception_sim.yaml`에서 0.52 |
| `reference_distance_m` | 0.5 | 기준점의 `base_link` 전방 거리. 판단팀 `lane.reference_distance_m`과 같아야 함 |
| `publish_distance_y` | false | 기존 `/distance_y` 발행 |
| `guard_enable` | true | 옆 도로 선 가드 |
| `cam_height_m`, `cam_x_m`, `cam_pitch_rad`, `focal_px` | 0.21, 0.10, 0.0, 381.0 | 지면 투영. 시뮬 값 |

## 구조

- `ar_lane_perception/lane_core.py`: 검출·lane_error·가드 (ROS 의존 없음, 오프라인 시험 가능)
- `ar_lane_perception/camera_perception_node.py`: ROS 노드 (파라미터, 구독/발행)
