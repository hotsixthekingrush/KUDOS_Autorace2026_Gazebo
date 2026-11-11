# limo_ros2_application

LIMO 차선 주행용 ROS 2 (Humble) 노드 모음. 이 문서는 라인트레이싱 노드 `detect_line`만 다룬다.

> ⚠️ **실차 미검증.** 아래 내용은 모두 Gazebo 시뮬(kudos_arena 트랙)에서만 확인했다.
> 카메라 높이·피치·초점거리, 색 임계값, 차선 폭은 실물에서 다시 맞춰야 한다.

## detect_line

카메라 영상에서 좌/우 차선을 찾아 두 가지를 발행한다.

| 토픽 | 타입 | 설명 |
|---|---|---|
| `distance_y` | `std_msgs/Int32` | 기존 출력. 화면상 차선 중심과의 차이(px). `limo_control`이 사용 |
| `/perception/lane_error` | `ar_msgs/LaneError` | 차선 중심선 대비 횡오차·방향오차 (아래 참고). **`ar_msgs`가 있을 때만 발행** |
| `debug_image` | `sensor_msgs/Image` | 디버그 영상 |

### `/perception/lane_error`와 `ar_msgs`

- `ar_msgs/LaneError`는 판단팀 메시지 패키지다. **이 저장소에는 포함하지 않는다.**
- `ar_msgs`를 빌드해서 source한 환경에서만 `/perception/lane_error`가 나온다.
  없으면 노드는 `ar_msgs 없음: /perception/lane_error 비활성` 경고만 내고 `distance_y` 등 기존 출력은 정상 동작한다.
- 그래서 `package.xml`에도 `ar_msgs` 의존성을 넣지 않았다 (없어도 빌드·실행 가능).
- 끄려면 `publish_lane_error:=false`.

필드 규약:

- **`lateral_offset_m`과 `heading_error_rad`는 차 중심(`base_link` 원점)에서 앞쪽 0.5 m 지점 기준이다.**
  (`reference_distance_m` 기본값 0.5. 0으로 하면 `base_link` 원점 기준)
- `lateral_offset_m` [m]: 기준점에서 차선 중심선까지 수직거리. 중심선이 **왼쪽이면 +**
- `heading_error_rad` [rad]: 차선 중심선 방향 − 차 진행방향. 차선이 **왼쪽으로 꺾이면 +**
- `valid`: false면 이번 프레임 값은 쓰지 말 것 (쓸 수 있는 중심점 2개 미만, 또는 전방 거리 범위 10 cm 미만)
- `confidence`: 0~1 품질 지표. 확률로 보정된 값이 아니다.
- `header.stamp`: 카메라 영상 촬영 시각, `header.frame_id`: `base_link`

### 주요 파라미터

| 이름 | 기본값 | 설명 |
|---|---|---|
| `lane_width_m` | 0.55 | 선 중심 간 차선 폭. **0.55는 미확인 값**(규정집 0.55, 도면 0.50, 실물 미측정). 시뮬 트랙 실측은 0.52이고 `params/detect_line_sim.yaml`에서 덮어쓴다. 한쪽 선만 보일 때 중심 추정에 쓰이므로, 틀리면 lateral이 (설정 − 실제)/2만큼 치우친다 |
| `reference_distance_m` | 0.5 | lane_error 기준점의 `base_link` 전방 거리 [m] |
| `publish_lane_error` | true | `/perception/lane_error` 발행 여부 |
| `cam_height_m`, `cam_x_m`, `cam_pitch_rad`, `focal_px` | 0.21, 0.10, 0.0, 381.0 | 카메라 → 지면 투영. 시뮬 기준값, 실차에서 재측정 필요 |

파라미터 파일:

- `params/detect_line.yaml`: 공통 (실차 launch도 이 파일 사용)
- `params/detect_line_sim.yaml`: 시뮬 전용 덮어쓰기 (`lane_width_m: 0.52`)

### 알려진 한계

- 실차에서 한 번도 돌려보지 않았다.
- 차선 폭 0.55 m는 확인되지 않은 값이다.
- 자기 차선의 한쪽 선이 안 보일 때 옆 구간의 선을 잡으면 lateral이 크게 틀릴 수 있고, 이때 `confidence`가 낮아지지 않는다.
- 지그재그 구간과 두 번째 코너 이후는 검증하지 않았다.
