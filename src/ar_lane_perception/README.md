# ar_lane_perception

카메라 영상에서 차선을 찾아 `/perception/lane_error` (`ar_msgs/LaneError`)를 발행한다.
판단팀 `ar_mission_supervisor`가 구독한다. 선 추적은 `limo_ros2_application/detect_line.py`와 같은 로직이다.

> ⚠️ **실차 미검증.** Gazebo 시뮬과 오프라인 합성 영상에서만 확인했다.
> 카메라 높이·피치·초점거리, 색 임계값, 차선 폭은 실물에서 다시 맞춰야 한다.

## 실행

가제보 + 판단 + 차선 한 번에 (YOLO 제외): `ros2 launch ar_lane_perception sim_full.launch.py [color_space:=lab] [gui:=false]`
또는 상위 폴더 `scripts/sim_up.sh` (README 대신 `report_lab_fix.md` 참고).

```bash
colcon build --symlink-install --base-paths src --packages-select ar_msgs ar_lane_perception
ros2 launch ar_lane_perception camera_perception.launch.py            # 실차: /camera/color/image_raw
ros2 launch ar_lane_perception camera_perception.launch.py sim:=true  # 시뮬: /rgb/image_raw
ros2 launch ar_lane_perception camera_perception.launch.py image_topic:=/my/image
```

## 입출력

| 방향 | 토픽 | 타입 | 비고 |
|---|---|---|---|
| 구독 | `image_topic` 파라미터 | `sensor_msgs/Image` | sensor data QoS |
| 발행 | `/perception/lane_error` | `ar_msgs/LaneError` | reliable, depth 10, 영상 프레임마다 |
| 발행 | `debug_image` | `sensor_msgs/Image` | `publish_debug_image: true`일 때만 (기본 꺼짐) |
| 발행 | `/perception/lane_status` | `std_msgs/String` (JSON) | 영상 프레임마다 판정 사유(`reason`), raw_valid, n_one/n_good, 처리 시간. `publish_status` (기본 켬). 분석용 |
| 구독 | `odom_topic` (`/odom`) | `nav_msgs/Odometry` | `predict_enable: true`일 때만 (시뮬 yaml 기본 켬) |
| 구독 | `imu_topic` (`/imu`) | `sensor_msgs/Imu` | `predict_enable: true` 이고 `odom_yaw_source: imu` 일 때만 (실차 기본) |

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
| `single_lane_estimated` | 한 줄이라도 한쪽 선 + 기억한 차선 폭/2 로 중심을 추정했으면 true |

**기준점:** `lateral_offset_m`과 `heading_error_rad`는 차 중심(`base_link` 원점)에서 **앞쪽 `reference_distance_m` = 0.5 m** 지점 기준이다.
**이 값은 판단팀 `ar_mission_supervisor`의 `lane.reference_distance_m`과 같아야 한다.** 한쪽만 바꾸면 제어가 틀어진다.

`valid=false`가 되는 경우: 쓸 수 있는 중심점이 2개 미만, 전방 거리 범위 10 cm 미만, 아래 가드,
또는 `confidence < min_confidence`(0.05). **판단팀은 confidence를 보지 않고 valid만 본다는 전제로**,
믿기 어려운 값은 모두 valid=false로 내린다. 사유는 노드 경고 로그(`lane guard: ...`)에 남는다.

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

## 노란 선 색 필터 (`color_space`)

`color_space: hls`(기본) 또는 `lab`. 기본값 hls는 기존 동작과 결과가 같다. 런타임 변경 가능:
`ros2 param set /camera_perception_node color_space lab` (그 외 값은 거부). `debug_image_num: 0` 마스크도 선택한 색 공간을 따른다.

`lab`: BGR→LAB(OpenCV 8bit, L 0~255, a/b 128 = 무채색) 후 `lab_l_min/max`, `lab_a_min/max`, `lab_b_min/max`로 inRange.

> ⚠️ **LAB 값은 시뮬 기준 임시값이다. 선배(실차) 임계값이 아니다.** 실차 영상으로 다시 맞출 것.

**LAB 시작값 근거** (오프라인 렌더 영상 + Gazebo 실제 카메라 프레임 1장으로 확인, 둘의 색이 같았음):

| 픽셀 | L | a | b |
|---|---|---|---|
| 노란 선 (평면 조명이라 단일 색, BGR 0,252,255) | 246 | 108 | 222 |
| 노란 선 가장자리 (노랑 75 / 50 / 25 %) | 205 / 163 / 119 | 111 / 114 / 119 | 207 / 189 / 165 |
| 바닥 | 72 | 127 | 132 |
| 흰 선 | 255 | 128 | 128 |
| 라바콘 (주황) | 152 | 190 | 197 |

- `lab_b_min` 175: 배경 b 최댓값 133과 노랑 222 사이. 노랑 50 % 이상 가장자리(b 189)까지 넣고 25 %(b 165)는 뺀다 (HLS 기본값의 가장자리 처리와 비슷).
- `lab_a_min/max` 90/140: 노랑 a 106~119와 바닥·흰색 a 126~128은 a로 못 가른다. 상한 140은 라바콘(a 190) 같은 주황·빨강을 빼기 위한 것. (HLS 기본값은 H 0~60이라 라바콘 색도 통과한다.)
- `lab_l_min/max` 100/255: 노랑 50 % 가장자리 L 163보다 충분히 아래. 어두운 바닥은 b로 이미 빠진다.

**hls / lab 오프라인 비교** (기존 정상 주행 로그 11개 `tools/log_*.csv` 포즈, 5457 프레임 렌더, `lane_width_m` 0.52).
정답 = 노랑이 50 % 이상 덮은 픽셀 마스크로 같은 파이프라인을 돌린 결과. 정답이 valid인 5142 프레임 기준.
오차는 정답 대비 |lateral|, |heading| 차이. 밝기 ×0.7 / ×1.3 + 가우시안 노이즈(σ 8)는 **두 필터의 민감도 비교용일 뿐 실제 조명 변화와 같다는 뜻이 아니다.**

| 조건 | 필터 | 마스크 px/프레임 (정답 2140) | 재현율 | valid=false | lat 오차 평균 / p95 [cm] | heading 오차 평균 / p95 [°] |
|---|---|---|---|---|---|---|
| 기본 | hls | 2140 | 1.000 | 0 | 0.00 / 0.00 | 0.00 / 0.00 |
| 기본 | lab | 2140 | 1.000 | 0 | 0.00 / 0.00 | 0.00 / 0.00 |
| ×0.7+노이즈 | hls | 1269 | 0.593 | 7 | 0.09 / 0.26 | 0.49 / 1.33 |
| ×0.7+노이즈 | lab | 2013 | 0.940 | 3 | 0.03 / 0.07 | 0.14 / 0.28 |
| ×1.3+노이즈 | hls | 2407 | 1.000 | 7 | 0.05 / 0.04 | 0.14 / 0.15 |
| ×1.3+노이즈 | lab | 2281 | 1.000 | 1 | 0.02 / 0.03 | 0.09 / 0.12 |

- ×0.7에서 HLS가 떨어지는 이유: 시뮬 노랑의 HLS L이 128이라 ×0.7이면 89로 `lane_l_l` 90 바로 아래가 된다. 노이즈로 절반쯤만 남는다.
  즉 HLS 임계값이 시뮬 색에 여유 없이 붙어 있다는 뜻이고, "LAB가 실차에서 낫다"는 근거는 아니다.
- ×1.3의 정답 밖 픽셀(hls 267, lab 140 px/프레임)은 노랑 25 % 가장자리가 밝아져 선이 조금 두꺼워진 것이다.
- 3D 물체(표지판, 과속방지턱, 라바콘)와 그림자는 렌더하지 않았다.

## 가로선 / 줄무늬 제거, 흰 점선, 선 쌍 가드 (lane-3lane-v1)

### 방지턱 오검출 (판단팀 보고: 방지턱 근처 heading -19°, confidence 0, 차 흔들림)

헤드리스 가제보에서 방지턱(x 3.4~3.7, 노랑 `1 0.8 0` / 검정 줄무늬 10줄)으로 다가가는 실제 프레임을 저장해 재현했다.
**가설이 맞았다:** 줄무늬는 진행 방향과 나란한 5 cm 폭 노란 띠 5개로, 노란 마스크에 그대로 잡힌다
(마스크 픽셀 약 2,700 → 14,000). f2f01ac은 x 3.00~3.25에서 heading +15~+18°, confidence 0.00~0.06, 한쪽 선 추정으로 나왔다
(부호는 차의 좌우 위치에 따라 바뀜). 차가 방지턱 위에 있을 때(x≈3.5)는 방지턱이 화면(0.45~1.1 m 앞) 밖이고,
실제 주행으로 넘을 때 차체 피치는 -3.8~+3.4°였지만 그 프레임 출력은 정상이었다.

처리 (노랑, 흰 마스크 각각):
- **(a) 덩어리:** 연결 덩어리를 지면 좌표로 바꿔 장축 방향이 현재 헤딩(마지막 valid 값)과 `blob_angle_max_deg`(30°) 넘게 다르거나,
  평균 폭(지면 면적/장축 길이)이 `blob_max_width_m`(0.08 m)보다 넓으면 제거. 정지선, 시작 지점 흰 테이프가 여기서 빠진다.
- **(b) 행:** 추적 줄에서 한 구간의 지면 폭이 차선 폭(실측 기억) × `row_width_ratio`(0.6) 이상이거나, 같은 색 구간이
  차선 폭 하나(실측 기억 + `guard_width_tol_m`) 안에 `row_max_runs`(3)개 이상이면 그 줄을 뺀다.
  방지턱 줄무늬는 진행 방향과 나란해서 (a)로는 안 빠지고 (b)로 빠진다.
  "한 줄에 3구간 이상"을 그대로 쓰면 옆 도로 선(약 0.2 m 바깥)이 보이는 정상 줄도 빠져서, 차선 폭 하나 안에 든 구간만 센다.
- **줄을 뺀 뒤의 추적:** 원래 -19°의 직접 원인은 빠진 줄 다음 줄에서 멀리 떨어진 아래 줄 위치로 선을 찾다가 옆 도로 선을 잡은 것이다.
  한 선을 바로 아래 줄에서 못 찾았으면 이번 프레임 점들을 이은 직선으로 연장해(점 1개면 소실점 쪽으로 원근 축소) 찾는다.
  점선 끊김에도 같은 처리가 쓰인다.

### 3차선 (흰 점선)

- 흰 마스크: `white_*` (color_space 따름). 시작값 LAB L 170~255, a/b 118~138 / HLS L 170~255, S 0~60.
  **시뮬 흰 선 실측(LAB 255,128,128) 기준 임시값, 실차 미확인.**
- 흰 선은 그 줄의 노란 선만으로 폭이 맞는 쌍을 못 만들 때만 후보로 쓴다 (일반 구간 정지선 조각, 글자, 마름모 차단).
- **점선 치수 (규정집에 없음, 가제보 월드 `track_paint`(track.png → 메시)에서 읽음, 임시값):**
  길이 0.18~0.19 m, 간격 0.166 m, 폭 약 0.008 m, x 4.30~8.34 (점선 2줄 각 12개, y 0.68 / 1.21). 차선 중심 y 0.43 / 0.95 / 1.47.
  실차 치수는 미확인이다.
- 차선 선택: 이력이 없을 때 지면 간격이 기억한 실측 폭 ± `guard_width_tol_m`인(기억이 없으면 `width_min_m`~`width_max_m` 안인) 이웃 선 쌍 중 차 중심에 가장 가까운 쌍을 잡는다.
  폭이 맞는 쌍이 한 줄도 없는 프레임은 `guard_need_pair`로 직전 정상값과 이어질 때만 valid.
- valid=false가 `lost_hold`(5) 프레임 넘게 이어지면 선 추적 이력을 지우고 다시 잡는다 (잘못 잡은 선에 갇히는 것 방지).

### 가드 추가 (C)

- 폭이 맞는 선 쌍이 없는 프레임(한쪽 선 추정만): 인접 차선 선일 수 있으므로 직전 정상값과 lateral 0.10 m, heading
  0.35 rad(≈ 20°) 안으로 이어질 때만 valid. 사유 `single_no_history` / `single_jump` / `single_head_jump`.
- 기존 애매한 프레임(폭 안 맞는 쌍 + 한쪽 추정)도 heading 연속성 검사 추가 (`ambiguous_head_jump`).
- `confidence < min_confidence`(0.05)이면 valid=false (`low_confidence`). 11개 로그에서 confidence 0.05 미만 valid 프레임의
  대부분이 heading 10° 넘게 틀린 프레임이었다. 메시지 정의는 바꾸지 않았다.

### 차선 갈라짐 (3차선 서쪽 섬 끝)

가운데 차선은 섬 끝(x 4.267)에서 끝난다. x ≤ 5.04부터 카메라에 섬의 갈라지는 노란 선이 보여 heading 최대 26° 오차,
`lane_width_m` 0.55에서는 섬을 지나며 옆 차선을 잡아 lateral 54 cm 오차(valid)가 나왔다.
- 흰 선을 차선 선으로 쓴 줄이 `fork_min_white_rows`(2) 이상인 프레임에서, 직전 정상 차선 띠
  (중심 ± 실측 폭/2 − `fork_margin_m`(0.08 m)) 안쪽에 노란 구간(제거 전 마스크, (b)로 뺀 줄 제외)이 있는 줄이
  `fork_min_rows`(2) 이상이면 valid=false (`lane_fork`).
- 그 뒤 갈라짐 직전 차선과 lateral 0.10 m 안으로 다시 이어지거나 `fork_timeout_sec`(3 s)가 지날 때까지 valid=false (`fork_hold`).
- 흰 선 조건이 없으면 코너에서 굽은 노란 선이 직선 띠 안으로 들어와 오탐이 났다 (11개 로그에서 283프레임 손실). 흰 선 2줄 조건으로 0.

전 구간(x 9.4→4.0, 3차선 × 9조건, 장애물 없음) 재시험: valid인데 15 cm 넘게 틀린 프레임 0개 (`lane_width_m` 0.52, 0.55 모두).
가운데 차선 x ≤ 5.04 heading 최대 25.8° → 1.7° (0.52), 바깥 차선은 x ≤ 5.04에서 heading 최대 9°(lateral ≤ 4.9 cm)가 남는다.

### 3차선 시작 (x 9.4 출발) 오차

원인: 테이프 앞 넓어지는 구간의 비스듬한 노란 외곽선(x 8.05~8.6)이 한쪽 선 추정으로, 흰 점선의 맞는 쌍과 섞여 헤딩을 끌어당겼다
(x 9.00 한 프레임 heading -30.6°, lateral 13.7 cm, 이력 있는 프레임). 흰 테이프는 그 줄을 빼서 invalid를 만들 뿐 틀린 값은 만들지 않았다.
- `single_tol_m`(0.05) / `single_conflict`(invalid): 흰 선을 쓴 프레임에서 맞는 쌍의 중심선과 어긋나는 한쪽 선 추정 줄이 섞이면 valid=false
  (`single_conflict`). 일반 코너에서는 굽은 선 때문에 오탐이 나서(코너 valid 160→36) 흰 선 프레임에만 적용.
- `confirm_head_rad`(2°) / `confirm_lat_m`(0.03): invalid 구간 바로 뒤 프레임은 직전 추정과 맞을 때만 valid (`confirm`).
  invalid 구간마다 1프레임 늦게 valid가 된다.

### 오프라인 / 가제보 확인 결과 (시뮬 전용, 실차 미검증)

GT: 3차선·직선은 차선 중심선 해석값, 로그는 트랙 그림의 노란 선 사이 골격. 렌더러는 평면 방지턱, 그림자·3D 물체 없음.

| 시험 | f2f01ac | 이번 |
|---|---|---|
| 가제보 방지턱 접근 (x 1.6~4.2, 0.05 m) | heading 최대 4.0° (x 3.00~3.25 +15~+18°는 가드로 invalid) | 최대 0.3°, valid 15/17 (방지턱 구간) |
| 가제보 방지턱 yaw ±5°, 옆 ±5 cm | heading 최대 5.0° | 최대 0.8° |
| 가제보 실제 주행으로 방지턱 넘기 (2회, 약 3 Hz) | heading 최대 4.3° | 최대 1.0°, invalid 1프레임씩 |
| 가제보 직선 yaw 0/±5/±10°, 옆 0/±5/±10 cm (25) | lat 최대 0.87 cm, heading 최대 0.46° | 같음. 이력 없는 단일 프레임 2개(yaw 10° + 옆 10 cm) invalid |
| 렌더러 3차선 x 9.4→5.8 연속 (전 구간 9.4→4.0 결과는 위 '차선 갈라짐' 참고), 3차선 × yaw 0/±5/±10 × 옆 ±5/±10 cm (27조건) | 가운데 차선 invalid 또는 lat 47~52 cm 오차, 바깥 차선 51~52 cm (가운데 차선을 잡음) | 점선 구간 valid 1728/1728, 깜빡임 0, lat 최대 1.0 cm, heading 평균 ≤0.2° 최대 0.6° |
| 같은 시험, `lane_width_m` 0.55 | - | valid 1646/1728, 깜빡임 127, lat 최대 2.3 cm, heading 최대 2.5° |
| 가제보 정차 LIMO 뒤 0.6~1.0 m | - | valid, lat ≤1.6 cm, heading ≤2.0° (1개 5.5°) |
| 가제보 정차 LIMO 뒤 0.45 m | - | 세 차선 모두 invalid |
| 11개 로그: 일반 직선 3108프레임 | valid 3102, lat 평균 0.08 cm, heading 평균 0.55° | valid 3083, lat 0.07 cm, heading 0.42° |
| 11개 로그: 코너 196 | valid 196, heading 평균 7.0° (최대 68°) | valid 160, heading 평균 3.1° (최대 26°) |
| 11개 로그: 15 cm 넘게 틀린 valid 프레임 | 3차선 구간 15 (가운데 차선 오인) + 분류 밖 16 (미확인, GT 오차 섞였을 수 있음) | 0 (골격 GT로 7개 걸렸으나 3차선 해석 GT로 다시 보면 모두 1 cm 이내) |
| 지그재그 (가제보 27프레임) | valid 27/27, heading 오차 ≤5° | valid 15/27, 오차 ≤9° — **목표(valid=false) 미달** |

**한계:** x 9.4 (3차선 시작 전 넓어지는 구간)에서 이력 없이 시작하면 처음 몇 프레임은 invalid이고 첫 valid 프레임의 heading 오차가
최대 3.8°다 (점선 구간 안에서는 ≤0.6°). 지그재그는 일부 프레임만 invalid가 된다. 판정 기준값은 모두 시뮬에서 정했다.

## odom 예측 발행 (lane-lab-fix, 시뮬 기본 켬 / 실차 기본 끔)

**현상 (가제보 폐쇄루프 로그로 확인, 상위 `report_lab_fix.md`):** 이 PC 가제보는 GPU 없이 llvmpipe로 렌더해 카메라가
3~4 Hz(간격 최대 1.7 s)였다. 판단(`ar_mission_supervisor`)은 받은 값이 오래될수록 pure pursuit 기준 거리를
`0.5 - v·age`로 줄여 조향을 키우고(값 자체는 차가 돈 만큼 갱신 안 됨), 0.5 s 넘게 새 값이 없거나 valid=false면
직전 명령을 1 s 유지(hold)한 뒤 0.12 m/s 직진(creep), 5 s 뒤 정지한다. 그래서 차선 추정이 1 cm 안으로 맞는데도
직선에서 yaw ±8° 넘게 휘청이다 hold 때 최대 조향으로 트랙을 벗어났고, invalid가 이어진 경우 방지턱 앞에서 멈췄다.
hls / lab 모두 같았다 (색 공간 문제가 아님).

**처리 (`predict_enable`):**
- valid 프레임의 차선 중심선을 그 촬영 시각의 odom 자세로 odom 좌표에 고정해 둔다.
- `predict_rate_hz`(20) 타이머마다 최신 odom 자세 기준 lateral / heading으로 옮겨 발행한다. `header.stamp` = 그 odom 시각.
- 카메라 프레임이 invalid이거나 안 와도 마지막 valid 뒤 `predict_max_sec`(1.0 s)까지는 옮긴 값을 valid로 발행.
  그 뒤 invalid 1번 발행. (판단의 hold/creep 대신 차선 기하를 따라감)
- 예측을 켜면 영상 콜백은 직접 발행하지 않는다 (오래된 stamp 값과 섞이지 않게). odom이 없으면 예전처럼 영상마다 발행.
- 가드(`*_jump`, `*_head_jump`)와 시작 확인(`confirm`)도 직전 값을 odom으로 이번 프레임 자세에 옮겨 비교한다
  (예전엔 프레임 사이 차가 몇 도씩 돌아 `confirm`이 계속 떨어졌다).
- `single_updates_ref: false` (시뮬): 폭 맞는 쌍이 없는 한쪽 선 프레임은 가드 기준을 갱신하지 않는다. 예전에는
  방지턱 위 틀린 한쪽 선 프레임이 0.10 m씩 기준을 끌고 가 GT 대비 19~26 cm 틀린 값이 valid로 나갔다 (base_hls_1).
- 비교용 예전 동작: `config/sim_baseline.yaml` (`predict_enable: false`, `single_updates_ref: true`)
  → `ros2 launch ar_lane_perception sim_full.launch.py lane_extra:=<그 경로>`.

**한계 (확인함):** 오른쪽 위 코너(x 8.5~9.5)는 안쪽 선이 x≈8.55부터 화각(가까운 쪽 0.45 m, 80°) 밖으로 나가고
바깥 선은 바깥으로 불룩해서, 바깥 선 − 반폭 중심이 틀리고(+0.09 m, 실제는 오른쪽) confidence가 0이 된다. 수정 전 파라미터로도
같은 프레임에서 같은 결과다 (오프라인 재생). 수정 전 폐쇄루프는 그 전에 트랙을 벗어나 여기까지 오지 못했다. 미해결.
실차 미검증 (실차 /odom = 바퀴 odom).

## 주요 파라미터 (`config/camera_perception.yaml`)

| 이름 | 기본값 | 설명 |
|---|---|---|
| `image_topic` | `/camera/color/image_raw` | launch의 `image_topic` / `sim` 인자로 바뀜 |
| `reference_distance_m` | 0.5 | 기준점의 `base_link` 전방 거리. 판단팀 `lane.reference_distance_m`과 같아야 함 |
| `guard_enable` | true | 옆 도로 선 가드 |
| `color_space` | hls | 노란 선 색 필터 `hls` / `lab`. 런타임 변경 가능 |
| `lab_l_min/max`, `lab_a_min/max`, `lab_b_min/max` | 100/255, 90/140, 175/255 | `lab`일 때만 사용. **시뮬 기준 임시값** |
| `white_enable`, `white_*` | true, LAB 170~255/118~138/118~138, HLS L 170~255 S 0~60 | 흰 점선. **시뮬 기준 임시값** |
| `clean_enable` | true | 가로선/줄무늬 제거 |
| `blob_angle_max_deg`, `blob_min_len_m`, `blob_max_width_m` | 30, 0.08, 0.08 | (a) 덩어리 제거 |
| `row_width_ratio`, `row_max_runs` | 0.6, 3 | (b) 행 제외 |
| `guard_need_pair` | true | 한 선 프레임은 최근 실측 폭이 있을 때만 valid |
| `min_confidence` | 0.05 | 이보다 낮으면 valid=false. 0이면 끔 |
| `single_tol_m`, `single_conflict` | 0.05, invalid | 흰 선 프레임의 쌍/한쪽 선 충돌 → valid=false |
| `confirm_head_rad`, `confirm_lat_m` | 0.035, 0.03 | invalid 뒤 첫 프레임 확인. 0이면 끔 |
| `fork_enable`, `fork_margin_m`, `fork_min_rows`, `fork_min_white_rows`, `fork_timeout_sec` | true, 0.08, 2, 2, 3.0 | 차선 갈라짐(섬 끝) valid=false. **시뮬 기준 임시값** |
| `cam_height_m`, `cam_x_m`, `cam_pitch_rad`, `focal_px` | 0.21, 0.10, 0.0, 381.0 | 지면 투영. 시뮬 값 |
| `predict_enable`, `odom_topic`, `predict_rate_hz`, `predict_max_sec` | false (시뮬 true), /odom, 20, 1.0 | odom 예측 발행 |
| `odom_yaw_source`, `imu_topic` | imu (시뮬 yaml 은 odom), /imu | 예측에 쓰는 차 방향. imu = /odom 속도 + IMU 방향으로 적분 (실차 WeGo 아커만 /odom 방향이 돌 때 약 30% 커서, 11/14 판단팀) |
| `single_updates_ref` | true (시뮬 false) | 한쪽 선 프레임의 가드 기준 갱신 |
| `publish_status` | true | `/perception/lane_status` |

## 구조

- `ar_lane_perception/lane_core.py`: 검출·lane_error·가드 (ROS 의존 없음, 오프라인 시험 가능)
- `ar_lane_perception/camera_perception_node.py`: ROS 노드 (파라미터, 구독/발행)

## 차선 폭 실시간 측정 (lane-rt-width)

고정 폭 `lane_width_m`, `lane_w_bottom`, `horizon_y`는 쓰지 않으며 파라미터도 지웠다 (11/14 정리).

- 두 선이 같은 줄에서 함께 보이는 프레임에서만 폭(줄별 픽셀 폭 + 미터 폭)을 재서 기억한다 (`width_alpha`로 완만하게 갱신).
- 프레임 폭이 `width_min_m`~`width_max_m`(0.40~0.95) 밖이거나 기억값에서 `width_max_step_m`(0.03) 넘게 바뀌면 기억을 갱신하지 않고,
  추적 기록과 이어지는 선 하나만 남겨 한 선 프레임으로 처리한다 (`width_note=width_reject`).
- 한 선 프레임: 보이는 선에 맞춘 직선을 반폭만큼 옮긴 경로. `valid=true`, `single_lane_estimated=true`, `confidence=single_confidence`(0.5).
- 고정 폭은 한 선 프레임의 두 경우에만 쓴다. (가) 실측이 없거나 `width_stale_sec`(3초) 넘게 지남: `lane_width_default_m`.
  (나) 보이는 선이 휘어 있음(곡률 `corner_curv_min`~`corner_curv_full`): `max(기억 폭, lane_width_corner_m)` 쪽으로 `corner_blend_per_sec` 속도로 옮김. 코드 기본값은 0.0(끔, 11/13 0.75에서 바꿈). safe3_*/step0_*도 0.0이다.
- 한 선 -> 두 선 전환 때만 lateral 변화를 `pair_return_rate_mps`(0.3 m/s)로 제한한다.
- 없앤 판정: 픽셀 비율 관문의 고정 폭 기준, `ambiguous_*`, `single_no_history/_jump/_head_jump`. `guard_need_pair`는
  "한 선 프레임은 최근 실측 폭이 있을 때만 valid"(`single_no_width`)로 바뀌었다.
- `/perception/lane_status`에 `kind`(pair/single), `width_meas`, `width_mem`, `width_used`, `fixed`(a=가, b=나), `width_note`, `slew`, `lat_raw`가 추가됐다.
  노드 로그에 `width_log_sec`(5초)마다 고정 폭 사용 프레임 수(가/나)가 나온다.
- `lane_extra`는 쉼표로 여러 yaml을 줄 수 있다 (뒤가 우선). 예: `lane_extra:=config/step0_hls.yaml,config/sim_hls.yaml`

## 코너 바깥 선 거리 corner_outer_offset_m (11/13, 기본 0 = 끔)

- 한 선 프레임에서 보이는 선이 코너 바깥 선일 때만 쓴다(오른쪽으로 휘는 왼쪽 선, 왼쪽으로 휘는 오른쪽 선). 이때 그 선에서 `corner_outer_offset_m`만큼 떨어진 경로를 쓴다.
  옮기는 정도는 (나)와 같은 비율 `corner_b`를 쓴다(보이는 선 곡률 `corner_curv_min` 0.3 ~ `corner_curv_full` 1.5 1/m, 1초에 최대 `corner_blend_per_sec`).
  그 비율에 비례해 기존 반폭에서 이 값으로 서서히 옮긴다. 직선과 두 선 프레임은 그대로다.
- 실행 중에 바꿀 수 있다: `ros2 param set /camera_perception_node corner_outer_offset_m 0.35`
- `/perception/lane_status`: `outer_off`(이 프레임에 적용됨), `corner_b`. 쓴 폭은 `width_used`(= 2 × 바깥 선 거리), 고정 폭 표시 `fixed`에 `o`가 붙는다.
- 실차 코너 폭은 최대 0.60 m다. 0.35로 두면 안쪽 선과 0.25 m 떨어진다.

## 예측 (predict_enable, safe3 에서 켬)

- `predict_max_sec` (1.0): 마지막 valid 카메라 프레임 뒤 이 시간까지 odom으로 옮긴 값을 낸다.
- `predict_confidence` (0.25): 예측 중(마지막 카메라 프레임이 invalid) LaneError confidence.
- `predict_path` (true): 예측 중에는 곡률 하나로 계속 휘게 하지 않는다. 마지막 valid 프레임의 가운데 경로 점들을 odom 좌표에 고정해 둔다.
  그 점들을 차 기준으로 옮긴 뒤 `reference_distance_m` 지점의 y × cos(경로 방향)을 lateral로 다시 잰다(legacy와 같은 정의).
  - 가운데 경로 점: 두 선 프레임은 줄별 가운데 점이다. 한 선 프레임은 보이는 선 점들을 맞춘 직선의 수직 방향으로 반폭만큼 통째로 옮긴 점이다(휜 모양은 그대로).
  - 점 범위를 넘어가야 할 때만 끝 구간 방향으로 직선 연장한다. 연장이 `predict_ext_max_m`(0.2 m)를 넘으면 confidence에 `predict_ext_conf_scale`(0.5)을 더 곱하고 예측은 계속한다.
  - false면 예전처럼 직선 하나를 옮긴다. 카메라 프레임 사이(마지막 프레임 valid)는 예전처럼 직선을 옮긴다.
- `/perception/lane_status`:
  - 카메라 프레임마다 `src: camera`, `predicting`(이 프레임이 invalid이고 예측 값이 나가는 중), `predict_sec`.
  - 예측으로 낸 값마다 `src: predict`, `predict_sec`(마지막 valid 뒤 지난 시간), `pred_beyond`(저장 범위를 넘어 연장), `pred_ext_m`, `lat`, `conf`.

## 가운데 경로와 디버그 표시

- 가운데 점들(한 선 프레임이면 보이는 선)에 직선 하나를 맞추고, `reference_distance_m`(0.5) 지점의 수직 거리를 lateral로 쓴다(한 선이면 ± 반폭).
  한 선 프레임의 이동은 맞춘 직선에 수직인 방향이다(화면 가로 이동이 아님).
- 디버그 영상: 자홍 점과 자홍 선은 실제 계산에 쓰는 가운데 경로 점, 흰 선은 보이는 선 점에서 옮긴 점까지,
  빨간 점은 lateral 을 읽는 0.5 m 목표점(가운데 직선의 `reference_distance_m` 지점)이다.
  셋째 줄 글자는 `single|pair line ang <선 각도>`다.
- `/perception/lane_status`: `line_angle_deg`(한 선이면 보이는 선, 두 선이면 가운데 점에 맞춘 직선의 각도).
- 11/13에 시험한 꺾은선 읽기(`single_path_mode: polyline`)는 가제보 기록에서 코너 효과가 없었고 지그재그에서 lateral이 흔들려 지웠다(11/14).
  줄별 픽셀 가운데(파란 점)와 `/distance_y` 발행도 lateral 계산에 쓰이지 않아 지웠다.
- 가제보 기록 결과: 코너 한 선에서 line과 거의 같다(Pure Pursuit 필요 대비 24% -> 25%). 코너에서 lateral이 작은 주원인은 바깥 선에서 옮기는 반폭이 실제 코너 폭보다 작은 것이다. 보고: `~/scalecar2026/tools/report_polyline.txt`
