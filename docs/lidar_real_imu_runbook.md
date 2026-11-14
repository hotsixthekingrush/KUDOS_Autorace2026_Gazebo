# 실차 LiDAR IMU 모드 실행 및 검증

작업 브랜치: `feature/lidar-real-imu-yaw`. 기준 main: `46e57694b46873193f3e5f77dbba9e2fe2f9b657`.
실차 검증 전 main 병합 금지. 이 변경은 LiDAR 인지에만 적용하며 차선 PR #22와 판단 노드 코드는 수정하지 않는다.

## 이 작업에서 확인한 결과 (2026-11-14)

- ROS Humble 의존 패키지 포함 5개 빌드 성공, 마지막 수정 후 ar_perception 재빌드 성공.
- 자동 테스트 **34 passed**: 수치 23개, ROS callback/실제 토픽 시험 10개, main 출력 비교 1개.
- Gazebo 모드 합성 스캔 30개에서 장애물·이벤트·벽·주차 gap·ROI 마커·클러스터 마커의 메시지 필드가 기준 main과 모두 동일. 실제 검출과 주차 gap 확정이 발생한 입력으로 검사했다.
- 실차 launch 파일 로딩과 use_sim_time 인자 확인, Python 문법 검사, git diff 공백 검사 통과.
- 환경의 SciPy/NumPy 지원 버전 범위 경고 1건이 있었으며 위 테스트는 모두 통과했다. 의존성 버전은 이 변경에서 수정하지 않았다.
- 실제 Gazebo 월드 실행/실측 bag 재생/실차 측정은 미실시. 원격 push 및 main 병합 미실시.

## 동작과 좌표 계약

| 항목 | 기존 Gazebo 실행 | 실차 전용 실행 |
|---|---|---|
| 설정 | lidar.yaml | lidar.yaml → lidar_real.yaml |
| use_imu_yaw | false | true |
| LiDAR x/y (m) | 0.1236 / 0 | 0.1025 / 0 |
| 뒷축 → 중심 (m) | 0 | 0.10 |
| 출력 frame | base_link | base_center |
| pose | 최신 raw odom pose (기존 경로) | scan stamp에 맞춘 IMU/속도 적분 pose |

실차 입력 `/scan`은 센서 원점의 원본 LaserScan, `/odom`은 뒷바퀴 축 기준 raw odometry여야 한다. `twist.linear.x`는 차체 전방 속도이며 후진은 음수다. IMU는 몸체와 같은 평면 yaw 회전 방향(좌회전 +)이어야 한다. 다른 축 장착의 3차원 보정이나 scan 광선별 deskew는 구현 범위 밖이다.

최초 유효한 IMU/odom 쌍의 yaw를 0으로 놓고 뒷축 위치 (0,0)부터 적분한다. 차체 중심은 `p_rear + R(yaw)*(0.10,0)`이고 센서 점은 `p_scan+(0.1025,0)`이다. 두 이동을 서로 다른 곳에 한 번씩 적용한다. 이 내부 좌표는 ROS `/odom`이나 TF로 발행하지 않는다. 판단 노드와 시작 시각이 달라도 현재 메시지는 차량 상대좌표로만 전달되므로 전역 원점 공유를 요구하지 않는다.

판단 노드가 raw odom에 자체 +0.10m를 적용하는 것은 별도 계산이다. 이미 중심 보정된 odom을 이 노드 입력으로 연결하거나 출력 점에 추가 +0.10m를 더하지 않는다. 실제 축간 0.205m와 요청한 뒷축 보정 0.10m 사이의 2.5mm 근사는 유지했다.

`output_frame`은 장애물·벽·주차 gap·모든 디버그 마커에 동일하게 적용된다. 현재 main supervisor의 장애물 구독은 frame을 TF 변환하지 않고 상대좌표 값만 읽는다. 다른 소비자가 base_link를 하드코딩했다면 해당 소비자도 중심 기준임을 확인해야 한다.

### 실차 TF 확인

먼저 `/odom.child_frame_id`, `/tf`, `/tf_static`으로 실제 기준점을 확인한다. 이 노드는 TF를 새로 발행하거나 기존 TF를 덮어쓰지 않는다.

**base_link가 실제 뒷축이고 base_center가 아직 없는 것이 확인된 경우에만** 별도 터미널에서 다음 정적 변환을 사용할 수 있다.

```bash
ros2 run tf2_ros static_transform_publisher --x 0.10 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 --frame-id base_link --child-frame-id base_center
```

이미 중심 frame이 있다면 real YAML의 output_frame을 그 이름으로 맞춘 뒤 재시작한다. TF용 오프셋은 좌표 frame 관계를 나타내며, scan 점에 다시 더하는 계산이 아니다. raw odom TF가 과장된 yaw를 사용하면 RViz의 odom 고정 화면은 여전히 틀어질 수 있다. 인지 상대좌표 검증은 base_center 고정 화면으로 하고, 이 노드가 전역 TF까지 보정했다고 해석하지 않는다.

## 빌드와 실행

저장소 루트에서 실행한다. 실차에서는 작업 브랜치의 파일이 먼저 배치되어 있어야 한다. 이번 작업의 원격 push 여부는 작업 결과를 확인한다.

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to ar_perception
source install/setup.bash
ros2 launch ar_perception lidar_real.launch.py
```

실차 launch는 인지 노드만 시작한다. `/scan`, `/imu`, raw `/odom`을 내는 기존 센서 드라이버는 별도로 실행한다. 기존 LiDAR 인지 노드를 중복 실행하지 않는다. Gazebo 실행 경로는 그대로다.

```bash
ros2 launch ar_perception lidar_decision_sim.launch.py
```

실차 설정 확인:

```bash
ros2 param get /lidar_perception use_imu_yaw
ros2 param get /lidar_perception lidar_offset_x
ros2 param get /lidar_perception odom_center_offset_x
ros2 param get /lidar_perception output_frame
ros2 topic hz /imu
ros2 topic hz /odom
ros2 topic hz /scan
```

모드/기하 파라미터는 실행 중 변경을 거부한다. 변경하려면 설정 파일을 고치고 노드를 재시작한다. 실차에서 use_imu_yaw=false로 비교할 때도 LiDAR 0.1025와 뒷축 0.10은 유지한다. 이 비교 모드는 raw odom의 기존 시각 처리 경로를 사용한다.

## 동기화와 실패 처리

- 센서 header.stamp를 사용하고, 수신시각은 대기 제한에만 사용한다.
- IMU yaw를 odom stamp에서 보간하고, 인접 속도의 평균과 중간 yaw로 뒷축 pose를 적분한다. 그 pose 이력을 scan 시작 stamp에서 보간한다. 미래/과거 외삽은 하지 않는다.
- 초기 제한: IMU 샘플 최대 간격 0.1초, odom 적분 최대 간격 0.5초, 동기화 대기 0.1초, 센서 stamp와 노드 ROS 시각 허용 차이 1초. 실제 주기를 보고 조정한다. ±pi를 지날 때는 짧은 회전 방향으로 보간한다.
- IMU orientation 미제공 표시, 잘못된 quaternion, NaN 속도, 누락 구간에서는 적분 이력과 주차 경계를 무효화한다. 다음 유효 쌍에서 새 원점을 만들고 주차 gap을 다시 확인한다.
- scan pose가 없으면 주차 gap은 valid=false, confirmed=false, gap_id=0이다. 장애물/벽의 scan 상대 검출은 계속 발행된다. 모든 scan 처리는 동기화를 기다리므로 정상 시에도 최대 대기 설정만큼 지연될 수 있다.
- 중복/역순 센서 stamp는 버린다. ROS clock이 역행하면 큐와 좌표 이력을 초기화한다. 센서만 독립적으로 재시작하여 stamp가 되감기는 경우에는 노드를 재시작하고 시간 기준을 맞춘다. timestamp가 맞더라도 잘못된 IMU yaw 리셋/점프를 자동 판별하지는 않는다.
- IMU/pose 이력은 각각 400개, odom 대기는 400개, scan 대기는 100개로 제한된다. 높은 센서 주기나 장시간 CPU 정체에서는 유효 이력이 부족해질 수 있으며 외삽 대신 주차 결과가 무효화된다.

## 자동 검증 재현

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=97 PYTHONPATH=src/ar_perception:$PYTHONPATH python3 -m pytest -q src/ar_perception/test/test_pose_imu.py src/ar_perception/test/test_lidar_pose_integration.py src/ar_perception/test/test_lidar_main_regression.py
```

- 수치 테스트: 직진/후진, 가속, 원호, 초기 yaw, 레버암, ±pi, 데이터 지연·누락·역순·리셋.
- ROS 테스트: 실제 메시지 callback, 주차 무효화/재확정, 출력 frame, 동적 변경 차단, best-effort IMU의 실제 ROS 토픽 송수신.
- main 비교: 30개 합성 스캔(벽·라바콘·차량 크기 원)을 동일하게 입력하고 6개 출력의 메시지 필드 전체 비교. 기준 commit이 로컬에 없으면 해당 시험은 skip된다. 실제 Gazebo 실행이나 실측 bag 시험을 대체하지 않는다.

## 실차 및 Gazebo 후속 검증

1. 실차를 정지시킨 상태에서 TF 기준점, 좌우 중앙 장착 가정, IMU 좌회전 부호를 확인한다. 전방 표적의 중심 기준 줄자 거리와 인지 좌표를 비교한다.
2. 센서 및 출력 기록:

```bash
ros2 bag record -o lidar_imu_check /scan /imu /odom /tf /tf_static /perception/obstacles /perception/parking_walls /perception/parking_gap
```

3. 수동 직진 1m·후진·좌우 90도·곡선 이동을 기록한다. 정적 벽과 주차 경계의 흔들림, invalid 비율, 출력 지연, gap 재확정 여부를 비교한다.
4. 기록된 인지 출력을 다시 입력하지 않도록 센서 토픽만 1배속 재생한다. TF는 필요 시 별도로 재생/구성한다.

```bash
ros2 launch ar_perception lidar_real.launch.py use_sim_time:=true
ros2 bag play lidar_imu_check --clock --rate 1.0 --topics /scan /imu /odom
```

5. IMU 중단/복구와 bag 되감기를 시험한다. 과거 주차 gap이 valid 상태로 재사용되지 않아야 한다. 서로 다른 시점에 판단/인지 노드를 시작해도 차량 상대 출력의 의미가 같아야 한다.
6. Gazebo의 동일 bag을 기준 main과 작업 브랜치에서 비교하고 실제 주차 미션도 재생한다. 이번 자동 회귀는 합성 입력만 검증했다.
7. 실측 허용 오차는 미션 여유폭과 센서 노이즈를 기준으로 정한다. 이 결과가 나오기 전에는 실차 검증 완료 또는 main 병합 가능으로 표시하지 않는다.
