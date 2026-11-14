# 실차 LiDAR IMU pose 변경 전 검토 및 검증 계획

검토일: 2026-11-14. 실행 코드 변경 전 작성한 계획을 보존한 문서다. 이후 구현 내용과 검증 결과는 `lidar_real_imu_runbook.md`를 참조한다. 실차 검증은 아직 수행하지 않았다.

## 확인한 기준

- main: `46e57694b46873193f3e5f77dbba9e2fe2f9b657`
- 작업 브랜치: `feature/lidar-real-imu-yaw` (로컬, main에서 분기)
- PR #22: https://github.com/hotsixthekingrush/KUDOS_Autorace2026_Gazebo/pull/22
- PR head: `lane-imu-1114`, `69ae356cb2ac5ead4dd26403ba8f4f6f9aa8c983`; base는 `lane-rt-width`. 검토 시 open/unmerged.
- PR 전체를 병합하지 않고 적분 방식을 참고한다. main에는 병합하지 않는다.

## 현재 코드와 영향 범위

| 파일/함수 | 현재 동작 | 검토 결과 |
|---|---|---|
| `src/ar_perception/config/lidar.yaml` | LiDAR x=0.1236, y=0 | Gazebo 값 유지 |
| `src/ar_perception/launch/lidar_decision_sim.launch.py` | lidar.yaml + use_sim_time=true | 실차 설정을 읽지 않도록 유지 |
| `lidar_perception.py:odom_callback/get_odom_pose` | 최신 odom pose 그대로 사용 | stamp 검증, IMU, 축 보정 없음 |
| `lidar_perception.py:to_base_link` | scan 점에 LiDAR x/y 오프셋 직접 추가 | TF 변환을 추가하면 같은 이동을 다시 적용하지 않아야 함 |
| `base_to_odom/odom_to_base` | 최신 pose로 왕복 변환 | scan 시각 pose를 한 번 선택해 전체 callback에서 고정할 필요 |
| 주차 경계/주차 공간 추적 | 내부 odom 좌표로 저장 후 차량 좌표로 발행 | 새 pose의 직접적인 영향 범위 |
| 일반 장애물/LIMO 검출 | 현재 scan의 차량 상대좌표 | yaw 변경만으로 검출 흔들림이나 누락이 해결되는 구조는 아님 |
| PR `ar_lane_perception/odom_imu.py` | 최신 IMU yaw + odom v, 중간 yaw 적분, 원점 (0,0), max_dt=0.5 | timestamp 없는 set_imu; stale 검사 없음; 역순 update도 기준시각을 바꿈. LiDAR에 그대로 복사하지 않음 |
| main supervisor `_cb_imu/_dead_reckon/_cb_odom` | 첫 IMU yaw를 0으로, odom v 적분, 중심 오프셋 추가 | params.yaml은 imu/+0.10, params_sim.yaml은 odom/0 |

PR helper는 절대 IMU yaw를 쓰고 supervisor는 시작 yaw를 뺀다. 두 결과를 같은 전역 좌표라고 취급하면 안 된다. PR의 검증 결과는 PR 작성자 보고이며 여기서 재실행한 결과가 아니다.

## 설정 분리안

기존 lidar.yaml은 Gazebo 기본으로 유지하고 명시적으로 use_imu_yaw=false, odom_center_offset_x=0.0을 넣는다. 별도 lidar_real.yaml은 공통 설정 뒤에 덮어쓰는 실차 override로 추가한다. 별도 lidar_real.launch.py는 두 파일을 순서대로 로드하며 센서와 판단 노드를 자동 시작하지 않는다.

| 설정 | Gazebo | 실차 |
|---|---:|---:|
| use_imu_yaw | false | true |
| lidar_offset_x | 0.1236 | 0.1025 |
| lidar_offset_y | 0.0 | 0.0 (좌우 중앙 가정) |
| odom_center_offset_x | 0.0 | 0.10 |
| use_sim_time | true | false (bag 재생 시 true) |
| imu_topic | 미구독 | /imu |

노드 기본 use_imu_yaw=false. 모드는 시작 시 고정하고 실행 중 전환으로 기존 주차 경계 좌표계를 섞지 않는다. 실차에서 IMU 모드를 끄더라도 장착 오프셋과 축 보정은 실차 값을 유지한다.

## pose 및 오프셋 설계

1. 최초 유효한 동기화 쌍에서 뒷축 위치를 (0,0), IMU yaw를 기준 yaw0로 둔다. theta=wrap(yaw-yaw0). 초기 중심 위치는 (0.10,0)이다. 독립적인 내부 로컬 좌표이며 ROS odom TF라고 발행하지 않는다.
2. 뒷축 속도 v=odom.twist.twist.linear.x를 적분한다. 인접 유효 속도의 평균과 shortest-angle 중간 yaw를 사용한다. 후진의 음수 속도를 보존한다. odom pose x/y나 yaw를 이후에 섞지 않는다.
3. 중심 pose는 매 조회 시 p_center=p_rear+R(theta)*(0.10,0). 적분 상태에 매번 0.10을 누적하지 않는다. 회전 시 레버암 이동도 이 식으로 반영된다.
4. 센서 점은 p_body=p_scan+(0.1025,0). 세계 점은 p_center+R(theta)*p_body. 센서 보정과 뒷축 보정은 서로 다른 두 변환이다.
5. supervisor도 raw /odom을 독립적으로 읽어 중심 pose를 만드는 것은 중복 보정이 아니다. 이미 중심으로 보정된 /odom을 입력으로 주거나, LiDAR 점에 0.10을 또 더하면 중복이다. raw /odom 입력 계약을 문서화한다.
6. 실제 TF base_link가 뒷축이라면 중심 좌표에 base_link 이름을 붙이는 것은 부정확하다. 실차 /tf, /tf_static, odom.child_frame_id를 확인해 중심 frame(예: base_center)을 선택하고 메시지/마커에 일관되게 사용한다. 기존 TF를 덮어쓰지 않는다. 판단부가 frame_id를 변환하는지 함께 확인한다.

측정 축간거리 0.205m의 절반은 0.1025m지만 요청한 뒷축 보정은 0.10m로 유지한다. 둘 사이 2.5mm는 알려진 기하 근사다. 설정을 임의로 0.1025로 통일하지 않는다.

## 시간동기화 및 장애 처리

- /imu, /odom, /scan 모두 header.stamp를 사용한다. 수신 시각을 섞지 않는다. 실차 센서가 동일 ROS 시간 기준인지 bag으로 확인한다.
- IMU yaw를 시간순 유한 길이 버퍼에 저장하고 odom 시각을 앞뒤 샘플로 보간한다. ±pi 경계는 shortest-angle로 보간한다. quaternion 유한성/노름 및 orientation_covariance[0]=-1을 검사한다.
- odom을 제한된 시간 대기시켜 뒤쪽 IMU 샘플을 받을 수 있게 한다. scan도 제한된 큐에서 pose 이력을 기다리고 scan 시작 stamp의 pose로 전체 처리를 고정한다. 샘플 간격/대기 한도는 별도 파라미터로 두고 실제 주기를 보고 설정한다(초기 후보 0.1s; 실측 전 확정 아님).
- 미래 pose 또는 무제한 오래된 최신 yaw로 적분하지 않는다. sensor QoS는 IMU publisher의 best-effort와 호환되게 설정한다.
- 중복/역순 샘플은 적분 기준시각을 되돌리지 않고 폐기한다. /clock 되감기/센서 재시작은 명시적으로 새 epoch로 처리한다.
- 장시간 누락, 비정상 stamp, 큰 dt(초기 후보 0.5s)에서는 이동을 추측하지 않는다. pose와 주차 경계/확정 gap을 무효화하고 새 동기화 쌍으로 재초기화한다. 복구 전에 예전 경계를 재사용하지 않는다.
- 해당 scan pose가 없으면 주차 gap은 valid=false로 발행하고 추적 갱신을 건너뛴다. 현재 scan만 쓰는 장애물 검출은 계속할 수 있다. raw odom yaw로 자동 fallback하지 않는다.
- scan의 첫 광선 stamp를 대표 시각으로 쓰는 것은 광선별 deskew가 아니다. 회전 중 한 스캔 내 왜곡은 잔여 오차로 별도 측정한다.

## 변경 예정 파일

- `ar_perception/pose_imu.py`: ROS 독립 동기화/적분/pose 이력과 리셋 상태.
- `ar_perception/lidar_perception.py`: 파라미터, IMU 구독, 큐 처리, scan pose 고정, 주차 상태 무효화, frame 계약.
- `config/lidar.yaml`: Gazebo의 기존 수치와 경로 유지, 모드 명시.
- `config/lidar_real.yaml`, `launch/lidar_real.launch.py`: 실차 전용 설정.
- `test/test_pose_imu.py`, `test/test_lidar_pose_integration.py`: 수치 및 callback/발행 경로 검증.
- 이 문서: 구현 후 실제 실행 명령/시험 결과/실차 frame 확인값 추가.

## 테스트 절차와 판정

1. ROS 없는 수치 시험: 직진/후진, 90도 원호, ±pi 통과, 비영 yaw 시작, 초기 중심 +0.10, 왕복 좌표변환. 직진 1m와 회전 레버암은 해석값과 비교한다. 원호 오차는 샘플 간격을 줄일수록 수렴해야 한다.
2. 시간 시험: IMU 늦게 시작, 지연/역순/중복, stale, scan이 pose보다 빠름, 잘못된 quaternion/NaN v, 긴 누락, clock reset. 누락 구간의 가짜 이동이나 이전 epoch 경계가 없어야 한다.
3. 연결 시험: 가짜 ROS 메시지로 on_scan부터 parking gap 발행까지 확인. 동기화 실패 시 valid=false, 회복 후 확인 프레임 수를 다시 충족해야 한다. 일반 장애물 발행은 유지한다.
4. Gazebo 회귀: 동일 bag을 main과 작업 브랜치 use_imu_yaw=false에서 재생. 장애물/벽/gap의 frame·수치·확정 시점이 동일해야 한다. IMU 구독이 없고 x=0.1236, 중심 보정 0인지 확인한다.
5. 실차 정지 측정: /scan /imu /odom /tf /tf_static와 perception 출력을 기록. 토픽 주기, stamp 차이, IMU 축/좌회전 yaw 부호, base_link 위치, scan 입력이 센서 원본인지 확인한다. 전방 표적의 줄자 거리와 중심 기준 출력 비교.
6. 수동 주행 기록: 직진 1m·후진·좌우 90도·곡선 통과. 같은 bag으로 raw odom과 IMU 모드를 비교하고 정적 벽/주차 경계의 위치 흔들림, gap 유지, 지연과 invalid 비율을 기록한다. 허용 오차는 실측 센서 노이즈와 미션 여유폭에 맞춰 정하며, 결과 없이 통과라고 하지 않는다.
7. 센서 끊김 재생 및 노드 재시작 후 복구 시험. 판단 노드와 서로 다른 시작 yaw여도 차량 상대 출력이 일치하는지 확인한다. 전역 좌표 공유가 필요하면 별도 공통 pose 제공자를 설계한다.
8. 실차 검증 전 main 병합 금지. 회귀/실차 결과와 남은 제한을 기록한 뒤 별도 검토한다.

최초 계획 작성 시 완료: 소스 검토, 별도 로컬 브랜치 생성, 변경/검증 계획. 이후 실행 코드와 자동 시험을 추가했다. 실제 Gazebo 재생, 실차 측정, 원격 브랜치 push는 아직 수행하지 않았다.
