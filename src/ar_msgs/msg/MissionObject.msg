# YOLO 미션 객체 1개 — yolo_detector_node -> mission_supervisor   (2026 규정 기준)
#
# 원칙은 LaneError 와 동일:
#   - 픽셀 좌표는 노드 밖으로 내보내지 않는다. 위치는 미터.
#   - 좌표계는 base_link (x 전방, y 좌측 양수) — LaneError 부호 규약과 일치.
#
# class 상수는 학습 클래스 "이름"으로 매핑한다 (학습 순서가 바뀌어도 안전하게).

uint8 PEDESTRIAN_SIGN = 0   # 보행자 표지판 (삼각형)              -> 미션 3
uint8 SIGN_A          = 1   # 갈림길 A (좌) 표지판                -> 미션 2
uint8 SIGN_B          = 2   # 갈림길 B (우) 표지판                -> 미션 2
uint8 TRAFFIC_LIGHT   = 3   # 신호등 (색은 light_color 에)        -> 미션 1
uint8 PARKING_SIGN    = 4   # 주차 표지판 (P)                     -> 미션 4
uint8 LIMO            = 5   # 정차 LIMO (장애물 차량)             -> 미션 7, 주차
uint8 BUMP_SIGN       = 6   # 방지턱 표지판 (삼각형)              -> 미션 5
uint8 SLOW_SIGN       = 7   # 서행 표지판 (원형, 세워진 표지판만. 노면 "서행" 글씨는 제외)
uint8 SLOW_GUIDE_LINE = 8   # 노면 지그재그 서행 유도선           -> 미션 6
uint8 CROSSWALK       = 9   # 횡단보도 (줄무늬 전체를 박스 1개로)  -> 미션 1·3 정지 위치 (11/02 추가, 정지선 대신)

uint8   class_id
float32 confidence          # 0.0~1.0 (클래스별 문턱은 인지 노드에서 이미 적용됨)

# 신호등 전용. class_id == TRAFFIC_LIGHT 일 때만 의미 있음.
uint8 LIGHT_UNKNOWN = 0
uint8 LIGHT_RED     = 1
uint8 LIGHT_YELLOW  = 2
uint8 LIGHT_GREEN   = 3
uint8   light_color

# 위치 추정 성공 여부. false 면 x_m, y_m 은 무의미.
bool    position_valid
float32 x_m                 # 전방 거리 [m] (base_link 기준). SLOW_GUIDE_LINE 은 유도선의 가까운 끝(시작점)까지,
                            #   CROSSWALK 는 횡단보도의 차에 가까운 쪽 끝(앞쪽 가장자리)까지 = 판단이 앞바퀴를 세우는 기준
float32 y_m                 # 좌측 양수 [m]

# 위치를 어떻게 구했는지 (디버깅·신뢰도 판단용)
uint8 POS_NONE   = 0
uint8 POS_GROUND = 1        # 바닥평면 교점
uint8 POS_DEPTH  = 2        # RGB-D 깊이
uint8 POS_SIZE   = 3        # 알려진 실제 크기로 역산 (오차 큼)
uint8 position_source
