# 차선 색 프리셋 (config/colors)

노란 차선 색 임계값 묶음. `camera_perception_node` 파라미터만 덮어쓴다 (코드 변경 없음).

| 파일 | 언제 / 어디서 뽑은 값 |
|---|---|
| `1.yaml` | 11/11 실내 트랙 (HLS, LAB 모두 실내 값) |
| `2.yaml` | 11/17 대회장 화면 캡처 (HLS, LAB 모두 캡처 값) |
| `3.yaml` | 11/18 대회장 bag comp_lap2 (HLS + 공통 LAB, max_seg_w_m 0.08, side_guard_m 0.25) |
| `4.yaml` | 11/18 3번에서 L/S 범위를 넓힌 값 (HLS + 공통 LAB, max_seg_w_m 0.08, side_guard_m 0.25) |
| `5.yaml` | 11/19 대회용: 4번에서 lane_l_h 240 (흰 페인트 빼기), LAB b 하한 140, max_seg_w_m 0.08, side_guard_m 0.25 |

**대회는 5번.**

공통 LAB (3, 4번): lab_l 110~255, lab_a 105~135, lab_b 148~255.

## LAB 버전
각 파일에 HLS 와 LAB 값이 같이 들어 있다. LAB 로 쓰려면 파일의 `color_space: hls` 를 `color_space: lab` 으로만 바꾸면 된다
(실행 중이면 `ros2 param set /camera_perception_node color_space lab`).

## 실차 사용법
```bash
mkdir -p ~/colors && cp <ws>/src/ar_lane_perception/config/colors/*.yaml ~/colors/
ros2 launch ar_lane_perception camera_perception.launch.py \
  lane_extra:=<ws>/src/ar_lane_perception/config/safe3_hls.yaml,$HOME/colors/3.yaml
```
`lane_extra` 는 앞에서부터 차례로 덮어쓰므로 프리셋을 safe3_hls.yaml 뒤에 둔다.
