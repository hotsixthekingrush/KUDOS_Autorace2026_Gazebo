"""미션 7 - 정적 장애물 회피 (3차선, 정차 LIMO 7대).

진입: YOLO 가 진행 경로 앞의 LIMO 를 보거나, 라이다 장애물이 진행 경로 앞에 있으면.
      (주차 구역 LIMO 처럼 옆에 있는 것은 무시 - supervisor 의 trigger 조건)
동작 (S1 - 차선 단위로 고르기, 실제 계산은 three_lane.ThreeLanePlanner):
  - 진입 순간 차선 방향을 기준 좌표로 잡고(odom), 시작 차선은 가운데(1번)로 둔다.
  - 라이다 장애물을 기준 좌표에 쌓아 두고 차선 3개에 나눈다.
  - 후보 차선 3곳(차선 가운데)마다 앞 plan_horizon 을 실제 조향 한계로 미리 굴려 보고(rollout)
    안 닿는 차선으로 간다. 옮기는 것은 odom 기준 Pure Pursuit. 차선 사이(선 위)로는 안 간다.
  - 차선 번호는 변경을 '끝냈을 때만' 바꾼다. odom 위치와 계속 어긋나면 odom 을 따른다.
  - 안 닿는 차선이 없으면 감속하고, 가까우면 서서 기다린다 (지도가 바뀌면 다시 계산).
  - odom 이 없을 때만 비상용으로 빈 공간 따라가기를 쓴다.
종료:
  - A/B 표지판을 보면 LEFT_A_RIGHT_B 로 바로 넘어간다. LEFT_A_RIGHT_B 가 같은 좌표·차선 번호로
    장애물 피하기를 이어 하면서 A/B 차선으로 옮긴다.
  - 장애물을 다 지나 exit_clear_distance 만큼 더 가면 LANE_FOLLOWING 으로.
  - max_distance / max_duration 초과 시 LANE_FOLLOWING 으로.
규정: 접촉 시 1점 감점, 15초 이내 미진입 시 실패, 체크포인트 거리 비례 부분점수.
"""

from ar_mission_supervisor.base_state import SafeState


class AvoidStatic(SafeState):

    OUTCOMES = ['done', 'to_left_a_right_b']
    HANDOFF = ('to_left_a_right_b',)

    def __init__(self, node):
        SafeState.__init__(self, node, 'AVOID_STATIC', self.OUTCOMES)
        self.plan = node.three_lane
        self.entered_at = None
        self.gap_mode = False

    def on_enter(self):
        node = self.node
        self.entered_at = node.clock_sec()
        ok = self.plan.begin(node.avoid_static['start_lane'])
        self.gap_mode = not ok
        if ok:
            node.get_logger().info('[AVOID_STATIC] 진입 - %s 차선에서 시작 (odom 기준)'
                                   % self.plan.name(self.plan.lane))
        else:
            node.get_logger().warn('[AVOID_STATIC] odom 없음 - 빈 공간 따라가기로만 주행')

    def run(self):
        node = self.node
        cfg = node.avoid_static
        plan = self.plan

        while node.ok():
            if self.reset_requested():
                plan.finish()
                return 'aborted'

            now = node.clock_sec()
            if now - self.entered_at > cfg['max_duration']:
                node.get_logger().warn('[AVOID_STATIC] 제한시간 초과 - 복귀')
                plan.release(now)
                return 'done'

            handoff = node.detect_mission(allowed=self.HANDOFF)
            if handoff is not None:
                node.get_logger().info('[AVOID_STATIC] A/B 표지판 인식 - LEFT_A_RIGHT_B 로 바로 전환 '
                                       '(지금 %s 차선)' % plan.name(plan.lane))
                plan.release(now)
                return handoff

            if self.gap_mode:
                plan.gap_step()
                self.sleep()
                continue

            res = plan.step(now, goal=None, who='AVOID_STATIC')
            if res is None:
                node.get_logger().warn('[AVOID_STATIC] odom 끊김 - 빈 공간 따라가기로 전환')
                self.gap_mode = True
                continue
            if res.blocked:
                self.sleep()                    # 길이 없음 - 정지한 채로 다시 계산
                continue

            if res.s > cfg['max_distance']:
                node.get_logger().warn('[AVOID_STATIC] 최대 주행거리 %.1fm 초과 - 복귀' % cfg['max_distance'])
                plan.release(now)
                return 'done'

            if (not plan.changing and plan.max_obstacle_s is not None
                    and not plan.obstacles_ahead(res.s)
                    and res.s - plan.max_obstacle_s >= cfg['exit_clear_distance']):
                node.get_logger().info('[AVOID_STATIC] 장애물 구간 통과 - 복귀 (%s 차선)'
                                       % plan.name(plan.lane))
                plan.release(now)
                return 'done'

            self.sleep()

        plan.finish()
        return 'aborted'
