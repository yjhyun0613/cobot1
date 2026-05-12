# last_work_final.py
# PASS 좌표 수정
import sys
import time

# ============================================================
# DSR_ROBOT2 / DR_init 경로 설정
# ============================================================
DSR_SITE_PACKAGES_PATH = "/home/rokey/cobot_ws/install/dsr_common2/lib/python3.10/site-packages"

if DSR_SITE_PACKAGES_PATH in sys.path:
    sys.path.remove(DSR_SITE_PACKAGES_PATH)
sys.path.insert(0, DSR_SITE_PACKAGES_PATH)

import DR_init
sys.modules["DR_init"] = DR_init

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from dsr_msgs2.srv import MoveJoint, MoveLine

# ============================================================
# 로봇 / 토픽 기본 설정
# ============================================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

ON, OFF = 1, 0

PROGRESS_TOPIC = f"/{ROBOT_ID}/progress_status"
INSPECTION_RESULT_TOPIC = f"/{ROBOT_ID}/inspection_result"

START_SIGNAL = "03"
PASS_SIGNAL = "PASS"
FAIL_SIGNAL = "FAIL"
DONE_SIGNAL = "04"

class M0609SortingTestNode(Node):
    def __init__(self):
        super().__init__("m0609_sorting_test_node", namespace=ROBOT_ID)

        # DSR_ROBOT2 import 전 DR_init 설정
        setattr(DR_init, "__dsr__id", ROBOT_ID)
        setattr(DR_init, "__dsr__model", ROBOT_MODEL)
        setattr(DR_init, "__dsr__node", self)

        # 💡 [핵심] get_robot_state 등 누락되었던 필수 모듈 추가
        from DSR_ROBOT2 import (
            set_digital_output, get_digital_input, wait,
            set_tool, set_tcp, set_robot_mode, get_robot_state,
            ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS,
            movej, movel
        )

        self.set_digital_output = set_digital_output
        self.get_digital_input = get_digital_input
        self.dsr_wait = wait
        self.set_tool = set_tool
        self.set_tcp = set_tcp
        self.set_robot_mode = set_robot_mode
        self.get_robot_state = get_robot_state 
        self.dsr_movej = movej
        self.dsr_movel = movel
        self.ROBOT_MODE_MANUAL = ROBOT_MODE_MANUAL
        self.ROBOT_MODE_AUTONOMOUS = ROBOT_MODE_AUTONOMOUS

        self.initialize_robot()

        # ROS2 통신 설정
        self.progress_status_sub = self.create_subscription(
            String, PROGRESS_TOPIC, self.progress_status_callback, 10
        )
        self.inspection_result_sub = self.create_subscription(
            String, INSPECTION_RESULT_TOPIC, self.inspection_result_callback, 10
        )
        self.progress_status_pub = self.create_publisher(String, PROGRESS_TOPIC, 10)

        # 깃발(Flag) 변수들
        self.is_running = False
        self.trigger_sorting = False
        self.received_start_signal = False
        self.pending_result = None
        self.latest_result = None

        # ====================================================
        # 좌표 / 속도 설정
        # ====================================================
        self.home_joint = [0, 0, 90, 0, 90, 0] # 수정 완료
        self.tool_pick_joint = [7.2, 28.08, 75.94, -6.06, 48.37, 9.12] 
        self.pass_pos_joint = [35.75, 60.97, 15.71, 8.41, 66.92, 66.99] # 수정 완료
        self.fail_pos_joint = [61.53, 31.85, 69.11, 1.16, 78.58, -14.88] # 수정 완료
        self.inspection_above_posx = [426.5, 135.72, 57.88, 104.72, -177.01, 105.27] # 수정 완료
        self.inspection_place_posx = [426.5, 135.72, 7.88, 104.72, -177.01, 105.27] # 수정 완료      
        self.tool_safe_posx = [708.82, 72.51, 99.76, 178.28, -151.92, 177.01]
        self.tool_place_posx = [744.15, 72.82, 33.01, 178.69, -152.04, 177.4]
        self.get_logger().info("✅ M0609 PASS/FAIL 분배 노드 준비 완료")

    def initialize_robot(self):
        try:
            self.set_robot_mode(self.ROBOT_MODE_MANUAL)
            time.sleep(0.5)
            self.set_tool(ROBOT_TOOL)
            self.set_tcp(ROBOT_TCP)
            self.set_robot_mode(self.ROBOT_MODE_AUTONOMOUS)
            time.sleep(2.0)
            self.get_logger().info("✅ 로봇 초기화 완료")
        except Exception as e:
            self.get_logger().error(f"로봇 초기화 실패: {e}")

    def clean_signal(self, data):
        return data.replace("'", "").replace('"', "").strip().upper()

    # ========================================================
    # 콜백 함수: 신호를 받으면 깃발만 꽂음
    # ========================================================
    def progress_status_callback(self, msg):
        signal = self.clean_signal(msg.data)
        if signal == START_SIGNAL and not self.is_running:
            self.received_start_signal = True
            self.get_logger().info(f"📥 '{START_SIGNAL}' 수신 완료. 결과를 기다립니다.")
            self.try_trigger()

    def inspection_result_callback(self, msg):
        result = self.clean_signal(msg.data)
        if result in [PASS_SIGNAL, FAIL_SIGNAL] and not self.is_running:
            self.latest_result = result
            self.get_logger().info(f"📥 검사 결과 수신: {result}")
            self.try_trigger()

    def try_trigger(self):
        if self.received_start_signal and self.latest_result:
            self.pending_result = self.latest_result
            self.trigger_sorting = True
            self.received_start_signal = False
            self.latest_result = None

    # ========================================================
    # 💡 [핵심] 리스트 기반 분배 시퀀스
    # ========================================================
    def run_sorting_sequence(self, result):
        target_pos = self.pass_pos_joint if result == PASS_SIGNAL else self.fail_pos_joint
        target_name = f"분배 이동 ({result})"

        tasks = [
            ("movej",   "초기 좌표 복귀", self.home_joint),
            ("movej",   "도구 반납 접근", self.tool_pick_joint),
            ("movel",   "도구 최종 좌표 이동", self.tool_place_posx),
            ("gripper", "도구 내려놓기", "open"),
            ("movel",   "도구 안전 탈출", self.tool_safe_posx),
            ("movej",   "초기 위치 복귀", self.home_joint),
            ("movel",   "검사 장소 위 이동", self.inspection_above_posx),
            ("gripper", "그리퍼 열기", "open"),
            ("movel",   "물체 위치 하강", self.inspection_place_posx),
            ("gripper", "물체 잡기", "close"),
            ("movel",   "검사 장소 위 상승", self.inspection_above_posx),
            ("movej",   target_name, target_pos),
            ("gripper", "물체 놓기", "open"),
            ("movej",   "최종 홈 복귀", self.home_joint)
        ]

        for action_type, desc, target in tasks:
            success = False
            if action_type == "movej": success = self.move_joint_smart(desc, target)
            elif action_type == "movel": success = self.move_line_smart(desc, target)
            elif action_type == "gripper": success = self.control_gripper(target, desc)

            if not success:
                self.get_logger().error(f"🚨 시퀀스 실패: {desc}")
                return False
        return True

    # ========================================================
    # 💡 [핵심] spin 코드 방식의 실시간 상태 감지 이동 함수
    # ========================================================
    def move_joint_smart(self, desc, joint):
        self.get_logger().info(f"🤖 이동: {desc}")
        self.dsr_movej(joint, vel=30, acc=30)
        return self.wait_until_stopped()

    def move_line_smart(self, desc, posx):
        self.get_logger().info(f"🤖 이동: {desc}")
        self.dsr_movel(posx, vel=50, acc=50)
        return self.wait_until_stopped()

    def wait_until_stopped(self):
        time.sleep(0.5) # 출발 대기
        stop_count = 0
        while rclpy.ok():
            if self.get_robot_state() == 1: # 1: Idle(정지)
                stop_count += 0.1
                if stop_count >= 0.5: break # 0.5초간 유지 시 도착 판정
            else:
                stop_count = 0
            time.sleep(0.1)
        return True

    def control_gripper(self, action, desc):
        self.get_logger().info(f"🦾 {desc}")
        on_pin, off_pin, di_pin = (1, 2, 1) if action == "close" else (2, 1, 2)
        self.set_digital_output(on_pin, ON)
        self.set_digital_output(off_pin, OFF)
        
        start_time = time.time()
        while time.time() - start_time < 5.0:
            if self.get_digital_input(di_pin):
                time.sleep(0.5) # 물리적 동작 시간 보정
                return True
            time.sleep(0.1)
        return False

def main(args=None):
    rclpy.init(args=args)
    node = M0609SortingTestNode()

    try:
        print(f"\n🔄 대기 모드: '{START_SIGNAL}' + 결과 신호를 기다립니다...")
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

            if node.trigger_sorting:
                node.is_running = True
                node.trigger_sorting = False
                res = node.pending_result

                print(f"\n▶️ [분배 시작] 결과: {res}")
                if node.run_sorting_sequence(res):
                    print(f"🏁 [분배 완료] '{DONE_SIGNAL}' 송신")
                    msg = String(); msg.data = DONE_SIGNAL
                    node.progress_status_pub.publish(msg)
                
                node.is_running = False
                print(f"\n🔄 대기 모드 진입")

    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()