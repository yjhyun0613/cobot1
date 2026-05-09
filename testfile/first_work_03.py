import sys
import time
import importlib.util

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

# 로봇 기본 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

ON, OFF = 1, 0
PROGRESS_TOPIC = f"/{ROBOT_ID}/progress_status"
START_SIGNAL = "01"
DONE_SIGNAL = "02"

class M0609ConveyorPickPlaceToolNode(Node):
    def __init__(self):
        super().__init__("m0609_conveyor_pick_place_tool_node", namespace=ROBOT_ID)

        # DSR_ROBOT2 import 전 DR_init 설정
        setattr(DR_init, "__dsr__id", ROBOT_ID)
        setattr(DR_init, "__dsr__model", ROBOT_MODEL)
        setattr(DR_init, "__dsr__node", self)

        from DSR_ROBOT2 import (
            set_digital_output, get_digital_input, wait,
            set_tool, set_tcp, get_robot_mode, set_robot_mode,
            ROBOT_MODE_MANUAL, ROBOT_MODE_AUTONOMOUS,
        )
        self.set_digital_output = set_digital_output
        self.get_digital_input = get_digital_input
        self.dsr_wait = wait
        self.set_tool = set_tool
        self.set_tcp = set_tcp
        self.set_robot_mode = set_robot_mode
        self.ROBOT_MODE_MANUAL = ROBOT_MODE_MANUAL
        self.ROBOT_MODE_AUTONOMOUS = ROBOT_MODE_AUTONOMOUS

        self.initialize_robot()

        # ROS2 통신 설정
        self.progress_status_sub = self.create_subscription(
            String, PROGRESS_TOPIC, self.progress_status_callback, 10
        )
        self.progress_status_pub = self.create_publisher(String, PROGRESS_TOPIC, 10)
        
        self.move_joint_client = self.create_client(MoveJoint, f"/{ROBOT_ID}/motion/move_joint")
        self.move_line_client = self.create_client(MoveLine, f"/{ROBOT_ID}/motion/move_line")

        # 💡 [핵심] spin 코드처럼 상태 깃발(Flag) 도입
        self.trigger_start = False
        self.is_running = False 

        # ====================================================
        # 좌표 및 속도 설정
        # ====================================================
        self.home_joint = [0, 0, 90, 0, 90, 0]
        self.conveyor_above_pick_posx = [547.76, -317.89, 149.03, 173.5, -179.41, 173.2]
        self.conveyor_pick_posx = [547.76, -317.89, 89.03, 173.5, -179.41, 173.2]
        self.inspection_above_posx = [424.48, 139.36, 59.95, 18.45, -179.09, 17.94]
        self.inspection_place_posx = [424.48, 139.36, 9.95, 18.45, -179.09, 17.94]
        self.tool_safe_posx = [708.82, 72.51, 99.76, 178.28, -151.92, 177.01]
        self.tool_place_posx = [744.15, 72.82, 33.01, 178.69, -152.04, 177.4]

        self.joint_vel, self.joint_acc = 30.0, 30.0
        self.line_vel, self.line_acc = [50.0, 30.0], [50.0, 30.0]
        self.move_time, self.wait_time = 2.0, 3.0

        self.get_logger().info("✅ 토픽 퍼블리셔/서브스크라이버 준비 완료")

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

    # ========================================================
    # 💡 [핵심] spin 코드 방식: 깃발만 꽂고 바로 퇴근하는 콜백
    # ========================================================
    def progress_status_callback(self, msg):
        raw_signal = msg.data
        signal = raw_signal.replace("'", "").replace('"', "").strip()

        if signal == START_SIGNAL:
            if not self.is_running and not self.trigger_start:
                self.get_logger().info(f"📥 수신 확인: '{START_SIGNAL}' -> 작업 시퀀스를 예약합니다.")
                self.trigger_start = True # 깃발 꽂기
            else:
                self.get_logger().warn("⏳ 이미 작업이 진행 중이거나 예약되어 무시됩니다.")

    def run_sequence(self):
        tasks = [
            ("movej",   "초기 좌표 복귀", self.home_joint),
            ("gripper", "초기 그리퍼 열기", "open"),
            ("movel",   "컨베이어 안전 접근", self.conveyor_above_pick_posx),
            ("movel",   "컨베이어 픽업 하강", self.conveyor_pick_posx),
            ("gripper", "컨베이어 물체 잡기", "close"),
            ("movel",   "컨베이어 픽업 후 상승", self.conveyor_above_pick_posx),
            ("movel",   "검사 장소 위 이동", self.inspection_above_posx),
            ("movel",   "검사 장소로 하강", self.inspection_place_posx),
            ("gripper", "물체 놓기", "open"),
            ("movel",   "검사 장소 위 상승", self.inspection_above_posx),
            ("movel",   "도구 안전 경유 위치", self.tool_safe_posx),
            ("movel",   "도구 최종 위치 하강", self.tool_place_posx),
            ("gripper", "도구 잡기", "close"),
            ("movel",   "도구 픽업 후 경유 위치", self.tool_safe_posx),
            ("movej",   "작업 완료 후 홈 복귀", self.home_joint)
        ]

        for action_type, desc, target in tasks:
            success = False
            if action_type == "movej":
                success = self.move_joint(desc, target)
            elif action_type == "movel":
                success = self.move_line(desc, target)
            elif action_type == "gripper":
                success = self.control_gripper(target, desc)

            if not success:
                self.get_logger().error(f"🚨 시퀀스 중단: {desc} 실패")
                self.control_gripper("open", "안전 정지 그리퍼 열기")
                self.move_joint("안전 정지 홈 복귀", self.home_joint)
                return False

        return True

    def control_gripper(self, action, desc):
        on_pin, off_pin = (2, 1) if action == "open" else (1, 2)
        
        self.get_logger().info(f"🦾 {desc}")
        try:
            self.set_digital_output(on_pin, ON)
            self.set_digital_output(off_pin, OFF)
            
            start_time = time.time()
            # 💡 여기서는 로봇 내부 제어기 상태를 대기하는 것이므로 통신을 뻗게 하지 않음
            while rclpy.ok():
                if self.get_digital_input(on_pin): return True
                if time.time() - start_time > 5.0:
                    self.get_logger().warn(f"{desc} 시간 초과 (센서 미감지)")
                    return False
                self.dsr_wait(0.1)
        except Exception as e:
            self.get_logger().error(f"{desc} 제어 오류: {e}")
            return False

    def move_joint(self, desc, joint):
        self.get_logger().info(f"🤖 이동: {desc}")
        req = MoveJoint.Request()
        req.pos = [float(x) for x in joint]
        req.vel, req.acc, req.time = self.joint_vel, self.joint_acc, self.move_time
        req.sync_type = 1
        self.move_joint_client.call_async(req)
        time.sleep(self.wait_time)
        return True

    def move_line(self, desc, posx):
        self.get_logger().info(f"🤖 이동: {desc}")
        req = MoveLine.Request()
        req.pos = [float(x) for x in posx]
        req.vel, req.acc, req.time = self.line_vel, self.line_acc, self.move_time
        req.sync_type = 1
        self.move_line_client.call_async(req)
        time.sleep(self.wait_time)
        return True

# ========================================================
# 💡 [핵심] spin 코드 방식: 메인 무한 루프에서 로봇 구동
# ========================================================
def main(args=None):
    rclpy.init(args=args)
    node = M0609ConveyorPickPlaceToolNode()

    try:
        print(f"\n🔄 대기 모드: 터미널에서 '{START_SIGNAL}' 신호를 기다립니다...")

        # MultiThreadedExecutor를 지우고 단일 스레드 무한 루프 적용
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            
            # 콜백 함수가 꽂아둔 깃발을 메인 루프가 발견하면 실행
            if node.trigger_start:
                node.is_running = True
                node.trigger_start = False # 깃발 내리기
                
                print(f"\n▶️ [작업 시작] '{START_SIGNAL}' 시퀀스 구동 중...")
                try:
                    if node.run_sequence():
                        print(f"\n🏁 [작업 완료] '{DONE_SIGNAL}' 송신")
                        msg_out = String()
                        msg_out.data = DONE_SIGNAL
                        node.progress_status_pub.publish(msg_out)
                except Exception as e:
                    print(f"\n🚨 시퀀스 실행 중 에러 발생: {e}")
                finally:
                    node.is_running = False 
                    print(f"\n🔄 대기 모드: 다음 '{START_SIGNAL}' 신호를 기다립니다...")

    except KeyboardInterrupt:
        print("\n✅ 노드를 안전하게 종료합니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()