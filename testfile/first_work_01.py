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
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
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
        self.callback_group = ReentrantCallbackGroup()

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
            String, PROGRESS_TOPIC, self.progress_status_callback, 10, callback_group=self.callback_group
        )
        self.progress_status_pub = self.create_publisher(String, PROGRESS_TOPIC, 10)
        
        self.move_joint_client = self.create_client(MoveJoint, f"/{ROBOT_ID}/motion/move_joint", callback_group=self.callback_group)
        self.move_line_client = self.create_client(MoveLine, f"/{ROBOT_ID}/motion/move_line", callback_group=self.callback_group)

        # 💡 [핵심] 01 반복 실행을 위한 상태 변수
        self.is_running = False 

        # ====================================================
        # 좌표 및 속도 설정
        # ====================================================
        self.home_joint = [0, 0, 90, 0, 90, 0]
        self.conveyor_above_pick_posx = [370.26, -301.44, 211.41, 28.05, -179.6, 27.48]
        self.conveyor_pick_posx = [370.26, -301.44, 170.33, 28.05, -179.6, 27.48]
        self.inspection_above_posx = [433.68, 73.82, 184.71, 3, -179.12, -6.38]
        self.inspection_place_posx = [433.68, 73.82, 84.71, 3, -179.12, -6.38]
        self.tool_safe_posx = [719.11, 68.34, 73.95, 2.44, 147.16, 0.18]
        self.tool_place_posx = [744.85, 70.11, 32.38, 2.66, 147.16, 0.39]

        self.joint_vel, self.joint_acc = 30.0, 30.0
        self.line_vel, self.line_acc = [50.0, 30.0], [50.0, 30.0]
        self.move_time, self.wait_time = 2.0, 3.0

        self.get_logger().info(f"✅ 노드 실행됨: '{START_SIGNAL}' 수신을 기다립니다...")

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
    # 💡 [핵심] 진행 상태 콜백 (반복 실행 대응 로직)
    # ========================================================
    def progress_status_callback(self, msg):
        signal = msg.data.strip()

        # 이미 작업 중이면 중복 실행 방지
        if self.is_running:
            return

        # 01 신호가 들어왔을 때만 동작 시작
        if signal == START_SIGNAL:
            self.get_logger().info(f"▶️ [작업 시작] '{START_SIGNAL}' 수신")
            self.is_running = True # 락(Lock) 걸기

            try:
                if self.run_sequence():
                    self.get_logger().info(f"🏁 [작업 완료] '{DONE_SIGNAL}' 송신")
                    msg = String(); msg.data = DONE_SIGNAL
                    self.progress_status_pub.publish(msg)
            except Exception as e:
                self.get_logger().error(f"시퀀스 오류: {e}")
            finally:
                # 💡 모든 작업이 끝나면 락을 풀어서 다음 '01' 신호를 받을 수 있게 함
                self.is_running = False 
                self.get_logger().info(f"🔄 대기 모드: 다음 '{START_SIGNAL}' 신호를 기다립니다...")

    # ========================================================
    # 💡 [핵심] 리스트 기반 15단계 시퀀스 자동화 (코드 70% 압축)
    # ========================================================
    def run_sequence(self):
        # (동작유형, 설명, 목표좌표/액션)
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

    # ========================================================
    # 💡 [핵심] 그리퍼 제어 함수 하나로 통합 (TMI 로그 제거)
    # ========================================================
    def control_gripper(self, action, desc):
        on_pin, off_pin = (2, 1) if action == "open" else (1, 2)
        
        self.get_logger().info(f"🦾 {desc}")
        try:
            self.set_digital_output(on_pin, ON)
            self.set_digital_output(off_pin, OFF)
            
            start_time = time.time()
            while rclpy.ok():
                if self.get_digital_input(on_pin): return True
                if time.time() - start_time > 5.0:
                    self.get_logger().warn(f"{desc} 시간 초과 (센서 미감지)")
                    return False
                self.dsr_wait(0.1)
        except Exception as e:
            self.get_logger().error(f"{desc} 제어 오류: {e}")
            return False

    # ========================================================
    # 이동 제어 (코드 간소화)
    # ========================================================
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


def main(args=None):
    rclpy.init(args=args)
    node = M0609ConveyorPickPlaceToolNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("사용자 종료")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()