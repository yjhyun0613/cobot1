import sys
import time
import importlib.util

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

        self.progress_status_sub = self.create_subscription(String, PROGRESS_TOPIC, self.progress_status_callback, 10)
        self.progress_status_pub = self.create_publisher(String, PROGRESS_TOPIC, 10)
        
        self.move_joint_client = self.create_client(MoveJoint, f"/{ROBOT_ID}/motion/move_joint")
        self.move_line_client = self.create_client(MoveLine, f"/{ROBOT_ID}/motion/move_line")

        self.trigger_start = False
        self.is_running = False 

        # 좌표 설정
        self.home_joint = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]

        # ============================================================

        self.conveyor_above_pick_joint = [-31.66, 43.77, 37.3, 0, 99.24, -31.41]
        self.inspection_above_joint = [16.43, 14.66, 93.17, 0.21, 72.25, 24.45]
        self.tool_safe_joint = [7.2, 28.08, 75.94, -6.06, 48.37, 9.12] 

        self.conveyor_above_pick_posx = [552.47, -332.12, 136.34, 104.72, -177.01, 105.27]
        self.conveyor_pick_posx = [552.47, -332.12, 86.34, 104.72, -177.01, 105.27]
        self.inspection_above_posx = [431.75, 138.3, 66.5, 121.82, 179.67, 129.54]
        self.inspection_place_posx = [431.75, 138.3, 6.5, 121.82, 179.67, 129.54]
        self.tool_safe_posx = [708.82, 72.51, 99.76, 178.28, -151.92, 177.01]
        self.tool_place_posx = [744.15, 72.82, 33.01, 178.69, -152.04, 177.4]       
        self.pressing_above_posx = [425.35, 74.68, 101.92, 50.91, -179.12, 49.78]
        self.pressing_posx = [425.35, 74.68, 51.92, 50.91, -179.12, 49.78]

        # 속도 및 가속도 (wait_time은 삭제됨)
        self.joint_vel, self.joint_acc = 30.0, 30.0
        self.line_vel, self.line_acc = [50.0, 30.0], [50.0, 30.0]
        self.move_time = 2.5 

        self.get_logger().info("✅ 진정한 동기식(Synchronous) 노드 준비 완료")

    def initialize_robot(self):
        try:
            self.set_robot_mode(self.ROBOT_MODE_MANUAL)
            time.sleep(0.5)
            self.set_tool(ROBOT_TOOL)
            self.set_tcp(ROBOT_TCP)
            self.set_robot_mode(self.ROBOT_MODE_AUTONOMOUS)
            time.sleep(1.0)
        except Exception as e:
            self.get_logger().error(f"초기화 실패: {e}")

    def progress_status_callback(self, msg):
        data = msg.data.replace("'", "").replace('"', "").strip()
        if data == START_SIGNAL and not self.is_running:
            self.trigger_start = True

    def run_sequence(self):
        tasks = [
            ("movej", "컨베이어 위 좌표 이동", self.conveyor_above_pick_joint),
            ("gripper", "그리퍼 열기", "open"),
            ("movel", "컨베이어 좌표 이동", self.conveyor_pick_posx),
            ("gripper", "그리퍼 닫기", "close"),
            ("movel", "컨베이어 위 좌표 복귀", self.conveyor_above_pick_posx),
            ("movej", "검사 장소 위 이동", self.inspection_above_joint),
            ("movel", "검사 장소 하강", self.inspection_place_posx),
            ("gripper", "그리퍼 열기", "open"),
            ("movel", "검사 장소 위 이동", self.inspection_above_posx),
            ("movel", "압입 지점 50mm 상부 이동", self.pressing_above_posx),
            ("movel", "압입 진행", self.pressing_posx),
            ("movel", "압입 지점 50mm 상부 이동", self.pressing_above_posx),
            ("movej", "그리퍼 위 장소 이동", self.tool_safe_joint),
            ("movel", "그리퍼 좌표 이동", self.tool_place_posx),
            ("gripper", "그리퍼 닫기", "close"),
            ("movel", "그리퍼 위 장소 복귀", self.tool_safe_posx),
            ("movej", "초기 위치 이동", self.home_joint)
        ]

        for action_type, desc, target in tasks:
            if action_type == "movej":
                success = self.move_joint(desc, target)
            elif action_type == "movel":
                success = self.move_line(desc, target)
            elif action_type == "gripper":
                success = self.control_gripper(target, desc)

            if not success:
                self.get_logger().error(f"🚨 시퀀스 실패: {desc}")
                return False
        return True

    def control_gripper(self, action, desc):
        on_pin, off_pin = (2, 1) if action == "open" else (1, 2)
        self.get_logger().info(f"🦾 {desc}")
        try:
            self.set_digital_output(on_pin, ON)
            self.set_digital_output(off_pin, OFF)
            self.dsr_wait(1.5) # 물리적 그리핑 시간 보장
            return True
        except: return False

    def move_joint(self, desc, joint):
        self.get_logger().info(f"🤖 이동: {desc}")
        req = MoveJoint.Request()
        req.pos = [float(x) for x in joint]
        req.vel, req.acc, req.time, req.sync_type = self.joint_vel, self.joint_acc, self.move_time, 0
        future = self.move_joint_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    def move_line(self, desc, posx):
        self.get_logger().info(f"🤖 이동: {desc}")
        req = MoveLine.Request()
        req.pos = [float(x) for x in posx]
        req.vel, req.acc, req.time, req.sync_type = self.line_vel, self.line_acc, self.move_time, 0
        future = self.move_line_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

def main(args=None):
    rclpy.init(args=args)
    node = M0609ConveyorPickPlaceToolNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.trigger_start:
                node.is_running = True
                node.trigger_start = False 
                if node.run_sequence():
                    msg = String(); msg.data = DONE_SIGNAL
                    node.progress_status_pub.publish(msg)
                node.is_running = False 
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node(); rclpy.shutdown()

if __name__ == "__main__":
    main()