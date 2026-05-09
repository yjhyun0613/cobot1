# first_work_02.py

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

# 혹시 다른 DR_init가 잡히는 것을 방지
sys.modules["DR_init"] = DR_init

print("DEBUG DR_init file:", DR_init.__file__)

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from std_msgs.msg import String
from dsr_msgs2.srv import MoveJoint, MoveLine


# ============================================================
# 로봇 기본 설정
# ============================================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
ROBOT_TOOL = "Tool Weight"
ROBOT_TCP = "GripperDA_v1"

ON = 1
OFF = 0

# ============================================================
# 진행 상태 통신 설정
# 이 노드는 01을 받으면 작업 시작, 작업 완료 후 02를 송신
# ============================================================
PROGRESS_TOPIC = f"/{ROBOT_ID}/progress_status"

START_SIGNAL = "01"
DONE_SIGNAL = "02"


class M0609ConveyorPickPlaceToolNode(Node):
    def __init__(self):
        super().__init__("m0609_conveyor_pick_place_tool_node", namespace=ROBOT_ID)

        # ====================================================
        # Callback Group 설정
        # ====================================================
        self.callback_group = ReentrantCallbackGroup()

        # ====================================================
        # DSR_ROBOT2 import 전 DR_init 설정
        # ====================================================
        setattr(DR_init, "__dsr__id", ROBOT_ID)
        setattr(DR_init, "__dsr__model", ROBOT_MODEL)
        setattr(DR_init, "__dsr__node", self)

        print("DEBUG ROBOT_ID:", getattr(DR_init, "__dsr__id"))
        print("DEBUG ROBOT_MODEL:", getattr(DR_init, "__dsr__model"))
        print("DEBUG NODE:", getattr(DR_init, "__dsr__node"))

        spec = importlib.util.find_spec("DSR_ROBOT2")
        if spec is not None:
            print("DEBUG DSR_ROBOT2 spec:", spec.origin)
        else:
            print("DEBUG DSR_ROBOT2 spec: None")

        # ====================================================
        # 반드시 DR_init 설정 후 DSR_ROBOT2 import
        # ====================================================
        from DSR_ROBOT2 import (
            set_digital_output,
            get_digital_input,
            wait,
            set_tool,
            set_tcp,
            get_tool,
            get_tcp,
            get_robot_mode,
            set_robot_mode,
            ROBOT_MODE_MANUAL,
            ROBOT_MODE_AUTONOMOUS,
        )

        self.set_digital_output = set_digital_output
        self.get_digital_input = get_digital_input
        self.dsr_wait = wait

        self.set_tool = set_tool
        self.set_tcp = set_tcp
        self.get_tool = get_tool
        self.get_tcp = get_tcp
        self.get_robot_mode = get_robot_mode
        self.set_robot_mode = set_robot_mode
        self.ROBOT_MODE_MANUAL = ROBOT_MODE_MANUAL
        self.ROBOT_MODE_AUTONOMOUS = ROBOT_MODE_AUTONOMOUS

        # ====================================================
        # 로봇 Tool / TCP / Mode 초기화
        # ====================================================
        self.initialize_robot()

        # ====================================================
        # ROS2 subscriber / publisher / service client
        # ====================================================
        self.progress_status_sub = self.create_subscription(
            String,
            PROGRESS_TOPIC,
            self.progress_status_callback,
            10,
            callback_group=self.callback_group,
        )

        self.progress_status_pub = self.create_publisher(
            String,
            PROGRESS_TOPIC,
            10,
        )

        self.move_joint_client = self.create_client(
            MoveJoint,
            f"/{ROBOT_ID}/motion/move_joint",
            callback_group=self.callback_group,
        )

        self.move_line_client = self.create_client(
            MoveLine,
            f"/{ROBOT_ID}/motion/move_line",
            callback_group=self.callback_group,
        )

        while not self.move_joint_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"/{ROBOT_ID}/motion/move_joint 서비스 대기 중...")

        while not self.move_line_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"/{ROBOT_ID}/motion/move_line 서비스 대기 중...")

        # 현재 공정 수행 중인지 여부
        self.is_running = False

        # ====================================================
        # Joint 좌표 설정
        # 단위: degree
        # ====================================================
        self.home_joint = [0, 0, 90, 0, 90, 0]

        self.conveyor_pick_joint = [-39.88, 15.86, 69.32, -0.51, 94.97, -40.12]
        self.conveyor_above_pick_joint = [-39.88, 15.86, 75.72, -0.51, 88.65, -40.12]

        self.inspection_above_joint = [8.48, 10.91, 79.72, 0, 90, 0]

        self.tool_pick_joint = [5.56, 27.54, 83.4, -3.44, 36.31, 5.75]

        # ====================================================
        # Task 좌표 설정
        # 단위: mm, degree = [x, y, z, rx, ry, rz]
        # ====================================================
        self.conveyor_above_pick_posx = [370.26, -301.44, 211.41, 28.05, -179.6, 27.48]
        self.conveyor_pick_posx = [370.26, -301.44, 170.33, 28.05, -179.6, 27.48]

        self.inspection_above_posx = [433.68, 73.82, 184.71, 3, -179.12, -6.38]
        self.inspection_place_posx = [433.68, 73.82, 84.71, 3, -179.12, -6.38]

        self.tool_safe_posx = [719.11, 68.34, 73.95, 2.44, 147.16, 0.18]
        self.tool_place_posx = [744.85, 70.11, 32.38, 2.66, 147.16, 0.39]

        # ====================================================
        # 이동 속도/가속도/시간 설정
        # ====================================================
        self.joint_vel = 30.0
        self.joint_acc = 30.0
        self.line_vel = [50.0, 30.0]
        self.line_acc = [50.0, 30.0]
        self.move_time = 2.0
        self.wait_time = 3.0

        self.get_logger().info("M0609 Pick & Place + Tool Pick 노드 실행됨")
        self.get_logger().info(f"{PROGRESS_TOPIC} topic에서 {START_SIGNAL} 신호 대기 중")
        self.get_logger().info(f"{START_SIGNAL} 수신 시 작업 시작, 작업 완료 후 {DONE_SIGNAL} 송신")

    # ========================================================
    # 로봇 초기화
    # ========================================================
    def initialize_robot(self):
        """
        Tool/TCP/Robot Mode 초기화
        """
        self.get_logger().info("로봇 Tool/TCP 초기화 시작")

        try:
            self.set_robot_mode(self.ROBOT_MODE_MANUAL)
            time.sleep(0.5)

            self.set_tool(ROBOT_TOOL)
            self.set_tcp(ROBOT_TCP)

            self.set_robot_mode(self.ROBOT_MODE_AUTONOMOUS)
            time.sleep(2.0)

            self.get_logger().info("로봇 초기화 완료")
            self.get_logger().info(f"ROBOT_ID: {ROBOT_ID}")
            self.get_logger().info(f"ROBOT_MODEL: {ROBOT_MODEL}")
            self.get_logger().info(f"ROBOT_TCP: {self.get_tcp()}")
            self.get_logger().info(f"ROBOT_TOOL: {self.get_tool()}")
            self.get_logger().info(f"ROBOT_MODE 0:수동, 1:자동 : {self.get_robot_mode()}")

        except Exception as e:
            self.get_logger().error(f"로봇 Tool/TCP 초기화 실패: {e}")

    # ========================================================
    # 진행 상태 callback
    # ========================================================
    def progress_status_callback(self, msg):
        signal = msg.data.strip()

        self.get_logger().info(f"진행 상태 신호 수신: {signal}")

        # 공정 중에는 어떤 신호도 상태 변경에 사용하지 않음
        # 공정 중 01이 다시 들어와도 여기서 무시됨
        if self.is_running:
            self.get_logger().warn("이미 동작 중이라 새 신호를 무시합니다.")
            return

        # 이 노드는 01을 받았을 때만 동작
        if signal != START_SIGNAL:
            self.get_logger().info(
                f"현재 노드 동작 신호가 아닙니다. 수신={signal}, 대기={START_SIGNAL}"
            )
            return

        # 여기서부터 실제 공정 시작
        self.is_running = True

        try:
            success = self.pick_place_and_pick_tool_sequence()

            if success:
                self.get_logger().info("공정 완료: 02 송신")
                self.publish_progress_status(DONE_SIGNAL)

            else:
                self.get_logger().error(
                    f"작업 실패로 인해 {DONE_SIGNAL} 신호를 송신하지 않습니다."
                )

        except Exception as e:
            self.get_logger().error(f"작업 시퀀스 중 오류 발생: {e}")

        finally:
            self.is_running = False

    # ========================================================
    # 진행 상태 publish
    # ========================================================
    def publish_progress_status(self, status_code):
        msg = String()
        msg.data = status_code

        self.progress_status_pub.publish(msg)
        self.get_logger().info(f"진행 상태 송신 완료: {PROGRESS_TOPIC} = {status_code}")

    # ========================================================
    # Pick & Place + Tool Pick 시퀀스
    # ========================================================
    def pick_place_and_pick_tool_sequence(self):
        self.get_logger().info("===== 작업 시작: 물체 가져오기 + 툴 잡기 =====")

        if not self.movej("초기 좌표 복귀", self.home_joint):
            self.safe_stop("초기 좌표 복귀 실패")
            return False

        if not self.open_gripper():
            self.safe_stop("초기 그리퍼 열기 실패")
            return False

        if not self.movel("컨베이어 물체 위 안전 접근 위치 이동", self.conveyor_above_pick_posx):
            self.safe_stop("컨베이어 물체 위 안전 접근 위치 이동 실패")
            return False

        if not self.movel("컨베이어 실제 픽업 위치로 직선 하강", self.conveyor_pick_posx):
            self.safe_stop("컨베이어 실제 픽업 위치 직선 하강 실패")
            return False

        if not self.close_gripper():
            self.safe_stop("컨베이어 물체 잡기 실패")
            return False

        if not self.movel("컨베이어 픽업 후 안전 위치로 직선 상승", self.conveyor_above_pick_posx):
            self.safe_stop("컨베이어 픽업 후 안전 위치 직선 상승 실패")
            return False

        if not self.movel("검사 장소 위 Task 좌표 이동", self.inspection_above_posx):
            self.safe_stop("검사 장소 위 Task 좌표 이동 실패")
            return False

        if not self.movel("검사 장소로 직선 이동", self.inspection_place_posx):
            self.safe_stop("검사 장소 직선 이동 실패")
            return False

        if not self.open_gripper():
            self.safe_stop("검사 장소에서 그리퍼 열기 실패")
            return False

        if not self.movel("검사 장소 위로 직선 이동", self.inspection_above_posx):
            self.safe_stop("검사 장소 위 직선 이동 실패")
            return False

        if not self.movel("도구 접근 전 안전 경유 위치 이동", self.tool_safe_posx):
            self.safe_stop("도구 접근 전 안전 경유 위치 이동 실패")
            return False

        if not self.movel("도구 최종 위치 이동", self.tool_place_posx):
            self.safe_stop("도구 최종 위치 이동 실패")
            return False

        if not self.close_gripper():
            self.safe_stop("도구 잡기 실패")
            return False

        if not self.movel("도구 픽업 후 안전 경유 위치 이동", self.tool_safe_posx):
            self.safe_stop("도구 픽업 후 안전 경유 위치 이동 실패")
            return False

        if not self.movej("툴 잡은 후 초기 좌표 복귀", self.home_joint):
            self.safe_stop("툴 잡은 후 초기 좌표 복귀 실패")
            return False

        self.get_logger().info("===== 작업 완료: 물체 가져오기 + 툴 잡기 + 홈 복귀 완료 =====")
        return True

    # ========================================================
    # 안전 정지
    # ========================================================
    def safe_stop(self, reason):
        self.get_logger().error(f"시퀀스 중단: {reason}")

        try:
            self.open_gripper()
        except Exception as e:
            self.get_logger().warn(f"중단 중 그리퍼 열기 실패: {e}")

        try:
            self.movej("중단 후 초기 좌표 복귀", self.home_joint)
        except Exception as e:
            self.get_logger().warn(f"중단 후 초기 좌표 복귀 실패: {e}")

    # ========================================================
    # Joint 이동
    # ========================================================
    def movej(self, action_name, joint):
        """
        /dsr01/motion/move_joint 서비스를 호출해서 joint 이동 수행
        joint 좌표 단위: degree
        """
        self.get_logger().info(f"{action_name}: posj{joint}")

        request = MoveJoint.Request()

        request.pos = [float(x) for x in joint]
        request.vel = self.joint_vel
        request.acc = self.joint_acc
        request.time = self.move_time
        request.radius = 0.0
        request.mode = 0
        request.blend_type = 0

        request.sync_type = 1

        self.move_joint_client.call_async(request)

        self.get_logger().info(f"{action_name} 명령 전송 완료")

        time.sleep(self.wait_time)

        return True

    # ========================================================
    # Linear 이동
    # ========================================================
    def movel(self, action_name, posx):
        """
        /dsr01/motion/move_line 서비스를 호출해서 task 좌표 직선 이동 수행
        task 좌표 단위: mm, degree = [x, y, z, rx, ry, rz]
        """
        self.get_logger().info(f"{action_name}: posx{posx}")

        request = MoveLine.Request()

        request.pos = [float(x) for x in posx]
        request.vel = self.line_vel
        request.acc = self.line_acc
        request.time = self.move_time
        request.radius = 0.0
        request.ref = 0
        request.mode = 0
        request.blend_type = 0

        request.sync_type = 1

        self.move_line_client.call_async(request)

        self.get_logger().info(f"{action_name} 명령 전송 완료")

        time.sleep(self.wait_time)

        return True

    # ========================================================
    # Digital Input 대기
    # ========================================================
    def wait_digital_input(self, sig_num, timeout=5.0):
        """
        DI 신호가 들어올 때까지 대기.
        """
        start_time = time.time()

        while rclpy.ok():
            try:
                if self.get_digital_input(sig_num):
                    self.get_logger().info(f"DI{sig_num} 입력 확인")
                    return True

            except Exception as e:
                self.get_logger().warn(f"DI{sig_num} 읽기 오류: {e}")
                return False

            if time.time() - start_time > timeout:
                self.get_logger().warn(f"DI{sig_num} 입력 대기 시간 초과")
                return False

            self.dsr_wait(0.1)

        return False

    # ========================================================
    # 그리퍼 닫기
    # ========================================================
    def close_gripper(self):
        """
        교안 grip() 기반 그리퍼 닫기
        DO1 = ON, DO2 = OFF
        DI1 입력 확인
        """
        self.get_logger().info("그리퍼 닫기 시작: DO1 ON, DO2 OFF")

        try:
            self.get_logger().info("DO1 ON 명령 전송 시작")
            self.set_digital_output(1, ON)
            self.get_logger().info("DO1 ON 명령 전송 완료")

            self.get_logger().info("DO2 OFF 명령 전송 시작")
            self.set_digital_output(2, OFF)
            self.get_logger().info("DO2 OFF 명령 전송 완료")

            if not self.wait_digital_input(1, timeout=5.0):
                self.get_logger().warn("그리퍼 닫힘 DI1 확인 실패")
                return False

            self.get_logger().info("그리퍼 닫기 완료")
            return True

        except Exception as e:
            self.get_logger().error(f"그리퍼 닫기 오류: {e}")
            return False

    # ========================================================
    # 그리퍼 열기
    # ========================================================
    def open_gripper(self):
        """
        교안 release() 기반 그리퍼 열기
        DO2 = ON, DO1 = OFF
        DI2 입력 확인
        """
        self.get_logger().info("그리퍼 열기 시작: DO2 ON, DO1 OFF")

        try:
            self.get_logger().info("DO2 ON 명령 전송 시작")
            self.set_digital_output(2, ON)
            self.get_logger().info("DO2 ON 명령 전송 완료")

            self.get_logger().info("DO1 OFF 명령 전송 시작")
            self.set_digital_output(1, OFF)
            self.get_logger().info("DO1 OFF 명령 전송 완료")

            if not self.wait_digital_input(2, timeout=5.0):
                self.get_logger().warn("그리퍼 열림 DI2 확인 실패")
                return False

            self.get_logger().info("그리퍼 열기 완료")
            return True

        except Exception as e:
            self.get_logger().error(f"그리퍼 열기 오류: {e}")
            return False


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