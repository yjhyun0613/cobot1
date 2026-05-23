# service_bridge.py

import time
import threading

import rclpy
from rclpy.node import Node

import DR_init

# ROS 2 서비스 타입 Doosan 전용
from dsr_msgs2.srv import MoveJoint, MoveLine, MoveHome, ChangeOperationSpeed

# Firebase 관련
import firebase_admin
from firebase_admin import credentials, db


# =========================================================
# Firebase 설정
# =========================================================

DATABASE_URL = "https://rokey-d-2-4c32a-default-rtdb.asia-southeast1.firebasedatabase.app"

# service_bridge.py와 같은 폴더에 serviceAccountKey.json을 둘 경우
SERVICE_ACCOUNT_KEY_PATH = "/home/yoon/cobot_ws/src/cobot1/rokey-d-2-4c32a-firebase-adminsdk-fbsvc-1bc6c65e35.json"

if not firebase_admin._apps:
    cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
    firebase_admin.initialize_app(cred, {
        "databaseURL": DATABASE_URL
    })


class RobotCommandBridge(Node):
    def __init__(self, robot_id):
        super().__init__(
            "robot_command_bridge",
            namespace=robot_id
        )

        self.robot_id = robot_id

        # 같은 명령 중복 실행 방지용
        self.last_timestamp = None

        # =========================================================
        # Doosan ROS 2 서비스 클라이언트 생성
        # =========================================================

        self.srv_move_joint = self.create_client(
            MoveJoint,
            f"/{self.robot_id}/motion/move_joint"
        )

        self.srv_move_line = self.create_client(
            MoveLine,
            f"/{self.robot_id}/motion/move_line"
        )

        self.srv_move_home = self.create_client(
            MoveHome,
            f"/{self.robot_id}/motion/move_home"
        )

        self.srv_change_speed = self.create_client(
            ChangeOperationSpeed,
            f"/{self.robot_id}/motion/change_operation_speed"
        )

        self.wait_for_services()

        # =========================================================
        # Firebase 명령 경로
        # =========================================================
        # app.py가 여기에 명령을 저장함
        self.cmd_ref = db.reference(f"commands/{self.robot_id}")

        # 로봇 상태 저장 경로
        self.status_ref = db.reference("robot_status/current")

        # Firebase listener는 blocking이므로 별도 thread에서 실행
        self.listener_thread = threading.Thread(
            target=self.start_firebase_listener,
            daemon=True
        )
        self.listener_thread.start()

        self.update_robot_status(
            message="service_bridge 연결 완료",
            bridge_ready=True
        )

        self.get_logger().info("--- Web to Robot Service Bridge Active ---")

    # =========================================================
    # 서비스 연결 확인
    # =========================================================

    def wait_for_services(self):
        service_list = [
            (self.srv_move_joint, "MoveJoint", f"/{self.robot_id}/motion/move_joint"),
            (self.srv_move_line, "MoveLine", f"/{self.robot_id}/motion/move_line"),
            (self.srv_move_home, "MoveHome", f"/{self.robot_id}/motion/move_home"),
            (self.srv_change_speed, "ChangeOperationSpeed", f"/{self.robot_id}/motion/change_operation_speed")
        ]

        for client, name, path in service_list:
            self.get_logger().info(f"{name} 서비스 연결 확인 중: {path}")

            if client.wait_for_service(timeout_sec=5.0):
                self.get_logger().info(f"{name} 서비스 연결 완료")
            else:
                self.get_logger().error(f"{name} 서비스를 찾을 수 없습니다: {path}")

    # =========================================================
    # Firebase 상태 업데이트
    # =========================================================

    def update_robot_status(self, message="", bridge_ready=True):
        try:
            status_data = {
                "bridge_ready": bridge_ready,
                "message": message,
                "updated_at": int(time.time() * 1000)
            }

            self.status_ref.update(status_data)

        except Exception as e:
            self.get_logger().error(f"robot_status 업데이트 실패: {e}")

    # =========================================================
    # Firebase listener 시작
    # =========================================================

    def start_firebase_listener(self):
        self.cmd_ref.listen(self.on_command_received)

    # =========================================================
    # Firebase 명령 수신 콜백
    # =========================================================

    def on_command_received(self, event):
        """
        app.py가 commands/dsr01에 저장한 명령을 수신한다.

        예시:
        {
            "action": "move_joint",
            "pos": [0, 0, 90, 0, 90, 0],
            "vel": 60,
            "acc": 60,
            "status": "pending",
            "timestamp": 123456789
        }
        """

        if event.data is None:
            return

        # commands/dsr01 전체가 변경된 경우
        if event.path == "/":
            command = event.data

        # /action, /status 등 일부만 변경된 경우 전체 명령을 다시 읽음
        else:
            command = self.cmd_ref.get()

        if not isinstance(command, dict):
            return

        # pending 상태인 명령만 실행
        if command.get("status") != "pending":
            return

        timestamp = command.get("timestamp")

        # 같은 timestamp 명령 중복 실행 방지
        if timestamp is not None and timestamp == self.last_timestamp:
            return

        self.last_timestamp = timestamp

        action = command.get("action")

        self.get_logger().info(f"새로운 명령 수신: {command}")

        self.cmd_ref.update({
            "status": "processing",
            "processing_at": int(time.time() * 1000)
        })

        self.update_robot_status(
            message=f"[{action}] 명령 실행 중...",
            bridge_ready=True
        )

        success = False

        # =========================================================
        # action에 따른 실제 서비스 호출
        # =========================================================

        if action == "move_joint":
            success = self.call_move_joint(
                pos=command.get("pos"),
                vel=command.get("vel", 60.0),
                acc=command.get("acc", 60.0)
            )

        elif action == "move_line":
            success = self.call_move_line(
                pos=command.get("pos"),
                vel=command.get("vel", 100.0),
                acc=command.get("acc", 100.0)
            )

        elif action == "move_home":
            success = self.call_move_home()

        elif action == "change_speed":
            success = self.call_change_speed(
                speed=command.get("speed", 100)
            )

        elif action == "tool_on":
            # 현재는 실제 그리퍼 IO 서비스 연결 전이므로 로그 처리
            self.get_logger().info("Tool ON 명령 수신")
            success = True

        elif action == "tool_off":
            # 현재는 실제 그리퍼 IO 서비스 연결 전이므로 로그 처리
            self.get_logger().info("Tool OFF 명령 수신")
            success = True

        elif action == "tcp_check":
            self.get_logger().info("TCP Check 명령 수신")
            success = True

        elif action == "tool_check":
            self.get_logger().info("Tool Check 명령 수신")
            success = True

        else:
            self.get_logger().error(f"알 수 없는 action입니다: {action}")
            success = False

        # =========================================================
        # 실행 결과 Firebase에 반영
        # =========================================================

        if success:
            self.cmd_ref.update({
                "status": "completed",
                "completed_at": int(time.time() * 1000)
            })

            self.update_robot_status(
                message=f"[{action}] 명령 완료",
                bridge_ready=True
            )

            self.get_logger().info(f"명령 처리 완료: {action}")

        else:
            self.cmd_ref.update({
                "status": "failed",
                "failed_at": int(time.time() * 1000)
            })

            self.update_robot_status(
                message=f"[{action}] 명령 실패",
                bridge_ready=True
            )

            self.get_logger().error(f"명령 처리 실패: {action}")

    # =========================================================
    # MoveJoint 서비스 호출
    # =========================================================

    def call_move_joint(self, pos, vel, acc):
        try:
            if pos is None or len(pos) != 6:
                self.get_logger().error("MoveJoint pos는 6개 값이어야 합니다.")
                return False

            if not self.srv_move_joint.service_is_ready():
                self.get_logger().error("MoveJoint 서비스가 준비되지 않았습니다.")
                return False

            req = MoveJoint.Request()

            req.pos = [float(x) for x in pos]
            req.vel = float(vel)
            req.acc = float(acc)

            # 사용 중인 dsr_msgs2 버전에 따라 필드가 있을 수도 있으므로 안전하게 처리
            if hasattr(req, "time"):
                req.time = 0.0

            if hasattr(req, "radius"):
                req.radius = 0.0

            if hasattr(req, "mode"):
                req.mode = 0

            if hasattr(req, "blend_type"):
                req.blend_type = 0

            if hasattr(req, "sync_type"):
                req.sync_type = 0

            self.get_logger().info(
                f"MoveJoint 요청: pos={req.pos}, vel={req.vel}, acc={req.acc}"
            )

            future = self.srv_move_joint.call_async(req)
            future.add_done_callback(self.service_response_callback)

            return True

        except Exception as e:
            self.get_logger().error(f"MoveJoint 호출 중 예외 발생: {e}")
            return False

    # =========================================================
    # MoveLine 서비스 호출
    # =========================================================

    def call_move_line(self, pos, vel, acc):
        try:
            if pos is None or len(pos) != 6:
                self.get_logger().error("MoveLine pos는 6개 값이어야 합니다.")
                return False

            if not self.srv_move_line.service_is_ready():
                self.get_logger().error("MoveLine 서비스가 준비되지 않았습니다.")
                return False

            req = MoveLine.Request()

            req.pos = [float(x) for x in pos]

            # MoveLine의 vel/acc는 보통 [선속도, 각속도] 형태
            req.vel = [float(vel), float(vel)]
            req.acc = [float(acc), float(acc)]

            if hasattr(req, "time"):
                req.time = 0.0

            if hasattr(req, "radius"):
                req.radius = 0.0

            if hasattr(req, "ref"):
                req.ref = 0

            if hasattr(req, "mode"):
                req.mode = 0

            if hasattr(req, "blend_type"):
                req.blend_type = 0

            if hasattr(req, "sync_type"):
                req.sync_type = 0

            self.get_logger().info(
                f"MoveLine 요청: pos={req.pos}, vel={req.vel}, acc={req.acc}"
            )

            future = self.srv_move_line.call_async(req)
            future.add_done_callback(self.service_response_callback)

            return True

        except Exception as e:
            self.get_logger().error(f"MoveLine 호출 중 예외 발생: {e}")
            return False

    # =========================================================
    # MoveHome 서비스 호출
    # =========================================================

    def call_move_home(self):
        try:
            if not self.srv_move_home.service_is_ready():
                self.get_logger().error("MoveHome 서비스가 준비되지 않았습니다.")
                return False

            req = MoveHome.Request()

            self.get_logger().info("MoveHome 요청")

            future = self.srv_move_home.call_async(req)
            future.add_done_callback(self.service_response_callback)

            return True

        except Exception as e:
            self.get_logger().error(f"MoveHome 호출 중 예외 발생: {e}")
            return False

    # =========================================================
    # ChangeOperationSpeed 서비스 호출
    # =========================================================

    def call_change_speed(self, speed):
        try:
            if not self.srv_change_speed.service_is_ready():
                self.get_logger().error("ChangeOperationSpeed 서비스가 준비되지 않았습니다.")
                return False

            req = ChangeOperationSpeed.Request()
            req.speed = int(speed)

            self.get_logger().info(f"ChangeOperationSpeed 요청: speed={req.speed}")

            future = self.srv_change_speed.call_async(req)
            future.add_done_callback(self.service_response_callback)

            return True

        except Exception as e:
            self.get_logger().error(f"ChangeOperationSpeed 호출 중 예외 발생: {e}")
            return False

    # =========================================================
    # 서비스 응답 콜백
    # =========================================================

    def service_response_callback(self, future):
        try:
            result = future.result()
            self.get_logger().info(f"서비스 응답 수신: {result}")

        except Exception as e:
            self.get_logger().error(f"서비스 응답 처리 중 예외 발생: {e}")


def main(args=None):
    ROBOT_ID = "dsr01"
    ROBOT_MODEL = "m0609"

    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    rclpy.init(args=args)

    bridge_node = RobotCommandBridge(ROBOT_ID)
    DR_init.__dsr__node = bridge_node

    # Doosan 로봇을 autonomous mode로 설정
    from DSR_ROBOT2 import set_robot_mode, ROBOT_MODE_AUTONOMOUS
    set_robot_mode(ROBOT_MODE_AUTONOMOUS)

    try:
        rclpy.spin(bridge_node)

    except KeyboardInterrupt:
        bridge_node.get_logger().info("service_bridge 종료 중...")

    finally:
        bridge_node.update_robot_status(
            message="service_bridge 오프라인",
            bridge_ready=False
        )

        bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()