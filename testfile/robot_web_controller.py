import time
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState 

# Firebase 관련
import firebase_admin
from firebase_admin import credentials, db

# ROS 2 서비스 타입 (Doosan 전용)
import DR_init
from dsr_msgs2.srv import MoveJoint, MoveLine, MoveHome

# =========================================================
# 1. 수학 계산 함수 (정기구학 및 오일러 각도 변환)
# =========================================================
def dh_transform_matrix(theta, a, d, alpha):
    return np.array([
        [np.cos(theta), -np.sin(theta)*np.cos(alpha),  np.sin(theta)*np.sin(alpha), a*np.cos(theta)],
        [np.sin(theta),  np.cos(theta)*np.cos(alpha), -np.cos(theta)*np.sin(alpha), a*np.sin(theta)],
        [0,              np.sin(alpha),                np.cos(alpha),               d],
        [0,              0,                            0,                           1]
    ])

def forward_kinematics_m0609(joint_angles):
    d1, a2, a3 = 0.135, 0.411, 0.368
    d4, d5, d6 = 0.120, 0.120, 0.120

    dh_params = [
        [joint_angles[0],  0, d1,  np.pi/2],
        [joint_angles[1], a2,  0,        0],
        [joint_angles[2], a3,  0,        0],
        [joint_angles[3],  0, d4,  np.pi/2],
        [joint_angles[4],  0, d5, -np.pi/2],
        [joint_angles[5],  0, d6,        0]
    ]

    T_base_to_end = np.eye(4)
    for params in dh_params:
        theta, a, d, alpha = params
        T_link = dh_transform_matrix(theta, a, d, alpha)
        T_base_to_end = np.dot(T_base_to_end, T_link)
        
    return T_base_to_end

def rotation_matrix_to_euler(R):
    sy = np.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-6
    if not singular:
        rx = np.arctan2(R[2, 1], R[2, 2])
        ry = np.arctan2(-R[2, 0], sy)
        rz = np.arctan2(R[1, 0], R[0, 0])
    else:
        rx = np.arctan2(-R[1, 2], R[1, 1])
        ry = np.arctan2(-R[2, 0], sy)
        rz = 0
    return rx, ry, rz

# =========================================================
# 2. 통합 ROS 2 노드 (상태 모니터링 + 로봇 제어)
# =========================================================
class RobotWebController(Node):
    def __init__(self, robot_id):
        super().__init__('robot_web_controller', namespace=robot_id)
        self.robot_id = robot_id
        
        # ---------------------------------------------------------
        # Firebase Admin SDK 초기화
        # ---------------------------------------------------------
        SERVICE_ACCOUNT_KEY_PATH = "/home/yoon/cobot_ws/src/cobot1/rokey-d-2-4c32a-firebase-adminsdk-fbsvc-1bc6c65e35.json" 
        DATABASE_URL = "https://rokey-d-2-4c32a-default-rtdb.asia-southeast1.firebasedatabase.app" 
        
        if not firebase_admin._apps:
            try:
                cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
                firebase_admin.initialize_app(cred, {'databaseURL': DATABASE_URL})
                self.get_logger().info("Firebase 앱 초기화 성공")
            except Exception as e:
                self.get_logger().error(f"Firebase 초기화 실패: {e}")

        # Firebase 참조 경로 (웹과 일치)
        self.status_ref = db.reference('m0609/status')
        self.cmd_ref = db.reference('m0609/command')

        # ---------------------------------------------------------
        # 로봇 상태 구독 (정기구학) 설정
        # ---------------------------------------------------------
        self.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        self.subscription = self.create_subscription(
            JointState, f'/{self.robot_id}/joint_states', self.robot_state_callback, 10)

        # ---------------------------------------------------------
        # 로봇 제어 서비스 클라이언트 설정
        # ---------------------------------------------------------
        self.srv_move_joint = self.create_client(MoveJoint, f"/{self.robot_id}/motion/move_joint")
        self.srv_move_line = self.create_client(MoveLine, f"/{self.robot_id}/motion/move_line")
        self.srv_move_home = self.create_client(MoveHome, f"/{self.robot_id}/motion/move_home")
        
        self.wait_for_services()

        # ---------------------------------------------------------
        # 웹 명령 감시 (스레드 실행)
        # ---------------------------------------------------------
        self.last_timestamp = None
        self.listener_thread = threading.Thread(target=self.start_firebase_listener, daemon=True)
        self.listener_thread.start()

        self.get_logger().info("통합 웹 컨트롤러 노드가 성공적으로 시작되었습니다.")

    def wait_for_services(self):
        services = [(self.srv_move_joint, "MoveJoint"), (self.srv_move_line, "MoveLine"), (self.srv_move_home, "MoveHome")]
        for client, name in services:
            if client.wait_for_service(timeout_sec=5.0):
                self.get_logger().info(f"{name} 서비스 준비 완료")
            else:
                self.get_logger().warn(f"{name} 서비스를 찾을 수 없습니다.")

    # =========================================================
    # 기능 1: 로봇 상태를 Firebase로 전송 (읽기)
    # =========================================================
    def robot_state_callback(self, msg):
        if msg.position and len(msg.position) >= 6:
            try:
                joint_dict = dict(zip(msg.name, msg.position))
                current_angles = [joint_dict[name] for name in self.joint_names]
                
                T = forward_kinematics_m0609(current_angles)
                x, y, z = T[0, 3] * 1000, T[1, 3] * 1000, T[2, 3] * 1000
                R = T[0:3, 0:3]
                rx, ry, rz = rotation_matrix_to_euler(R)
                a, b, c = np.rad2deg(rx), np.rad2deg(ry), np.rad2deg(rz)

                # HTML에서 "COPY" 버튼을 눌렀을 때 배열 형식으로 받을 수 있게 업데이트
                self.status_ref.update({
                    'joint': [round(np.rad2deg(current_angles[i]), 3) for i in range(6)],
                    'pose': [round(x, 3), round(y, 3), round(z, 3), round(a, 3), round(b, 3), round(c, 3)],
                    'system_status': "로봇 제어 모드 활성화",
                    'last_update_timestamp': time.time()
                })
            except Exception as e:
                pass # 에러 로깅 생략 (초당 여러번 발생하는 데이터이므로)

    # =========================================================
    # 기능 2: 웹에서 보내는 명령 수신 및 제어 (쓰기)
    # =========================================================
    def start_firebase_listener(self):
        # m0609/command 에 변경이 생기면 on_command_received 함수 실행
        self.cmd_ref.listen(self.on_command_received)

    def on_command_received(self, event):
        command = event.data if event.path == "/" else self.cmd_ref.get()
        if not isinstance(command, dict):
            return

        timestamp = command.get("timestamp")
        if timestamp is None or timestamp == self.last_timestamp:
            return # 중복 명령 방지
        self.last_timestamp = timestamp

        action = command.get("action")
        self.get_logger().info(f"웹 명령 수신: {action}")

        # 로봇 제어 서비스 호출
        success = False
        if action == "movej":
            success = self.call_move_joint(command.get("pos"), command.get("vel", 60), command.get("acc", 60))
        elif action == "movel":
            success = self.call_move_line(command.get("pos"), command.get("vel", 60), command.get("acc", 60))
        elif action == "home":
            success = self.call_move_home()

        # 로그 업데이트
        status_msg = f"[{action.upper()}] 명령이 성공적으로 전송되었습니다." if success else f"[{action.upper()}] 명령 전송 실패"
        self.status_ref.update({'latest_log': status_msg})


    # ---------------------------------------------------------
    # 로봇 모션 제어 서비스 호출부
    # ---------------------------------------------------------
    def call_move_joint(self, pos, vel, acc):
        try:
            req = MoveJoint.Request()
            req.pos = [float(x) for x in pos]
            req.vel = float(vel)
            req.acc = float(acc)
            # DSR 펌웨어 버전에 따른 호환성 처리
            if hasattr(req, "time"): req.time = 0.0
            if hasattr(req, "radius"): req.radius = 0.0
            if hasattr(req, "mode"): req.mode = 0

            future = self.srv_move_joint.call_async(req)
            return True
        except Exception as e:
            self.get_logger().error(f"MoveJoint 예외: {e}")
            return False

    def call_move_line(self, pos, vel, acc):
        try:
            req = MoveLine.Request()
            req.pos = [float(x) for x in pos]
            req.vel = [float(vel), float(vel)] # 선속도, 각속도
            req.acc = [float(acc), float(acc)]
            if hasattr(req, "time"): req.time = 0.0
            if hasattr(req, "radius"): req.radius = 0.0

            future = self.srv_move_line.call_async(req)
            return True
        except Exception as e:
            self.get_logger().error(f"MoveLine 예외: {e}")
            return False

    def call_move_home(self):
        try:
            req = MoveHome.Request()
            future = self.srv_move_home.call_async(req)
            return True
        except Exception as e:
            self.get_logger().error(f"MoveHome 예외: {e}")
            return False

# =========================================================
# 3. 메인 실행부
# =========================================================
def main(args=None):
    ROBOT_ID = "dsr01"
    ROBOT_MODEL = "m0609"

    DR_init.__dsr__id = ROBOT_ID
    DR_init.__dsr__model = ROBOT_MODEL

    rclpy.init(args=args)
    bridge_node = RobotWebController(ROBOT_ID)
    DR_init.__dsr__node = bridge_node

    # DSR 로봇 오토너머스 모드 설정 (실제 로봇 제어 필수)
    try:
        from DSR_ROBOT2 import set_robot_mode, ROBOT_MODE_AUTONOMOUS
        set_robot_mode(ROBOT_MODE_AUTONOMOUS)
    except ImportError:
        bridge_node.get_logger().warn("DSR_ROBOT2 모듈을 찾을 수 없습니다. 시뮬레이션 환경이라면 무시하세요.")

    try:
        rclpy.spin(bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        bridge_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()