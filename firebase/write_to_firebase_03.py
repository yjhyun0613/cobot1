import time
import threading
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState 

class DoosanFlaskRobotNode(Node):
    def __init__(self):
        super().__init__('doosan_flask_robot_node')
        
        # =========================================================
        # 1. Firebase Admin SDK 초기화
        # =========================================================
        SERVICE_ACCOUNT_KEY_PATH = "/home/yoon/cobot_ws/src/cobot1/rokey-d-2-4c32a-firebase-adminsdk-fbsvc-1bc6c65e35.json" 
        DATABASE_URL = "https://rokey-d-2-4c32a-default-rtdb.asia-southeast1.firebasedatabase.app" 
        
        if not firebase_admin._apps:
            try:
                cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
                firebase_admin.initialize_app(cred, {'databaseURL': DATABASE_URL})
                self.get_logger().info("Firebase 초기화 성공")
            except Exception as e:
                self.get_logger().error(f"Firebase 초기화 실패: {e}")

        self.status_ref = db.reference('robot_status/current')
        self.command_ref = db.reference('robot_commands')

        # =========================================================
        # 2. 로봇 상태 관리 변수 초기화
        # =========================================================
        self.current_joint_pos = [0.0] * 6
        # TODO: 현재 토픽 리스트에 TCP 직접 출력 토픽이 없으므로, 
        # 향후 tf2_ros를 사용하여 /tf 토픽에서 base_link -> tcp 변환 값을 구해와야 합니다.
        self.current_tcp_pos = [367.315, 6.992, 424.293, 98.566, 179.978, 98.468] 
        self.tcp_ready = False
        self.tool_ready = False
        self.gripper_model = "GripperDA_mh"
        self.tool_weight = 1.2
        self.status_message = "시스템 대기 중"

        # =========================================================
        # 3. ROS 2 구독 및 타이머 설정 (수정된 부분)
        # =========================================================
        # ros2 topic list에 맞춰 네임스페이스가 포함된 정확한 토픽명 지정
        self.subscription = self.create_subscription(
            JointState, 
            '/dsr01/joint_states', # <--- 이 부분이 실제 로봇 데이터 경로로 변경되었습니다.
            self.joint_callback, 
            10
        )
        
        # 0.5초(2Hz)마다 현재 상태를 Firebase로 전송
        self.status_timer = self.create_timer(0.5, self.publish_status_to_firebase)

        # =========================================================
        # 4. 명령 수신 리스너 (백그라운드)
        # =========================================================
        self.listener_thread = threading.Thread(target=self.start_firebase_listener, daemon=True)
        self.listener_thread.start()

        self.get_logger().info("Flask 연동 두산(dsr01) 로봇 제어 노드가 시작되었습니다.")
        self.status_message = "로봇 노드 연결 완료"
        self.publish_status_to_firebase()


    # ---------------------------------------------------------
    # 로봇 상태 수신 콜백 (ROS 2 -> Python 변수)
    # ---------------------------------------------------------
    def joint_callback(self, msg):
        # /dsr01/joint_states 토픽에서 들어온 6축 조인트 각도 데이터를 저장합니다.
        if msg.position and len(msg.position) >= 6:
            self.current_joint_pos = list(msg.position[:6])


    # ---------------------------------------------------------
    # 로봇 상태 퍼블리시 (Python 변수 -> Firebase)
    # ---------------------------------------------------------
    def publish_status_to_firebase(self):
        try:
            status_data = {
                "tcp_ready": self.tcp_ready,
                "tool_ready": self.tool_ready,
                "gripper_model": self.gripper_model,
                "tool_weight": self.tool_weight,
                "tcp_pos": [round(p, 3) for p in self.current_tcp_pos],
                "joint_pos": [round(j, 3) for j in self.current_joint_pos],
                "message": self.status_message
            }
            self.status_ref.set(status_data)
        except Exception as e:
            self.get_logger().error(f"상태 전송 실패: {e}")


    # ---------------------------------------------------------
    # 웹 명령 수신부 (Firebase -> Python 처리)
    # ---------------------------------------------------------
    def start_firebase_listener(self):
        self.command_ref.listen(self.command_listener)

    def command_listener(self, event):
        if event.data is None: return
        
        data_to_process = {}
        
        if event.path == '/':
            if isinstance(event.data, dict):
                data_to_process = event.data
        else:
            key = event.path.strip('/')
            data_to_process = {key: event.data}

        for key, cmd_data in data_to_process.items():
            if isinstance(cmd_data, dict) and cmd_data.get("status") == "pending":
                self.process_command(key, cmd_data)


    def process_command(self, key, cmd_data):
        command = cmd_data.get("command")
        pos = cmd_data.get("pos")
        vel = cmd_data.get("vel", 60)
        acc = cmd_data.get("acc", 60)
        
        self.get_logger().info(f"새로운 명령 수신: {command}")
        
        self.command_ref.child(key).update({"status": "processing"})
        self.status_message = f"[{command}] 명령 실행 중..."
        self.publish_status_to_firebase()

        # =========================================================
        # 실제 로봇 구동 Action / Service 연결부
        # =========================================================
        if command == "movej":
            self.get_logger().info(f"-> MoveJ: 목표={pos}, V={vel}, A={acc}")
            # 향후 dsr_msgs2 패키지의 MoveJoint 서비스를 여기에 호출합니다.

        elif command == "movel":
            self.get_logger().info(f"-> MoveL: 목표={pos}, V={vel}, A={acc}")
            # 향후 dsr_msgs2 패키지의 MoveLine 서비스를 여기에 호출합니다.

        elif command == "home":
            self.get_logger().info("-> Home 이동 명령")

        elif command == "tcp_check":
            self.tcp_ready = True
            self.get_logger().info("-> TCP 확인 완료")

        elif command == "tool_check":
            self.tool_ready = True
            self.get_logger().info("-> Tool 확인 완료")

        elif command == "tool_on":
            self.tool_ready = True
            self.get_logger().info("-> Tool ON (dsr01/io/set_ctrl_box_digital_output 호출 등)")

        elif command == "tool_off":
            self.tool_ready = False
            self.get_logger().info("-> Tool OFF")

        self.command_ref.child(key).update({"status": "completed"})
        self.status_message = f"[{command}] 완료됨"
        self.publish_status_to_firebase()
        self.get_logger().info(f"명령 처리 완료: {command}")


def main(args=None):
    rclpy.init(args=args)
    node = DoosanFlaskRobotNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("종료 중...")
    finally:
        node.status_message = "로봇 제어 노드 오프라인"
        node.publish_status_to_firebase()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()