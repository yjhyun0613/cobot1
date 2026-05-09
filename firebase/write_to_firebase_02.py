import time
import threading
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db

import rclpy
from rclpy.node import Node
# 두산 전용 메시지 타입 (환경에 맞게 수정 필요)
# from dsr_msgs2.msg import RobotState 
from sensor_msgs.msg import JointState 

class DoosanWebControlNode(Node):
    def __init__(self):
        super().__init__('doosan_web_control_node')
        
        # 1. Firebase 초기화
        SERVICE_ACCOUNT_KEY_PATH = "/home/yoon/cobot_ws/src/cobot1/rokey-d-2-4c32a-firebase-adminsdk-fbsvc-1bc6c65e35.json" 
        DATABASE_URL = "https://rokey-d-2-4c32a-default-rtdb.asia-southeast1.firebasedatabase.app" 
        
        if not firebase_admin._apps:
            cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
            firebase_admin.initialize_app(cred, {'databaseURL': DATABASE_URL})

        self.status_ref = db.reference('/m0609/status')
        self.command_ref = db.reference('/m0609/command')

        # 2. 로봇 상태 퍼블리시 (웹으로 전송)
        self.subscription = self.create_subscription(
            JointState, '/dsr01/joint_states', self.robot_state_callback, 10
        )
        self.last_update_time = time.time()

        # 3. Firebase 명령 리스너 백그라운드 실행
        # 웹에서 버튼을 누르면 command_listener 함수가 호출됩니다.
        self.listener_thread = threading.Thread(target=self.start_firebase_listener, daemon=True)
        self.listener_thread.start()

        self.get_logger().info("양방향 웹 인터페이스 노드가 시작되었습니다.")
        self.send_log_to_web("[INFO] 시스템 준비 완료. 대기 중...")

    # ==========================================
    # 웹 -> 파이썬 (명령 수신부)
    # ==========================================
    def start_firebase_listener(self):
        # Firebase 데이터 변경 이벤트 감지
        self.command_ref.listen(self.command_listener)

    def command_listener(self, event):
        # 웹에서 새로운 명령이 들어왔을 때 실행
        if event.data is None: return
        
        data = event.data
        if not isinstance(data, dict): return
        
        cmd_type = data.get('type')
        
        if cmd_type == 'movej':
            target_j = data.get('target_joint', [])
            vel = data.get('vel', 60)
            acc = data.get('acc', 60)
            log_msg = f"[CMD_RCV] movej 실행 (Target: {target_j}, V:{vel}, A:{acc})"
            self.get_logger().info(log_msg)
            self.send_log_to_web(log_msg)
            self.status_ref.update({'system_status': 'MoveJ 실행 중'})
            
            # TODO: 여기에 실제 두산 로봇 MoveJ 서비스/액션 호출 코드 추가

        elif cmd_type == 'movel':
            target_p = data.get('target_pose', [])
            vel = data.get('vel', 60)
            acc = data.get('acc', 60)
            log_msg = f"[CMD_RCV] movel 실행 (Target: {target_p}, V:{vel}, A:{acc})"
            self.get_logger().info(log_msg)
            self.send_log_to_web(log_msg)
            self.status_ref.update({'system_status': 'MoveL 실행 중'})
            
            # TODO: 여기에 실제 두산 로봇 MoveL 서비스/액션 호출 코드 추가

        elif cmd_type == 'home':
            self.send_log_to_web("[CMD_RCV] HOME 복귀 명령 실행")
            self.status_ref.update({'system_status': '홈 위치로 이동 중'})
            # TODO: Home 이동 코드

    # ==========================================
    # 파이썬 -> 웹 (상태 및 로그 송신부)
    # ==========================================
    def send_log_to_web(self, log_message):
        # 웹 UI의 로그 창에 텍스트 띄우기
        self.status_ref.update({'latest_log': log_message})

    def robot_state_callback(self, msg):
        current_time = time.time()
        # 트래픽 과부하 방지: 0.5초마다 한 번씩만 웹으로 전송
        if current_time - self.last_update_time < 0.5:
            return
            
        self.last_update_time = current_time

        # 임시 예시 데이터 (실제 토픽 메시지에서 추출해야 함)
        j_pos = msg.position if (msg.position and len(msg.position) >= 6) else [0.0]*6
        
        # Cartesian(TCP) 좌표는 /joint_states에 없으므로 TF나 두산 전용 로봇 상태 토픽에서 받아와야 합니다.
        # 여기서는 임시 0 데이터로 처리했습니다.
        tcp_pos = [367.315, 6.992, 424.293, 98.566, 179.978, 98.468] 

        try:
            self.status_ref.update({
                'joint': [round(j, 3) for j in j_pos],
                'pose': [round(p, 3) for p in tcp_pos],
                'timestamp': current_time
            })
        except Exception as e:
            self.get_logger().error(f"상태 전송 실패: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DoosanWebControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()