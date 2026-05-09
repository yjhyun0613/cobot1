import time
import firebase_admin
from firebase_admin import credentials
from firebase_admin import db

import rclpy
from rclpy.node import Node
# 두산 로봇 메시지 타입에 맞게 수정 필요 (여기서는 범용 JointState 예시)
from sensor_msgs.msg import JointState 

class DoosanFirebaseNode(Node):
    def __init__(self):
        super().__init__('doosan_firebase_node')
        
        # 1. Firebase Admin SDK 초기화 (기존 코드 활용)
        SERVICE_ACCOUNT_KEY_PATH = "/home/yoon/cobot_ws/src/cobot1/rokey-d-2-4c32a-firebase-adminsdk-fbsvc-1bc6c65e35.json" 
        DATABASE_URL = "https://rokey-d-2-4c32a-default-rtdb.asia-southeast1.firebasedatabase.app" 
        
        # Firebase가 여러 번 초기화되는 것을 방지
        if not firebase_admin._apps:
            try:
                cred = credentials.Certificate(SERVICE_ACCOUNT_KEY_PATH)
                firebase_admin.initialize_app(cred, {
                    'databaseURL': DATABASE_URL
                })
                self.get_logger().info("Firebase 앱 초기화 성공")
            except Exception as e:
                self.get_logger().error(f"Firebase 초기화 실패: {e}")

        # 데이터를 쓸 DB 경로 참조 변경 (예: m0609_status)
        self.ref = db.reference('/m0609_status')

        # 2. 로봇 데이터 구독 (Subscriber) 생성
        # 실제 두산 로봇 패키지(dsr_rospkg2 등)에서 발행하는 로봇 상태 토픽으로 변경해야 합니다.
        self.subscription = self.create_subscription(
            JointState,          # 수신할 메시지 타입
            '/dsr01/joint_states',     # 수신할 토픽 이름 (예시)
            self.robot_state_callback, # 콜백 함수
            10                   # QoS 프로파일
        )
        self.get_logger().info("로봇 데이터 구독을 시작합니다...")

    def robot_state_callback(self, msg):
        """
        로봇에서 새로운 데이터가 들어올 때마다 실행되는 함수
        """
        # 3. 로봇 데이터 추출 (메시지 구조에 따라 다름)
        # 예시로 첫 번째 조인트의 각도(위치) 값을 가져옵니다.
        if msg.position:
            j1_pos = msg.position[0]
            j2_pos = msg.position[1]
            j3_pos = msg.position[2]
            j4_pos = msg.position[3]
            j5_pos = msg.position[4]
            j6_pos = msg.position[5]
        else:
            j1_pos, j2_pos = 0.0, 0.0

        # 4. Firebase DB에 실제 데이터 쓰기 (업데이트)
        try:
            self.ref.update({
                'joint1_position': round(j1_pos, 4),
                'joint2_position': round(j2_pos, 4),
                'joint3_position': round(j3_pos, 4),
                'joint4_position': round(j4_pos, 4),
                'joint5_position': round(j5_pos, 4),
                'joint6_position': round(j6_pos, 4),
                'last_update_timestamp': time.time()
            })
            self.get_logger().info(f"Firebase 업데이트 완료 (J1: {j1_pos:.2f}, J2: {j2_pos:.2f})")
        except Exception as e:
            self.get_logger().error(f"Firebase 업데이트 중 오류 발생: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = DoosanFirebaseNode()
    
    try:
        # 노드를 실행 상태로 유지하며 콜백 함수 대기
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("프로그램을 종료합니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()