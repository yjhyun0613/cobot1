import rclpy
from rclpy.node import Node
import sys
import time
import firebase_admin
from firebase_admin import credentials, db
from dsr_msgs2.srv import GetJointTorque

ROBOT_ID = "dsr01"

class TorqueFirebaseNode(Node):
    def __init__(self):
        super().__init__('torque_firebase_node')
        
        # 1. 파이어베이스 초기화 (사용자님의 기존 JSON 키 경로 적용)
        try:
            cred = credentials.Certificate('/home/yoon/cobot_ws/src/cobot1/rokey-d-2-4c32a-firebase-adminsdk-fbsvc-7f5d874f48.json')
            if not firebase_admin._apps:
                firebase_admin.initialize_app(cred, {
                    'databaseURL': 'https://rokey-d-2-4c32a-default-rtdb.asia-southeast1.firebasedatabase.app'
                })
            self.db_ref = db.reference(f'robot/{ROBOT_ID}/torques')
            self.get_logger().info('✅ Firebase 연결 완료!')
        except Exception as e:
            self.get_logger().error(f'Firebase 연결 실패: {e}')

        # 2. 로봇 서비스 클라이언트 설정
        self.cli = self.create_client(GetJointTorque, '/dsr01/aux_control/get_joint_torque')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('로봇 서비스 연결 대기 중...')
            
        self.get_logger().info('✅ 로봇 연결 성공! 파이어베이스로 실시간 전송을 시작합니다.')
        self.req = GetJointTorque.Request()
        
        # 3. 20Hz(0.05초) 간격으로 타이머 실행
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        future = self.cli.call_async(self.req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            
            # 💡 드디어 찾은 진짜 이름! 'jts'
            if hasattr(response, 'jts'):
                torques = response.jts
                
                if len(torques) >= 6:
                    # Firebase에 덮어쓰기 (DB 용량 낭비 방지)
                    self.db_ref.set({
                        'j1': float(torques[0]),
                        'j2': float(torques[1]),
                        'j3': float(torques[2]),
                        'j4': float(torques[3]),
                        'j5': float(torques[4]),
                        'j6': float(torques[5]),
                        'timestamp': int(time.time() * 1000)
                    })
                    
                    sys.stdout.write(f"\r☁️ Firebase 전송 중... J1: {torques[0]:6.2f} Nm")
                    sys.stdout.flush()
        except Exception as e:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = TorqueFirebaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 전송을 종료합니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()