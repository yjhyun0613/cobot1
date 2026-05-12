import rclpy
from rclpy.node import Node
import sys
import time
import firebase_admin
from firebase_admin import credentials, db

# 💡 1. 서비스 메시지 타입을 GetExternalTorque로 변경
from dsr_msgs2.srv import GetExternalTorque

ROBOT_ID = "dsr01"

class ExternalTorqueNode(Node):
    def __init__(self):
        super().__init__('external_torque_node')
        
        # Firebase 초기화 (사용자 설정 경로 유지)
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

        # 💡 2. 서비스 클라이언트 주소와 타입을 외력용으로 변경
        self.cli = self.create_client(GetExternalTorque, '/dsr01/aux_control/get_external_torque')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('로봇 외력 서비스 연결 대기 중...')
            
        self.get_logger().info('✅ 외력 감지 모드 가동! 충격 및 걸림을 실시간 감시합니다.')
        
        # 💡 3. 요청서 타입을 GetExternalTorque용으로 변경
        self.req = GetExternalTorque.Request()
        
        # 20Hz (0.05초) 간격으로 서비스 호출
        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        future = self.cli.call_async(self.req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            
            # 💡 4. 데이터 접근 이름을 'ext_torque'로 변경 (사용자 확인 데이터 반영)
            if hasattr(response, 'ext_torque'):
                torques = response.ext_torque
                
                if len(torques) >= 6:
                    # Firebase 업데이트 (동일 경로 유지하여 HTML 그래프와 연동)
                    self.db_ref.set({
                        'j1': float(torques[0]),
                        'j2': float(torques[1]),
                        'j3': float(torques[2]),
                        'j4': float(torques[3]),
                        'j5': float(torques[4]),
                        'j6': float(torques[5]),
                        'timestamp': int(time.time() * 1000)
                    })
                    
                    # # 터미널 출력 (외력은 평상시 0에 가깝습니다)
                    # sys.stdout.write(
                    #     f"\r🛡️ [실시간 외력(Nm)] "
                    #     f"J1: {torques[0]:6.2f} | J2: {torques[1]:6.2f} | J3: {torques[2]:6.2f} | "
                    #     f"J4: {torques[3]:6.2f} | J5: {torques[4]:6.2f} | J6: {torques[5]:6.2f}   "
                    # )
                    # sys.stdout.flush()
        except Exception as e:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = ExternalTorqueNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 외력 모니터링을 종료합니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()