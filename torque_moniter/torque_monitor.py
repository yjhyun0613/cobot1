import rclpy
from rclpy.node import Node
import sys
from dsr_msgs2.srv import GetJointTorque

class TorqueServiceMonitor(Node):
    def __init__(self):
        super().__init__('torque_service_monitor_node')
        
        self.get_logger().info('📡 토크 서비스 서버를 찾는 중...')
        
        self.cli = self.create_client(GetJointTorque, '/dsr01/aux_control/get_joint_torque')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('로봇 서비스 연결 대기 중...')
            
        self.get_logger().info('✅ 직통 서비스 연결 성공! 고속 터미널 모니터링을 시작합니다.')
        
        self.req = GetJointTorque.Request()
        
        # 50Hz (0.02초) 간격으로 서비스 콜 날리기
        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        # 비동기로 콜을 날립니다.
        future = self.cli.call_async(self.req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            # 로봇의 응답 결과 받기
            response = future.result()
            
            # 💡 문제가 되었던 .success 체크를 없애고 바로 토크 배열을 찾습니다.
            if hasattr(response, 'joint_torque'):
                torques = response.joint_torque
            elif hasattr(response, 'jxt'): # 두산 구버전 호환용 이름
                torques = response.jxt
            else:
                # 만약 예상치 못한 구조라면 화면에 강제로 프린트해서 보여줍니다.
                print(f"\n[디버그] 응답 데이터 확인용: {response}")
                return
                
            if len(torques) >= 6:
                # 터미널 화면 제자리 덮어쓰기
                sys.stdout.write(
                    f"\r🦾 [실시간 토크(Nm)] "
                    f"J1: {torques[0]:6.2f} | "
                    f"J2: {torques[1]:6.2f} | "
                    f"J3: {torques[2]:6.2f} | "
                    f"J4: {torques[3]:6.2f} | "
                    f"J5: {torques[4]:6.2f} | "
                    f"J6: {torques[5]:6.2f}   "
                )
                sys.stdout.flush()
                
        except Exception as e:
            # 💡 에러를 조용히 숨기지 않고 빨간 글씨로 터미널에 찍어줍니다.
            self.get_logger().error(f"데이터 처리 중 에러 발생: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TorqueServiceMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n🛑 모니터링을 종료합니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()