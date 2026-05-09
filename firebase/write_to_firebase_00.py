import rclpy
from rclpy.node import Node

# 두산 ROS 2 패키지의 DRL 서비스 메시지 임포트
from dsr_msgs2.srv import GetCurrentPosx

class DoosanDrlMonitor(Node):
    def __init__(self):
        super().__init__('doosan_drl_monitor')

        # 1. get_current_posx 서비스 클라이언트 생성
        self.posx_client = self.create_client(
            GetCurrentPosx, 
            '/dsr01/system/get_current_posx'
        )

        # 서비스가 연결될 때까지 대기
        while not self.posx_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('로봇 좌표 서비스 대기 중... (로봇이 켜져 있는지 확인하세요)')

        self.get_logger().info("DRL 서비스를 이용한 좌표 모니터링을 시작합니다. (Ctrl+C 종료)")

        # 2. 0.5초마다 로봇에게 좌표를 달라고 요청(Request)하는 타이머
        self.timer = self.create_timer(0.5, self.request_current_pos)

    def request_current_pos(self):
        # 서비스 요청 객체 생성
        req = GetCurrentPosx.Request()
        req.ref = 0  # 기준 좌표계 설정 (0: Base 기준, 1: Tool 기준, 2: World 기준)

        # 로봇 제어기에게 비동기(Async)로 질문을 던집니다.
        future = self.posx_client.call_async(req)
        
        # 답변이 오면 posx_callback 함수를 실행하도록 연결
        future.add_done_callback(self.posx_callback)

    def posx_callback(self, future):
        try:
            # 로봇 제어기로부터 온 답변(Response)을 까봅니다.
            response = future.result()
            
            if response.success:
                # response.task_pos_info 리스트 안에 [X, Y, Z, A, B, C] 값이 순서대로 들어있습니다.
                pos = response.task_pos_info
                
                print("\n" + "="*50)
                print("[ DRL Service: get_current_posx() ]")
                print(f" X: {pos[0]:>8.3f} mm  |  A (Roll) : {pos[3]:>8.3f} deg")
                print(f" Y: {pos[1]:>8.3f} mm  |  B (Pitch): {pos[4]:>8.3f} deg")
                print(f" Z: {pos[2]:>8.3f} mm  |  C (Yaw)  : {pos[5]:>8.3f} deg")
                print("="*50)
            else:
                self.get_logger().warn("좌표를 가져오는데 실패했습니다.")
                
        except Exception as e:
            self.get_logger().error(f'서비스 호출 중 오류 발생: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DoosanDrlMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n모니터링을 종료합니다.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()